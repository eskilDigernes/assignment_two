import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
import rclpy.qos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
import time
import math

""" wall_follow_state determnines which side of the robot is closer to the wall. """
""" wall_follow_state == -1: Initial state                                       """ 
""" wall_follow_state == 0: Left side                                            """
""" wall_follow_state, 1: Right side                                             """


class WallFollower(Node):
    def __init__(self):    
        super().__init__('wall_follower')

        # Define a QoS profile
        qos = QoSProfile(
            depth=10,  # Small queue depth
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            lifespan=Duration(seconds=0, nanoseconds=0),  # Infinite lifespan
            deadline=Duration(seconds=0, nanoseconds=0),  # Infinite deadline
            liveliness=rclpy.qos.QoSLivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=0, nanoseconds=0)  # Infinite lease duration
        )


        # qos = QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        #print("Wall Follower Node Started 2")
        # Use the custom QoS profile for the publisher
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel', 10
        )


        # Use the custom QoS profile for the subscription to LaserScan messages
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile=qos
        )

        self.subscription  # prevent unused variable warning

        # Use the custom QoS profile for the subscription to Odometry messages
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback, 10
        )

        #self.odom_subscription
        
        # To store the initial and current position of the robot
        self.init_pos = None
        self.curr_pos = None
        self.adjusted_distance = 0.5

        self.state_ = 0
        self.section = {'front': 0, 'left': 0, 'right': 0}
        self.state_dict_ = {
            0: 'Find wall',
            1: 'Turn right',
            2: 'Follow the wall',
            3: 'Turn left',
            4: 'Diagonally right',
            5: 'Diagonally left',

        }
        self.wall_follow_state = -1


    def listener_callback(self, msg:LaserScan):
        laser_range = np.array(msg.ranges)
        print(len(laser_range))
        
        # Default value when the laser range is empty
        default_val = float('inf')  # or any suitable value
        
        # Check for empty laser_range before proceeding
        if len(laser_range) == 0:
            self.get_logger().warn('Laser range is empty')
            return
        
        # Helper function to calculate min safely
        def safe_min(*args, **kwargs):
            try:
                return min(*args, **kwargs)
            except ValueError:
                return default_val
        
        # Define sections and their respective slices
        slices = {
            'front': (slice(207, None), slice(None, 20)),
            'left': slice(30, 86),
            'right': slice(142, 198),
        }
        
        self.section = {}
        for key, slc in slices.items():
            if isinstance(slc, tuple):
                self.section[key] = safe_min(safe_min(laser_range[slc[0]]), safe_min(laser_range[slc[1]]))
            else:
                self.section[key] = safe_min(laser_range[slc])
        
        self.get_logger().info(f'Front: {self.section["front"]:.2f} | Left: {self.section["left"]:.2f} | Right: {self.section["right"]:.2f}')
        self.bug_action()



    def change_state(self, state):
        if state != self.state_:
            self.get_logger().info(f'State of Bot - [{state}] - {self.state_dict_[state]}')
            self.state_ = state
        

    def bug_action(self):
        wall_threshold = 0.4
        min_threshold = 0.3
        
        if self.section['front'] < min_threshold:
            if self.section['left'] < self.section['right']:
                self.change_state(1)  # Turn right if left side is closer to the wall
            else:
                self.change_state(3)  # Turn left if right side is closer to the wall
        elif self.section['left'] < min_threshold and self.wall_follow_state != 1:
            self.change_state(1)  # Turn right when too close to a wall on the left
        elif self.section['right'] < min_threshold and self.wall_follow_state != 0:
            self.change_state(3)  # Turn left when too close to a wall on the right
        elif self.section['front'] > wall_threshold and self.section['left'] > wall_threshold and self.section['right'] > wall_threshold:
            self.change_state(0)  # Find wall when there's no wall detected in all directions
        elif self.wall_follow_state == -1:  # Initial state
            if self.section['left'] < wall_threshold :
                self.change_state(1)  # Turn right if a wall is detected on the left
                self.wall_follow_state = 0
            elif self.section['right'] < wall_threshold :
                self.change_state(3)  # Turn left if a wall is detected on the right
                self.wall_follow_state = 1
            else:
                self.change_state(2)  # Move ahead if no wall is detected
        elif 0.3 < self.section['front'] < 0.6:  # If somewhat close to a front wall but not too close
            if self.section['left'] < self.section['right']:
                self.change_state(5)  # Move diagonally left if left side is closer to the wall
            else:
                self.change_state(4)  # Move diagonally right if right side is closer to the wall
        else:
            self.change_state(2)  # Move ahead
        
        
        self.take_action()

    def odom_callback(self, msg):
        try:
            #print(f"{time.time()} Odom message received")
            # Update the current position of the robot
            self.curr_pos = msg.pose.pose.position
        except Exception as e:
            self.get_logger().error(f"Error in odom_callback: {str(e)}")

        
        
        
        if self.init_pos is None:
            # If it's the first reading, initialize the starting position
            self.init_pos = self.curr_pos

        

    def get_distance(self):

        # Calculate the distance between the initial and current position
        dx = self.curr_pos.x - self.init_pos.x
        dy = self.curr_pos.y - self.init_pos.y
        return np.sqrt(dx**2 + dy**2)
        

    # def take_action(self):
        
    #     # Initialize a Twist message
    #     velocity_msg = Twist()
        
    #     # Check if the robot has traveled 5 meters
    #     if self.init_pos is not None and self.get_distance() >= 5:
    #         # If it has, stop moving
    #         velocity_msg.linear.x = 0.0
    #         velocity_msg.angular.z = 0.0
    #         self.get_logger().info("Traveled 5 meters. Stopping.")
    #     else:
    #         # If it hasn't, proceed with the predefined states and actions
    #         if self.state_ == 0:  # Find wall
    #             velocity_msg = self.find_wall()
    #         elif self.state_ == 1:  # Turn right
    #             velocity_msg = self.turn_right()
    #         elif self.state_ == 2:  # Move ahead
    #             velocity_msg = self.move_ahead()
    #         elif self.state_ == 3:  # Turn left
    #             velocity_msg = self.turn_left()
    #         elif self.state_ == 4:  # Move diagonally right
    #             velocity_msg = self.move_diag_right()
    #         elif self.state_ == 5:  # Move diagonally left
    #             velocity_msg = self.move_diag_left()
    #         else:
    #             self.get_logger().error('Unknown state!')
        
    #     # Publish the velocity message
    #     self.publisher_.publish(velocity_msg)

######################################################
############ WITH P CONTROLLER #######################
######################################################

    def take_action(self):
        
        # Initialize a Twist message
        velocity_msg = Twist()
        
        # Check if the robot has traveled 5 meters
        if self.init_pos is not None and self.get_distance() >= 5:
            # If it has, stop moving
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
            self.get_logger().info("Traveled 5 meters. Stopping.")
        else:
            # Get actual distance from laser scan data
            # Assume that you want to keep the robot a certain distance from the wall to its right
            # You'll need to determine how to calculate this based on your laser scan data
            actual_distance = self.section['right']  # This is an example, you might need to adjust
            
            # Implement P controller in 'Follow the wall' state
            if self.state_ == 2:  # Follow the wall state
                desired_distance = 0.5  # Desired distance from the wall in meters
                control_signal = self.wall_following_P_controller(desired_distance, actual_distance)
                
                # Use control signal to adjust robot's motion
                velocity_msg.linear.x = 0.3  # Keep forward velocity constant
                velocity_msg.angular.z = control_signal  # Adjust angular velocity based on control signal
            else:
                # If it hasn't, proceed with the predefined states and actions
                if self.state_ == 0:  # Find wall
                    velocity_msg = self.find_wall()
                elif self.state_ == 1:  # Turn right
                    velocity_msg = self.turn_right()
                elif self.state_ == 3:  # Turn left
                    velocity_msg = self.turn_left()
                elif self.state_ == 4:  # Move diagonally right
                    velocity_msg = self.move_diag_right()
                elif self.state_ == 5:  # Move diagonally left
                    velocity_msg = self.move_diag_left()
                else:
                    self.get_logger().error('Unknown state!')
        
        # Publish the velocity message
        self.publisher_.publish(velocity_msg)


    def wall_following_P_controller(self, desired_distance, actual_distance):
        Kp = 0.03  # Proportional gain. You will need to tune this value.
        error = desired_distance - actual_distance  # Error between desired and actual distance
        
        # Control signal
        control_signal = Kp * error
        
        return control_signal

##########################################################################
########### P CONTROLLER END #############################################
##########################################################################
    # State machine actions
    def find_wall(self):
        
        velocity = Twist()
        velocity.linear.x = 0.3
        velocity.angular.z = 0.0
        return velocity

    def turn_left(self):
        
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = 0.3
        return velocity

    def turn_right(self):
        
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = -0.3
        return velocity

    def move_ahead(self):
        
        velocity = Twist()
        velocity.linear.x = 0.3
        velocity.angular.z = 0.0
        return velocity

    def move_diag_right(self):
        
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = -0.3
        return velocity

    def move_diag_left(self):
        
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = 0.3
        return velocity


def main(args=None):
    
    rclpy.init(args=args)
    
    wall_follower = WallFollower()
    
    rclpy.spin(wall_follower)
    
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()