import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
import time

class WallFollower(Node):
    def __init__(self):    
        print("Wall Follower Node Started 1")
        super().__init__('wall_follower')

        # Define a QoS profile
# Define QoS profile
        qos = QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        print("Wall Follower Node Started 2")
        # Use the custom QoS profile for the publisher
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel', 10
        )
        print("Wall Follower Node Started 3")

        # Use the custom QoS profile for the subscription to LaserScan messages
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile=qos
        )
        print("Wall Follower Node Started 4")
        self.subscription  # prevent unused variable warning

        # Use the custom QoS profile for the subscription to Odometry messages
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback, 10
        )
        print("Wall Follower Node Started 5")
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
        self.follow_dir = -1
        print("Wall Follower Node Started 6")

    def listener_callback(self, msg:LaserScan):
        laser_range = np.array(msg.ranges)
        self.section = {
            'front': min(min(laser_range[198:]), min(laser_range[:29])),
            'left': min(laser_range[29:86]),
            'right': min(laser_range[141:199]),
        }
        print("Wall Follower Node Started 7")
        self.get_logger().info(f'Front: {self.section["front"]:.2f} | Left: {self.section["left"]:.2f} | Right: {self.section["right"]:.2f}')
        self.bug_action()

    def change_state(self, state):
        print("Wall Follower Node Started 8")
        if state != self.state_:
            self.get_logger().info(f'State of Bot - [{state}] - {self.state_dict_[state]}')
            self.state_ = state
        

    def bug_action(self):
        print("Wall Follower Node Started 9")
        wall_detection_threshold = 1
        too_close_threshold = 0.3
        
        if self.section['front'] < too_close_threshold:
            if self.section['left'] < self.section['right']:
                self.change_state(1)  # Turn right if left side is closer to the wall
            else:
                self.change_state(3)  # Turn left if right side is closer to the wall
        elif self.section['left'] < too_close_threshold and self.follow_dir != 1:
            self.change_state(1)  # Turn right when too close to a wall on the left
        elif self.section['right'] < too_close_threshold and self.follow_dir != 0:
            self.change_state(3)  # Turn left when too close to a wall on the right
        elif self.section['front'] > wall_detection_threshold and self.section['left'] > wall_detection_threshold and self.section['right'] > wall_detection_threshold:
            self.change_state(0)  # Find wall when there's no wall detected in all directions
        elif self.follow_dir == -1:  # Initial state
            if self.section['left'] < wall_detection_threshold :
                self.change_state(1)  # Turn right if a wall is detected on the left
                self.follow_dir = 0
            elif self.section['right'] < wall_detection_threshold :
                self.change_state(3)  # Turn left if a wall is detected on the right
                self.follow_dir = 1
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
            print(f"{time.time()} Odom message received")
            # Update the current position of the robot
            self.curr_pos = msg.pose.pose.position
        except Exception as e:
            self.get_logger().error(f"Error in odom_callback: {str(e)}")

        
        
        
        if self.init_pos is None:
            # If it's the first reading, initialize the starting position
            self.init_pos = self.curr_pos
            print("Wall Follower Node Started odom?")
        

    def get_distance(self):
        print("Wall Follower Node Started 11")
        # Calculate the distance between the initial and current position
        dx = self.curr_pos.x - self.init_pos.x
        dy = self.curr_pos.y - self.init_pos.y
        return np.sqrt(dx**2 + dy**2)
        

    def take_action(self):
        print("Wall Follower Node Started 12")
        # Initialize a Twist message
        velocity_msg = Twist()
        
        # Check if the robot has traveled 5 meters
        if self.init_pos is not None and self.get_distance() >= 5:
            # If it has, stop moving
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
            self.get_logger().info("Traveled 5 meters. Stopping.")
        else:
            # If it hasn't, proceed with the predefined states and actions
            if self.state_ == 0:  # Find wall
                velocity_msg = self.find_wall()
            elif self.state_ == 1:  # Turn right
                velocity_msg = self.turn_right()
            elif self.state_ == 2:  # Move ahead
                velocity_msg = self.move_ahead()
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


    def find_wall(self):
        print("Wall Follower Node Started 13")
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = 0.0
        return velocity

    def turn_left(self):
        print("Wall Follower Node Started 14")
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = 0.3
        return velocity

    def turn_right(self):
        print("Wall Follower Node Started 15")
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = -0.3
        return velocity

    def move_ahead(self):
        print("Wall Follower Node Started 16")
        velocity = Twist()
        velocity.linear.x = 0.3
        velocity.angular.z = 0.0
        return velocity

    def move_diag_right(self):
        print("Wall Follower Node Started 17")
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = -0.3
        return velocity

    def move_diag_left(self):
        print("Wall Follower Node Started 18")
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = 0.3
        return velocity


def main(args=None):
    print("Wall Follower Node Started 19")
    rclpy.init(args=args)
    
    wall_follower = WallFollower()
    
    rclpy.spin(wall_follower)
    
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
