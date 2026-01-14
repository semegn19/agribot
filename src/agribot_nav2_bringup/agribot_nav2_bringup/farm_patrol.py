import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class OdomPatrol(Node):
    def __init__(self):
        super().__init__('odom_patrol')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = None
        self.is_moving = False
        
        # S-Pattern Logic
        self.row_count = 0
        self.target_distance = 5.0 # Meters to drive down each row
        
        # Create a timer to check progress and send commands
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x = self.current_x

    def control_loop(self):
        if self.start_x is None: return # Wait for first odom message

        # Calculate distance from where we started this row
        distance_traveled = math.sqrt((self.current_x - self.start_x)**2)
        
        msg = Twist()
        if distance_traveled < self.target_distance:
            self.get_logger().info(f'Moving: {distance_traveled:.2f}/{self.target_distance}m', throttle_duration_sec=1.0)
            msg.linear.x = 0.4  # Move forward
        else:
            self.get_logger().info('Target Reached! Stopping.')
            msg.linear.x = 0.0
            # Logic to reset start_x and switch to a turn would go here
            self.start_x = self.current_x # For next segment
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPatrol()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
