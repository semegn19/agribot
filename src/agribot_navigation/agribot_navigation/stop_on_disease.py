#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class StopOnDisease(Node):
    def __init__(self):
        super().__init__('stop_on_disease')
        self.sub = self.create_subscription(String, '/plant_health', self.vision_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cancel_client = self.create_client(Trigger, 'cancel_nav_goal')
        self.get_logger().info('stop_on_disease node started; waiting for cancel service...')
        if not self.cancel_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('cancel_nav_goal service not available (will still publish stop).')

    def vision_cb(self, msg):
        text = msg.data.lower()
        self.get_logger().info(f'Vision event: "{text}"')
        if 'unhealthy' in text or 'diseased' in text or 'disease' in text:
            self.get_logger().warn('Unhealthy plant detected — stopping robot.')
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.cmd_pub.publish(stop)
            # call cancel service if available
            if self.cancel_client.service_is_ready():
                req = Trigger.Request()
                fut = self.cancel_client.call_async(req)
                fut.add_done_callback(lambda f: self.get_logger().info('Cancel service call done.'))
            else:
                self.get_logger().warn('Cancel service not ready; only published immediate stop.')

def main(args=None):
    rclpy.init(args=args)
    node = StopOnDisease()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

