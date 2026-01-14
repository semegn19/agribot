#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTfBridge(Node):
    """Subscribe to /odom and publish odom -> base_link transform using odom.pose."""
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, '/odom', self.cb_odom, 10)
        self.get_logger().info('odom_tf_bridge started, relaying /odom -> base_link TF.')

    def cb_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # preserve whatever frame_id the odom message uses (e.g. "agribot/odom")
        t.header.frame_id = 'odom'
        # publish child_frame as plain 'base_link' so robot_state_publisher frames connect
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

