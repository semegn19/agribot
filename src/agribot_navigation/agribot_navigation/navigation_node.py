#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None

        # Waypoints (map frame). Replace coordinates to match your farm rows.
        self.declare_parameter('waypoints', [
            {'x': 1.0, 'y': 1.0, 'w': 1.0},
            {'x': 3.0, 'y': 6.0, 'w': 1.0},
            {'x': 6.0, 'y': 3.0, 'w': 1.0}
        ])
        wp_param = self.get_parameter('waypoints').get_parameter_value().string_array_value
        # We won't parse string_array here — keep the hardcoded list above for simplicity.
        self.waypoints = [(1.0,1.0,1.0), (3.0,6.0,1.0), (6.0,3.0,1.0)]

        self.current_wp = 0

        # Expose a simple cancel service so other nodes can request goal cancellation
        self.create_service(Trigger, 'cancel_nav_goal', self.cancel_service_cb)

        self.get_logger().info('Navigation node started, waiting for action server...')
        self._action_client.wait_for_server()
        self.send_next_goal()

    def build_goal(self, x, y, w=1.0):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.w = float(w)
        return p

    def send_next_goal(self):
        if self.current_wp >= len(self.waypoints):
            self.get_logger().info('All waypoints completed.')
            return
        x,y,w = self.waypoints[self.current_wp]
        goal = NavigateToPose.Goal()
        goal.pose = self.build_goal(x,y,w)
        self.get_logger().info(f'Sending goal #{self.current_wp} -> x={x} y={y}')
        fut = self._action_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        fut.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            return
        self._goal_handle = goal_handle
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self.get_result_cb)
        self.get_logger().info('Goal accepted.')

    def feedback_cb(self, feedback_msg):
        # optional: examine feedback_msg.feedback
        pass

    def get_result_cb(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # canceled
            self.get_logger().info('Goal was canceled.')
        elif status == 5:  # aborted
            self.get_logger().warn('Goal aborted.')
        else:
            self.get_logger().info('Goal succeeded (or finished).')
        # Move to next waypoint if there are more
        if status not in (4,):  # even if canceled, you might decide to not continue
            self.current_wp += 1
            self.send_next_goal()
        self._goal_handle = None

    def cancel_service_cb(self, request, response):
        if self._goal_handle is None:
            response.success = False
            response.message = 'No active goal to cancel'
            self.get_logger().info('Cancel requested but no active goal.')
            return response
        self.get_logger().info('Cancel service called — cancelling current goal...')
        cancel_fut = self._goal_handle.cancel_goal_async()
        # optionally wait a little or attach callback; we just respond immediately
        response.success = True
        response.message = 'Cancel request sent'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

