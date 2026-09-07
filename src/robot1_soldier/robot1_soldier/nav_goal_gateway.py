#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NavGoalGateway(Node):
    def __init__(self):
        super().__init__("nav_goal_gateway")
        self._action_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        self._goal_subscription = self.create_subscription(
            PoseStamped, "goal_pose", self._goal_callback, 10
        )
        self._goal_handle = None
        self.get_logger().info("Nav goal gateway ready.")

    def _goal_callback(self, pose: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server is unavailable.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f"Forwarding goal: x={pose.pose.position.x:.2f}, "
            f"y={pose.pose.position.y:.2f}"
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected.")
            return

        self.get_logger().info("Navigation goal accepted.")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        self.get_logger().info(
            f"Navigation completed with status {result.status}."
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalGateway()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()