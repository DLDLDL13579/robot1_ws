#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.declare_parameter('robot_id', 'robot_1')
        self.robot_id = self.get_parameter('robot_id').value
        # AMCL publishes amcl_pose with TRANSIENT_LOCAL + RELIABLE QoS
        # Must match or be compatible to receive messages
        amcl_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.pose_cb, amcl_pose_qos)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, 'soldier_pose', 10)
        self.get_logger().info(f'Soldier [{self.robot_id}] position publisher ready')

    def pose_cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
