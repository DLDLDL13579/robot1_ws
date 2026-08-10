#!/usr/bin/env python3
"""
Relay /map (from robot_0) → /robot_1/map (for robot_1 AMCL + costmaps).
Run on robot_1 machine, same ROS_DOMAIN_ID as robot_0.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapRelay(Node):
    def __init__(self):
        super().__init__("map_relay")
        # Subscribe to main car's /map (Transient Local, so we get it even if published before we start)
        from rclpy.qos import QoSProfile, DurabilityPolicy
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.sub = self.create_subscription(OccupancyGrid, "/map", self._cb, map_qos)
        self.pub = self.create_publisher(OccupancyGrid, "/robot_1/map", map_qos)
        self.get_logger().info("Relaying /map → /robot_1/map")

    def _cb(self, msg: OccupancyGrid):
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(MapRelay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
