#!/usr/bin/env python3
# Accumulates nav_msgs/Odometry into nav_msgs/Path for clean trajectory
# lines in RViz. Subscribes to three sources and publishes three paths.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

MAX_POSES = 5000


class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")

        self._pub_fc  = self.create_publisher(Path, "/fusion/path", 10)
        self._pub_rl  = self.create_publisher(Path, "/rl/path",     10)
        self._pub_gps = self.create_publisher(Path, "/gps/path",    10)

        self._path_fc  = Path(); self._path_fc.header.frame_id  = "odom"
        self._path_rl  = Path(); self._path_rl.header.frame_id  = "odom"
        self._path_gps = Path(); self._path_gps.header.frame_id = "odom"

        self.create_subscription(
            Odometry, "/fusion/odom",
            lambda m: self._append(m, self._pub_fc,  self._path_fc),  10)
        self.create_subscription(
            Odometry, "/odometry/filtered",
            lambda m: self._append(m, self._pub_rl,  self._path_rl),  10)
        self.create_subscription(
            Odometry, "/gps/odometry",
            lambda m: self._append(m, self._pub_gps, self._path_gps), 10)

    def _append(self, odom_msg: Odometry, pub, path: Path):
        ps = PoseStamped()
        ps.header = odom_msg.header
        ps.pose   = odom_msg.pose.pose
        path.header.stamp = odom_msg.header.stamp
        path.poses.append(ps)
        if len(path.poses) > MAX_POSES:
            path.poses.pop(0)
        pub.publish(path)


def main():
    rclpy.init()
    rclpy.spin(PathPublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
