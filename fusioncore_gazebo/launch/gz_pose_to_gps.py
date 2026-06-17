#!/usr/bin/env python3
# Converts Gazebo ground truth pose to NavSatFix (/gnss/fix) and
# nav_msgs/Odometry (/gps/odometry) with configurable GPS spike injection
# for the demo run. Both outputs spike simultaneously so FusionCore and
# robot_localization see the same corrupted measurement.

import math
import random
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry

ORIGIN_LAT = 43.2557
ORIGIN_LON = -79.8711
ORIGIN_ALT = 100.0
A  = 6378137.0
E2 = 0.00669437999014


def enu_to_lla(x, y, z):
    lat0 = math.radians(ORIGIN_LAT)
    lon0 = math.radians(ORIGIN_LON)
    alt0 = ORIGIN_ALT
    sl = math.sin(lat0); cl = math.cos(lat0)
    sn = math.sin(lon0); cn = math.cos(lon0)
    N0 = A / math.sqrt(1.0 - E2 * sl * sl)
    X0 = (N0 + alt0) * cl * cn
    Y0 = (N0 + alt0) * cl * sn
    Z0 = (N0 * (1 - E2) + alt0) * sl
    dX = -sn * x - sl * cn * y + cl * cn * z
    dY =  cn * x - sl * sn * y + cl * sn * z
    dZ =  cl * y + sl * z
    Xp = X0 + dX; Yp = Y0 + dY; Zp = Z0 + dZ
    p = math.sqrt(Xp * Xp + Yp * Yp)
    lat = math.atan2(Zp, p * (1.0 - E2))
    for _ in range(5):
        s = math.sin(lat)
        N = A / math.sqrt(1.0 - E2 * s * s)
        lat = math.atan2(Zp + E2 * N * s, p)
    s = math.sin(lat)
    N = A / math.sqrt(1.0 - E2 * s * s)
    alt = p / math.cos(lat) - N
    return math.degrees(lat), math.degrees(math.atan2(Yp, Xp)), alt


def _is_base_link(frame_id):
    tail = frame_id.rsplit("::", 1)[-1].rsplit("/", 1)[-1]
    return tail == "base_link"


class GzPoseToGps(Node):
    def __init__(self):
        super().__init__("gz_pose_to_gps")

        self.declare_parameter("world_name",        "fusioncore_outdoor")
        self.declare_parameter("noise_h",            0.5)
        self.declare_parameter("noise_v",            0.3)
        self.declare_parameter("spike_at_s",        -1.0)   # <0 disables spike
        self.declare_parameter("spike_duration_s",   6.0)
        self.declare_parameter("spike_dx_m",        50.0)
        self.declare_parameter("spike_dy_m",         0.0)

        world_name = self.get_parameter("world_name").get_parameter_value().string_value

        self.pub_fix  = self.create_publisher(NavSatFix, "/gnss/fix",     10)
        self.pub_odom = self.create_publisher(Odometry,  "/gps/odometry", 10)
        self.sub = self.create_subscription(
            TFMessage, f"/world/{world_name}/pose/info", self.pose_cb, 10)

        self.body_frame_id = None
        self.ref_published = False
        self.start_ns = None

        self.get_logger().info(f"GPS publisher ready (world={world_name})")

    def _spike_active(self):
        spike_at = self.get_parameter("spike_at_s").get_parameter_value().double_value
        if spike_at < 0.0 or self.start_ns is None:
            return False
        elapsed = (self.get_clock().now().nanoseconds - self.start_ns) * 1e-9
        dur = self.get_parameter("spike_duration_s").get_parameter_value().double_value
        return spike_at <= elapsed < (spike_at + dur)

    def _find_body(self, msg):
        if self.body_frame_id is not None:
            for tf in msg.transforms:
                if tf.child_frame_id == self.body_frame_id:
                    t = tf.transform.translation
                    if 0.05 < t.z < 0.4:
                        return t
        for tf in msg.transforms:
            if _is_base_link(tf.child_frame_id):
                t = tf.transform.translation
                if 0.05 < t.z < 0.4:
                    self.body_frame_id = tf.child_frame_id
                    return t
        best = None; best_mag = -1.0; best_fid = None
        for tf in msg.transforms:
            t = tf.transform.translation
            if not (0.05 < t.z < 0.4):
                continue
            mag = t.x * t.x + t.y * t.y
            if mag > best_mag:
                best_mag = mag; best = t; best_fid = tf.child_frame_id
        if best_fid is not None:
            self.body_frame_id = best_fid
        return best

    def pose_cb(self, msg):
        best = self._find_body(msg)
        if best is None:
            return

        noise_h  = self.get_parameter("noise_h").get_parameter_value().double_value
        noise_v  = self.get_parameter("noise_v").get_parameter_value().double_value
        spike_dx = self.get_parameter("spike_dx_m").get_parameter_value().double_value
        spike_dy = self.get_parameter("spike_dy_m").get_parameter_value().double_value

        if self.start_ns is None:
            self.start_ns = self.get_clock().now().nanoseconds

        x = best.x + random.gauss(0, noise_h)
        y = best.y + random.gauss(0, noise_h)
        z = best.z if not self.ref_published else best.z + random.gauss(0, noise_v)

        spike = self._spike_active()
        if spike:
            x += spike_dx
            y += spike_dy
            if not hasattr(self, "_spike_logged"):
                self.get_logger().warn(
                    f"GPS SPIKE ACTIVE: +{spike_dx:.0f} m East, +{spike_dy:.0f} m North")
                self._spike_logged = True
        elif hasattr(self, "_spike_logged"):
            del self._spike_logged
            self.get_logger().info("GPS spike ended, resuming normal fix")

        now = self.get_clock().now().to_msg()

        # NavSatFix for FusionCore (chi2 gate should reject during spike)
        lat, lon, alt = enu_to_lla(x, y, z)
        fix = NavSatFix()
        fix.header.stamp    = now
        fix.header.frame_id = "gnss_link"
        fix.status.status   = NavSatStatus.STATUS_FIX
        fix.status.service  = NavSatStatus.SERVICE_GPS
        fix.latitude  = lat
        fix.longitude = lon
        fix.altitude  = alt
        fix.position_covariance = [
            noise_h ** 2, 0, 0,
            0, noise_h ** 2, 0,
            0, 0, noise_v ** 2,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)

        # ENU Odometry for robot_localization (no navsat_transform_node needed)
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        var = noise_h ** 2
        odom.pose.covariance[0]  = var
        odom.pose.covariance[7]  = var
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e6
        self.pub_odom.publish(odom)

        self.ref_published = True


def main():
    rclpy.init()
    rclpy.spin(GzPoseToGps())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
