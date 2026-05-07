#!/usr/bin/env python3
"""
Oxford Robotcar dataset player for FusionCore benchmarking.

Reads Oxford Robotcar INS/GPS CSV files and publishes as ROS 2 sensor topics
with simulated clock.

Oxford Robotcar download: https://robotcar-dataset.robots.ox.ac.uk/

Expected directory layout (one sequence):
  <seq>/
    gps/
      ins.csv       -- RTK GPS/INS solution (GROUND TRUTH)
      gps.csv       -- raw GPS only (optional; used as sensor input if present)

ins.csv columns:
  timestamp, latitude, longitude, altitude,
  northing, easting, down,
  velocity_north, velocity_east, velocity_down,
  roll, pitch, yaw

  - timestamp : Unix microseconds
  - lat/lon   : decimal degrees
  - altitude  : meters above WGS84 ellipsoid
  - velocity  : NED frame, m/s
  - angles    : NED frame, radians (yaw clockwise from North)

gps.csv columns:
  timestamp, latitude, longitude, altitude, accuracy

IMU synthesis:
  Angular velocity is derived by differentiating roll/pitch/yaw and converting
  NED -> ENU body frame. Linear acceleration is derived from NED velocity
  derivatives, rotated to body frame, with gravity added (+9.81 m/s^2 on z).
  This matches what a real body-frame accelerometer measures (specific force).

Wheel odometry synthesis:
  Forward velocity is projected from v_north/v_east onto the heading vector.
  Yaw rate is taken from d(yaw)/dt with NED->ENU sign correction.

GPS sensor input:
  If gps/gps.csv exists, it is used as the raw GPS sensor input.
  Otherwise, ins.csv positions are used with added Gaussian noise (noise_m sigma).

Usage:
  ros2 run fusioncore_datasets robotcar_player.py \
    --ros-args -p data_dir:=/path/to/robotcar/<sequence> \
               -p playback_rate:=3.0

Optional test modes:
  gps_spike_time_s      : inject a 500m spike at this sim-time (default -1 = off)
  gps_spike_magnitude_m : spike size in meters (default 500.0)
  gps_outage_start_s    : begin GPS blackout at this sim-time (default -1 = off)
  gps_outage_duration_s : outage length in seconds (default 45.0)
"""

import csv
import math
import os
import random
import threading
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry


def _euler_to_quat(roll, pitch, yaw):
    """ZYX Euler (rad) -> quaternion (w, x, y, z)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


def _wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _utime_to_ros(utime_us):
    ns = utime_us * 1000
    t = Time()
    t.sec = int(ns // 1_000_000_000)
    t.nanosec = int(ns % 1_000_000_000)
    return t


def _enu_yaw_from_ned(yaw_ned):
    """Convert NED yaw (CW from North) to ENU yaw (CCW from East)."""
    return _wrap(math.pi / 2.0 - yaw_ned)


class RobotcarPlayer(Node):

    def __init__(self):
        super().__init__('robotcar_player')

        self.declare_parameter('data_dir', '')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('duration_s', 0.0)
        self.declare_parameter('gps_noise_m', 3.0)
        self.declare_parameter('gps_spike_time_s', -1.0)
        self.declare_parameter('gps_spike_magnitude_m', 500.0)
        self.declare_parameter('gps_outage_start_s', -1.0)
        self.declare_parameter('gps_outage_duration_s', 45.0)

        data_dir = self.get_parameter('data_dir').value
        if not data_dir:
            raise RuntimeError('robotcar_player: data_dir parameter is required')

        self._rate        = self.get_parameter('playback_rate').value
        self._duration    = self.get_parameter('duration_s').value
        self._gps_noise   = self.get_parameter('gps_noise_m').value
        self._spike_time  = self.get_parameter('gps_spike_time_s').value
        self._spike_mag   = self.get_parameter('gps_spike_magnitude_m').value
        self._outage_start = self.get_parameter('gps_outage_start_s').value
        self._outage_dur  = self.get_parameter('gps_outage_duration_s').value

        self._clock_pub = self.create_publisher(Clock,     '/clock',       10)
        self._imu_pub   = self.create_publisher(Imu,       '/imu/data',    100)
        self._gps_pub   = self.create_publisher(NavSatFix, '/gnss/fix',    20)
        self._odom_pub  = self.create_publisher(Odometry,  '/odom/wheels', 100)

        ins_path = os.path.join(data_dir, 'gps', 'ins.csv')
        gps_path = os.path.join(data_dir, 'gps', 'gps.csv')

        self.get_logger().info(f'Loading Robotcar data from: {data_dir}')

        ins_rows = self._load_ins(ins_path)
        gps_rows = self._load_gps_csv(gps_path) if os.path.exists(gps_path) else None

        if gps_rows:
            self.get_logger().info(
                f'  GPS: using gps/gps.csv ({len(gps_rows)} fixes)')
        else:
            self.get_logger().info(
                f'  GPS: synthesizing from ins.csv + {self._gps_noise:.1f}m noise '
                f'(gps/gps.csv not found)')

        self._events = []
        self._build_imu_events(ins_rows)
        self._build_odom_events(ins_rows)
        self._build_gps_events(ins_rows, gps_rows)

        self._events.sort(key=lambda e: e[0])
        self.get_logger().info(
            f'Total events: {len(self._events)} '
            f'({self._count("imu")} IMU, {self._count("odom")} odom, '
            f'{self._count("gps")} GPS)  rate={self._rate}x')

        threading.Thread(target=self._play, daemon=True).start()

    def _load_ins(self, path):
        rows = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or line.startswith('timestamp'):
                    continue
                parts = line.split(',')
                if len(parts) < 13:
                    continue
                try:
                    rows.append({
                        'ts':    int(parts[0]),
                        'lat':   float(parts[1]),
                        'lon':   float(parts[2]),
                        'alt':   float(parts[3]),
                        'v_n':   float(parts[7]),
                        'v_e':   float(parts[8]),
                        'v_d':   float(parts[9]),
                        'roll':  float(parts[10]),
                        'pitch': float(parts[11]),
                        'yaw':   float(parts[12]),
                    })
                except (ValueError, IndexError):
                    continue
        rows.sort(key=lambda r: r['ts'])
        self.get_logger().info(f'  INS: {len(rows)} rows from gps/ins.csv')
        return rows

    def _load_gps_csv(self, path):
        rows = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or line.startswith('timestamp'):
                    continue
                parts = line.split(',')
                if len(parts) < 4:
                    continue
                try:
                    rows.append({
                        'ts':  int(parts[0]),
                        'lat': float(parts[1]),
                        'lon': float(parts[2]),
                        'alt': float(parts[3]),
                        'acc': float(parts[4]) if len(parts) > 4 else 5.0,
                    })
                except (ValueError, IndexError):
                    continue
        rows.sort(key=lambda r: r['ts'])
        return rows

    def _build_imu_events(self, ins_rows):
        """
        Synthesize IMU from ins.csv orientation and velocity.

        Angular velocity: differentiate NED Euler angles, convert to ENU body frame.
          omega_x_enu =  d_roll      (about forward axis, sign same NED/ENU)
          omega_y_enu = -d_pitch     (about lateral axis, y-right vs y-left flip)
          omega_z_enu = -d_yaw       (about vertical axis, z-down vs z-up flip)

        Linear acceleration: differentiate NED velocity, rotate to ENU body frame,
        add gravity (+9.81 on z). Matches raw specific-force output of an accelerometer.
        """
        count = 0
        for i in range(1, len(ins_rows)):
            r    = ins_rows[i]
            prev = ins_rows[i - 1]
            dt = (r['ts'] - prev['ts']) / 1e6
            if dt <= 0.0 or dt > 0.5:
                continue

            d_roll  = _wrap(r['roll']  - prev['roll'])  / dt
            d_pitch = _wrap(r['pitch'] - prev['pitch']) / dt
            d_yaw   = _wrap(r['yaw']   - prev['yaw'])   / dt

            wx =  d_roll
            wy = -d_pitch
            wz = -d_yaw

            dvn = (r['v_n'] - prev['v_n']) / dt
            dve = (r['v_e'] - prev['v_e']) / dt
            dvd = (r['v_d'] - prev['v_d']) / dt

            yaw = r['yaw']

            ax_ned = dvn
            ay_ned = dve
            az_ned = dvd

            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            ax_body =  ax_ned * cos_y + ay_ned * sin_y
            ay_body = -ax_ned * sin_y + ay_ned * cos_y
            az_body = -az_ned

            ax_body_enu =  ax_body
            ay_body_enu = -ay_body
            az_body_enu =  az_body + 9.81

            roll_enu  = r['roll']
            pitch_enu = -r['pitch']
            yaw_enu   = _enu_yaw_from_ned(r['yaw'])

            self._events.append((r['ts'], 'imu', [wx, wy, wz,
                                                   ax_body_enu, ay_body_enu, az_body_enu,
                                                   roll_enu, pitch_enu, yaw_enu]))
            count += 1
        self.get_logger().info(f'  IMU: {count} synthesized events')

    def _build_odom_events(self, ins_rows):
        """
        Synthesize wheel odometry from forward velocity component.
          v_forward = v_north * cos(yaw_ned) + v_east * sin(yaw_ned)
          omega_z   = -d_yaw_ned/dt   (NED->ENU sign flip)
        """
        count = 0
        for i in range(1, len(ins_rows)):
            r    = ins_rows[i]
            prev = ins_rows[i - 1]
            dt = (r['ts'] - prev['ts']) / 1e6
            if dt <= 0.0 or dt > 0.5:
                continue

            yaw       = r['yaw']
            v_forward = r['v_n'] * math.cos(yaw) + r['v_e'] * math.sin(yaw)
            d_yaw     = _wrap(r['yaw'] - prev['yaw']) / dt
            omega_z   = -d_yaw

            self._events.append((r['ts'], 'odom', [v_forward, omega_z]))
            count += 1
        self.get_logger().info(f'  Odom: {count} synthesized events')

    def _build_gps_events(self, ins_rows, gps_rows):
        """
        Build GPS events from gps/gps.csv if available, else from ins with noise.
        Publish at ~1 Hz (sub-sample the ins rows if synthesizing).
        """
        count = 0
        if gps_rows:
            for r in gps_rows:
                acc  = r['acc']
                var  = acc * acc
                self._events.append((r['ts'], 'gps', [r['lat'], r['lon'], r['alt'], var]))
                count += 1
        else:
            rng = random.Random(42)
            prev_ts = None
            for r in ins_rows:
                if prev_ts is not None and (r['ts'] - prev_ts) < 900_000:
                    continue
                prev_ts = r['ts']
                lat_noise = rng.gauss(0, self._gps_noise / 111_111.0)
                lon_noise = rng.gauss(0, self._gps_noise / (111_111.0 * math.cos(math.radians(r['lat']))))
                alt_noise = rng.gauss(0, self._gps_noise * 1.5)
                var = self._gps_noise ** 2
                self._events.append((r['ts'], 'gps', [
                    r['lat'] + lat_noise,
                    r['lon'] + lon_noise,
                    r['alt'] + alt_noise,
                    var,
                ]))
                count += 1
        self.get_logger().info(f'  GPS: {count} events')

    def _count(self, kind):
        return sum(1 for e in self._events if e[1] == kind)

    def _play(self):
        if not self._events:
            self.get_logger().error('No events loaded')
            return

        sim_start_us = self._events[0][0]
        wall_start   = time.monotonic()
        spike_fired  = False

        for utime, kind, data in self._events:
            sim_elapsed_s = (utime - sim_start_us) / 1e6

            if self._duration > 0 and sim_elapsed_s > self._duration:
                break

            sleep_s = wall_start + sim_elapsed_s / self._rate - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

            ros_time = _utime_to_ros(utime)

            clk = Clock()
            clk.clock = ros_time
            self._clock_pub.publish(clk)

            if kind == 'imu':
                self._pub_imu(ros_time, data)
            elif kind == 'odom':
                self._pub_odom(ros_time, data)
            elif kind == 'gps':
                if self._outage_start >= 0:
                    outage_end = self._outage_start + self._outage_dur
                    if self._outage_start <= sim_elapsed_s <= outage_end:
                        continue

                if self._spike_time >= 0 and not spike_fired:
                    if sim_elapsed_s >= self._spike_time:
                        lat, lon, alt, var = data
                        dlat = self._spike_mag / 111_111.0
                        self._pub_gps(ros_time, [lat + dlat, lon, alt, var])
                        self.get_logger().warn(
                            f'GPS spike injected at t={sim_elapsed_s:.1f}s '
                            f'({self._spike_mag:.0f}m offset)')
                        spike_fired = True
                        continue

                self._pub_gps(ros_time, data)

        self.get_logger().info('Playback complete.')

    def _pub_imu(self, ros_time, data):
        wx, wy, wz, ax, ay, az, roll, pitch, yaw = data
        msg = Imu()
        msg.header.stamp    = ros_time
        msg.header.frame_id = 'imu_link'

        w, qx, qy, qz = _euler_to_quat(roll, pitch, yaw)
        msg.orientation.w = w
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation_covariance = [
            1e-4, 0.0,  0.0,
            0.0,  1e-4, 0.0,
            0.0,  0.0,  1e6,
        ]

        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz
        msg.angular_velocity_covariance = [
            2.5e-5, 0.0,    0.0,
            0.0,    2.5e-5, 0.0,
            0.0,    0.0,    2.5e-5,
        ]

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01,
        ]
        self._imu_pub.publish(msg)

    def _pub_gps(self, ros_time, data):
        lat, lon, alt, var = data
        msg = NavSatFix()
        msg.header.stamp    = ros_time
        msg.header.frame_id = 'gnss_link'
        msg.status.status   = NavSatStatus.STATUS_FIX
        msg.status.service  = NavSatStatus.SERVICE_GPS
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = alt
        msg.position_covariance = [
            var,  0.0,  0.0,
            0.0,  var,  0.0,
            0.0,  0.0,  var * 4.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(msg)

    def _pub_odom(self, ros_time, data):
        v_forward, omega_z = data
        msg = Odometry()
        msg.header.stamp    = ros_time
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        msg.twist.twist.linear.x  = v_forward
        msg.twist.twist.angular.z = omega_z
        msg.twist.covariance[0]   = 0.04
        msg.twist.covariance[35]  = 0.001
        self._odom_pub.publish(msg)


def main():
    rclpy.init()
    node = RobotcarPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
