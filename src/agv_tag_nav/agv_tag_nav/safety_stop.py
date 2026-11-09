"""Lidar safety gate between the navigator and the wheels.

In : /cmd_vel_nav (Twist, from the navigator), /scan (LaserScan), TF scan frame -> base
Out: /cmd_vel (filtered), /agv/obstacle (Bool), /agv/safety_viz (footprint + front stop zone for RViz)

Footprint = the robot body rectangle (0.40 x 0.30 m, centred on base_footprint, from the URDF) + padding.
 * driving forward : a corridor as wide as the footprint ahead of the nose; length = braking distance
                     (margin + v*latency + v^2/2a). Obstacle inside -> v = 0, inside the slow band -> v scaled down.
 * rotating        : the footprint is swept by the commanded rotation; a scan point in the swept area blocks rotation.
 * speed is capped (max_linear 0.5 m/s, max_angular 1.0 rad/s) so the braking distance stays small
 * no scan / no command for a while -> stop (fail-safe).
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from .geom import tf_to_mat


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')
        p = self.declare_parameter
        self.hl = p('footprint_length', 0.40).value / 2             # half length (x)
        self.hw = p('footprint_width', 0.30).value / 2              # half width (y)
        self.pad_side = p('padding_side', 0.05).value               # extra clearance each side while driving
        self.pad_rot = p('padding_rotate', 0.03).value
        self.margin = p('brake_margin', 0.08).value                 # stop this far from the obstacle (beyond the nose)
        self.decel = p('decel', 0.6).value
        self.latency = p('latency', 0.25).value
        self.slow_band = p('slow_band', 0.40).value
        self.max_range = p('max_range', 3.0).value
        self.v_cap = p('max_linear', 0.5).value                     # speed caps: Gazebo's keyboard panel can ask for >1.5 m/s,
        self.w_cap = p('max_angular', 1.0).value                    # whose braking distance (v^2/2a) would exceed 2 m
        self.scan_timeout = p('scan_timeout', 0.5).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.base = p('base_frame', 'base_footprint').value
        self.buf = Buffer()
        TransformListener(self.buf, self, spin_thread=True)
        self.pts = np.zeros((0, 2))
        self.scan_t = None
        self.cmd, self.cmd_t = Twist(), None
        self.blocked = False
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_cmd, 10)
        self.out = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obs = self.create_publisher(Bool, '/agv/obstacle', 10)
        self.viz = self.create_publisher(MarkerArray, '/agv/safety_viz', 1)
        self.create_timer(0.05, self.step)

    def on_scan(self, m):
        try:
            T = tf_to_mat(self.buf.lookup_transform(self.base, m.header.frame_id, Time()))
        except Exception:
            return
        r = np.asarray(m.ranges, np.float64)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, self.max_range))
        r, a = r[ok], a[ok]
        xyz = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r), np.ones_like(r)])
        q = T @ xyz
        self.pts = q[:2].T
        self.scan_t = self.get_clock().now()

    def on_cmd(self, m):
        self.cmd, self.cmd_t = m, self.get_clock().now()

    # ---------- the gate ----------
    def front_distance(self):
        """distance from the nose to the nearest point inside the footprint-wide corridor ahead (inf if none)"""
        P = self.pts
        sel = P[(P[:, 0] > 0) & (np.abs(P[:, 1]) < self.hw + self.pad_side)]
        return float(sel[:, 0].min() - self.hl) if len(sel) else math.inf

    def rotation_blocked(self, w):
        """does rotating by up to ~12 deg in the commanded direction sweep the (padded) footprint over a scan point?"""
        P = self.pts
        P = P[np.hypot(P[:, 0], P[:, 1]) < math.hypot(self.hl, self.hw) + self.pad_rot + 0.05]
        if not len(P):
            return False
        s = 1.0 if w > 0 else -1.0
        for d in (0.0, 0.05, 0.10, 0.20):                          # rotate the points back by d in the turn direction
            c, sn = math.cos(-s * d), math.sin(-s * d)
            x, y = c * P[:, 0] - sn * P[:, 1], sn * P[:, 0] + c * P[:, 1]
            if np.any((np.abs(x) < self.hl + self.pad_rot) & (np.abs(y) < self.hw + self.pad_rot)):
                return True
        return False

    def step(self):
        now = self.get_clock().now()
        out, why = Twist(), ''
        scan_ok = self.scan_t is not None and (now - self.scan_t).nanoseconds * 1e-9 < self.scan_timeout
        cmd_ok = self.cmd_t is not None and (now - self.cmd_t).nanoseconds * 1e-9 < self.cmd_timeout
        v, w = (self.cmd.linear.x, self.cmd.angular.z) if cmd_ok else (0.0, 0.0)
        v = max(-self.v_cap, min(self.v_cap, v))
        w = max(-self.w_cap, min(self.w_cap, w))
        stop_d = self.margin + abs(v) * self.latency + v * v / (2 * self.decel)
        if not scan_ok:
            if cmd_ok and (v or w):
                why = 'no lidar data'
            v = w = 0.0
        else:
            if v > 0:
                d = self.front_distance()
                if d <= stop_d:
                    v, why = 0.0, f'obstacle {d:.2f} m ahead'
                elif d < stop_d + self.slow_band:
                    v *= (d - stop_d) / self.slow_band
                    if v < 0.05 * min(self.cmd.linear.x, self.v_cap):                # crawling to a halt = blocked
                        why = f'obstacle {d:.2f} m ahead'
            elif v < 0:                                            # no rear scan guarantee: allow only creeping
                v = max(v, -0.05)
            if w != 0.0 and self.rotation_blocked(w):
                w, why = 0.0, why or 'obstacle in the turning area'
        out.linear.x, out.angular.z = float(v), float(w)
        self.out.publish(out)
        blocked = bool(why)
        self.obs.publish(Bool(data=blocked))
        if blocked != self.blocked or blocked:
            if blocked != self.blocked:
                self.get_logger().warn(f'STOP: {why}') if blocked else self.get_logger().info('path clear')
        self.blocked = blocked
        self.draw(stop_d if v >= 0 else 0.0, blocked)

    def draw(self, stop_d, blocked):
        ma = MarkerArray()

        def box(ns, i, x0, x1, hw, rgb):
            m = Marker(type=Marker.LINE_STRIP, ns=ns, id=i)
            m.header.frame_id, m.scale.x = self.base, 0.02
            m.frame_locked = True
            m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
            m.points = [Point(x=x0, y=-hw, z=0.3), Point(x=x1, y=-hw, z=0.3), Point(x=x1, y=hw, z=0.3),
                        Point(x=x0, y=hw, z=0.3), Point(x=x0, y=-hw, z=0.3)]
            return m
        ma.markers.append(box('footprint', 0, -self.hl, self.hl, self.hw, (0.2, 0.6, 1.0)))
        zone = (1.0, 0.1, 0.1) if blocked else (0.1, 0.9, 0.2)
        ma.markers.append(box('stop_zone', 1, self.hl, self.hl + max(stop_d, 0.05), self.hw + self.pad_side, zone))
        self.viz.publish(ma)


def main():
    rclpy.init()
    rclpy.spin(SafetyStop())
