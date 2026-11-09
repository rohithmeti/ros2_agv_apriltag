"""Tag localisation: robot pose in `map` from the tags the cameras see.

map->base = (map->tag) * inverse(base->tag seen by camera).  Wheel odometry carries the pose between tags,
so the node only publishes the correction  map->odom.
In : /agv/tag_detections, TF base->det_<cam>_<id>, TF odom->base, TF map->tag_<id> (tag_map_node)
Out: TF map->odom"""
import json
import math
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from std_msgs.msg import String
from .geom import wrap, tf_to_mat, mat_to_tf, inv, mat_from_xyyaw, xyyaw


class TagLocalizer(Node):
    def __init__(self):
        super().__init__('tag_localizer')
        p = self.declare_parameter
        self.base, self.odom, self.map = p('base_frame', 'base_footprint').value, p('odom_frame', 'odom').value, p('map_frame', 'map').value
        self.front_max_range = p('front_max_range', 2.5).value      # m: far front-camera poses are noisy
        self.gain = {'down': p('down_gain', 1.0).value, 'front': p('front_gain', 0.5).value}
        self.jump = p('jump_reject', 0.6).value                     # m: reject a fix this far from the current estimate...
        self.jump_n = p('jump_accept_after', 8).value               # ...unless it repeats this many times
        self.min_up = p('min_tag_up', 0.7).value                    # tag z axis must point up (cos)
        self.buf = Buffer()
        TransformListener(self.buf, self, spin_thread=True)
        self.tfb = TransformBroadcaster(self)
        self.off = None                                             # (x, y, yaw) of map->odom
        self.rejects = 0
        self.last_fix = None
        self.dropped = 0
        self.pending = deque(maxlen=20)       # detections whose TF has not arrived yet are retried
        self.create_subscription(String, '/agv/tag_detections', lambda m: self.pending.append((time.monotonic(), m)), 10)
        self.create_timer(0.05, self.tick)

    def tick(self):
        keep = deque(maxlen=20)
        while self.pending:
            t0, m = self.pending.popleft()
            r = self.on_det(m)
            if r is False:
                if time.monotonic() - t0 < 1.0:                  # TF not there yet: try again next tick
                    keep.append((t0, m))
                else:
                    self.dropped += 1
                    self.get_logger().warn(f'detection dropped, its TF never arrived ({self.dropped} so far)',
                                           throttle_duration_sec=5.0)
        self.pending = keep
        self.publish()

    def on_det(self, msg):
        """-> False if the TF was not available yet (retry), else True/None."""
        d = json.loads(msg.data)
        cam, stamp = d['cam'], Time(seconds=d['sec'], nanoseconds=d['nsec'])
        best = None
        for t in d['tags']:
            if cam == 'front' and t['range'] > self.front_max_range:
                continue
            if best is None or t['range'] < best['range']:
                best = t                                            # closest tag = most accurate
        if best is None:
            return True
        try:
            T_mt = tf_to_mat(self.buf.lookup_transform(self.map, f'tag_{best["id"]}', Time()))
            T_bt = tf_to_mat(self.buf.lookup_transform(self.base, f'det_{cam}_{best["id"]}', stamp,
                                                       ))
            T_ob = tf_to_mat(self.buf.lookup_transform(self.odom, self.base, stamp))
        except Exception as e:
            self.get_logger().debug(f'tag fix waiting: {e}')
            return False
        if T_bt[2, 2] < self.min_up:                                # tag normal (z) vs base up: flipped/grazing pose
            return True
        T_mb = T_mt @ inv(T_bt)
        bx, by, byaw = xyyaw(T_mb)
        ox, oy, oyaw = xyyaw(T_ob)
        yaw = wrap(byaw - oyaw)
        c, s = math.cos(yaw), math.sin(yaw)
        new = (bx - (c * ox - s * oy), by - (s * ox + c * oy), yaw)
        if self.off is None:
            self.off = new
            self.get_logger().info(f'localised from tag {best["id"]} ({cam}): robot at ({bx:.2f}, {by:.2f}, {math.degrees(byaw):.0f} deg)')
        else:
            # a down-camera fix (tag centred under the robot, <0.2 m) is exact: always accept it, even a big jump,
            # so a drifted estimate is repaired by the next tag the robot drives over. Far/front fixes still get the jump check.
            if cam != 'down' and math.hypot(new[0] - self.off[0], new[1] - self.off[1]) > self.jump:
                self.rejects += 1
                if self.rejects < self.jump_n:
                    return True
            self.rejects = 0
            a = self.gain[cam]
            self.off = (self.off[0] + a * (new[0] - self.off[0]), self.off[1] + a * (new[1] - self.off[1]),
                        wrap(self.off[2] + a * wrap(new[2] - self.off[2])))
        self.last_fix = (best['id'], cam, self.get_clock().now())
        return True

    def publish(self):
        if self.off is not None:
            self.tfb.sendTransform(mat_to_tf(mat_from_xyyaw(*self.off), self.map, self.odom, self.get_clock().now().to_msg()))


def main():
    rclpy.init()
    rclpy.spin(TagLocalizer())
