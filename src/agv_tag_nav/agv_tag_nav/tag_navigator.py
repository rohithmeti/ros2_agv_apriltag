"""Node-to-node AGV follower. AGV rules: stop at every tag node, rotate to the next lane heading, drive the lane.

In : /agv/goal_node (Int32 tag id; -1 = cancel), TF map->base (tag localised), TF base->det_down_<id> (final centring)
Out: /cmd_vel_nav (-> safety_stop -> /cmd_vel), /agv/status (String), /agv/route (String)
Per leg: TURN (in place to lane heading) -> DRIVE (carrot on the lane line, cross-track corrected)
         -> CENTRE (last ~0.3 m: down camera error to the tag centre) -> node reached, stop."""
import json
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped, PointStamped, Point
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import Marker, MarkerArray
from .graph import TagGraph
from .geom import wrap, tf_to_mat, xyyaw


def clamp(v, lim):
    return max(-lim, min(lim, v))


class TagNavigator(Node):
    def __init__(self):
        super().__init__('tag_navigator')
        p = self.declare_parameter
        default = os.path.join(get_package_share_directory('agv_tag_nav'), 'config', 'tag_map.yaml')
        self.g = TagGraph(p('tag_map_file', default).value)
        self.base, self.map = p('base_frame', 'base_footprint').value, p('map_frame', 'map').value
        self.vmax, self.wmax = p('v_max', 0.30).value, p('w_max', 0.8).value
        self.lookahead = p('lookahead', 0.35).value
        self.centre_dist = p('centre_dist', 0.30).value             # switch to down-camera centring inside this
        self.tol = p('node_tol', 0.02).value                        # m, accepted when centred by the down camera
        self.tol_map = p('node_tol_map', 0.06).value                # m, when only the map pose is available
        self.turn_tol = p('turn_tol', 0.05).value
        self.buf = Buffer()
        TransformListener(self.buf, self, spin_thread=True)
        self.cmd = self.create_publisher(Twist, '/cmd_vel_nav', 10)       # safety_stop filters it -> /cmd_vel
        self.status = self.create_publisher(String, '/agv/status', 10)
        self.create_subscription(Int32, '/agv/goal_node', self.on_goal, 10)
        # RViz: "2D Goal Pose" tool (/goal_pose) or "Publish Point" tool (/clicked_point) -> nearest tag is the goal
        self.pick_radius = p('pick_radius', 0.6).value
        self.create_subscription(PoseStamped, '/goal_pose', lambda m: self.pick(m.pose.position.x, m.pose.position.y), 10)
        self.create_subscription(PointStamped, '/clicked_point', lambda m: self.pick(m.point.x, m.point.y), 10)
        self.viz = self.create_publisher(MarkerArray, '/agv/route_viz', 1)
        self.blocked = False
        self.create_subscription(Bool, '/agv/obstacle', lambda m: setattr(self, 'blocked', m.data), 10)
        self.route, self.i, self.state, self.goal = [], 0, 'IDLE', None
        self.create_timer(0.05, self.step)
        self.create_timer(1.0, self.report)

    # ---------- goal -> route ----------
    def pose(self):
        try:
            return xyyaw(tf_to_mat(self.buf.lookup_transform(self.map, self.base, Time())))
        except Exception:
            return None

    def pick(self, x, y):
        n = self.g.nearest(x, y)
        gx, gy, _ = self.g.nodes[n]
        if math.hypot(gx - x, gy - y) > self.pick_radius:
            self.get_logger().warn(f'click ({x:.2f}, {y:.2f}) is not on a tag (nearest {n}, {math.hypot(gx - x, gy - y):.2f} m)')
            return
        self.get_logger().info(f'clicked tag {n}')
        self.on_goal(Int32(data=n))

    def on_goal(self, m):
        if m.data < 0:
            self.stop('IDLE')
            return
        if m.data not in self.g.nodes:
            self.get_logger().warn(f'goal {m.data} is not a node')
            return
        pose = self.pose()
        if pose is None:
            self.get_logger().warn('not localised yet (no tag seen): cannot plan')
            return
        start = self.g.nearest(pose[0], pose[1])
        path, length = self.g.shortest_path(start, m.data)
        if path is None:
            self.get_logger().warn(f'no path {start} -> {m.data}')
            return
        self.route, self.i, self.goal, self.state = path, 0, m.data, 'TURN'
        self.get_logger().info(f'route {path} ({length:.2f} m)')

    # ---------- control ----------
    def drive(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)

    def stop(self, state):
        self.state = state
        self.drive(0.0, 0.0)

    def down_error(self, tid):
        """tag centre in base frame from the down camera (fresh only) -> (ex, ey) or None"""
        try:
            t = self.buf.lookup_transform(self.base, f'det_down_{tid}', Time())
        except Exception:
            return None
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if age > 0.3:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def step(self):
        if self.state == 'IDLE' or not self.route:
            return
        pose = self.pose()
        if pose is None:
            self.stop('LOST')
            return
        x, y, yaw = pose
        tid = self.route[self.i]
        tx, ty, _ = self.g.nodes[tid]
        d = math.hypot(tx - x, ty - y)
        prev = self.g.nodes[self.route[self.i - 1]] if self.i > 0 else None

        if self.state == 'LOST':
            self.state = 'TURN'
        if self.state == 'CENTRE':
            e = self.down_error(tid)
            if e is None:
                if d < self.tol_map:
                    return self.reached(tid)
                err = wrap(math.atan2(ty - y, tx - x) - yaw)           # tag not in view: fall back to map pose
                return self.drive(min(0.08, 0.5 * d) * max(0.0, math.cos(err)), clamp(1.5 * err, self.wmax))
            ex, ey = e
            if math.hypot(ex, ey) < self.tol:
                return self.reached(tid)
            return self.drive(clamp(0.8 * ex, 0.08), clamp(2.0 * ey, 0.3))
        # lane heading: along the lane when we have a previous node, else straight at the target
        if prev is not None and (prev[0], prev[1]) != (tx, ty):
            lx, ly = tx - prev[0], ty - prev[1]
            ln = math.hypot(lx, ly)
            ux, uy = lx / ln, ly / ln
            along = (x - prev[0]) * ux + (y - prev[1]) * uy
            cx, cy = prev[0] + ux * min(ln, max(0.0, along) + self.lookahead), prev[1] + uy * min(ln, max(0.0, along) + self.lookahead)
        else:
            cx, cy = tx, ty
        des = math.atan2(cy - y, cx - x) if d > 0.05 else yaw
        err = wrap(des - yaw)
        if d < self.centre_dist and self.state == 'DRIVE':
            self.stop('CENTRE')
            return
        if self.state == 'TURN':
            if d < self.centre_dist:                                   # already on the node
                self.stop('CENTRE')
                return
            if abs(err) < self.turn_tol:
                self.state = 'DRIVE'
            else:
                return self.drive(0.0, clamp(1.5 * err, self.wmax) if abs(err) > 0.15 else math.copysign(0.15, err))
        if abs(err) > 0.7:
            return self.stop('TURN')
        v = min(self.vmax, 0.6 * (d - self.centre_dist) + 0.08) * max(0.0, math.cos(err))
        self.drive(v, clamp(2.0 * err, self.wmax))

    def reached(self, tid):
        self.get_logger().info(f'node {tid} reached')
        self.i += 1
        if self.i >= len(self.route):
            self.route = []
            self.stop('ARRIVED')
        else:
            self.stop('TURN')

    def draw_route(self):
        ma = MarkerArray()
        line = Marker(type=Marker.LINE_STRIP, ns='route', id=0)
        goal = Marker(type=Marker.SPHERE, ns='goal', id=0)
        for m in (line, goal):
            m.header.frame_id = self.map
        if self.route:
            pose = self.pose()
            pts = ([pose[:2]] if pose else []) + [self.g.nodes[n][:2] for n in self.route[self.i:]]
            line.points = [Point(x=x, y=y, z=0.04) for x, y in pts]
            line.scale.x = 0.06
            line.color.g, line.color.a = 1.0, 1.0
            gx, gy, _ = self.g.nodes[self.goal]
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = gx, gy, 0.1
            goal.scale.x = goal.scale.y = goal.scale.z = 0.25
            goal.color.r, goal.color.g, goal.color.a = 1.0, 0.6, 1.0
        else:
            line.action = goal.action = Marker.DELETE
        ma.markers = [line, goal]
        self.viz.publish(ma)

    def report(self):
        self.draw_route()
        if self.state != 'IDLE':
            pose = self.pose()
            self.status.publish(String(data=json.dumps(
                {'state': 'BLOCKED' if (self.blocked and self.state in ('DRIVE', 'CENTRE', 'TURN')) else self.state, 'goal': self.goal, 'route': self.route, 'next': self.route[self.i] if self.route else None,
                 'pose': None if pose is None else [round(v, 3) for v in pose]})))


def main():
    rclpy.init()
    rclpy.spin(TagNavigator())
