"""Publishes the tag map: static TF map->tag_<id> for every node, and RViz markers of the graph.
In : config/tag_map.yaml      Out: /tf_static (map->tag_N), /agv/graph (MarkerArray, latched)"""
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from .graph import TagGraph
from .geom import mat_from_xyyaw, mat_to_tf


class TagMapNode(Node):
    def __init__(self):
        super().__init__('tag_map')
        default = os.path.join(get_package_share_directory('agv_tag_nav'), 'config', 'tag_map.yaml')
        self.g = TagGraph(self.declare_parameter('tag_map_file', default).value)
        now = self.get_clock().now().to_msg()
        StaticTransformBroadcaster(self).sendTransform(
            [mat_to_tf(mat_from_xyyaw(*self.g.nodes[i]), self.g.frame, f'tag_{i}', now) for i in self.g.nodes])
        pub = self.create_publisher(MarkerArray, '/agv/graph',
                                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        ma = MarkerArray()
        lines = Marker(type=Marker.LINE_LIST, ns='lanes', id=0)
        lines.header.frame_id = self.g.frame
        lines.scale.x = 0.03
        lines.color.r, lines.color.g, lines.color.b, lines.color.a = 0.1, 0.6, 1.0, 1.0
        for a, nbrs in self.g.adj.items():
            for b, _ in nbrs:
                if a < b or all(x != a for x, _ in self.g.adj[b]):
                    for n in (a, b):
                        lines.points.append(Point(x=self.g.nodes[n][0], y=self.g.nodes[n][1], z=0.02))
        ma.markers.append(lines)
        for i, (x, y, yaw) in self.g.nodes.items():
            sq = Marker(type=Marker.CUBE, ns='tags', id=i)
            sq.header.frame_id = self.g.frame
            sq.pose.position.x, sq.pose.position.y, sq.pose.position.z = x, y, 0.005
            sq.scale.x = sq.scale.y = 0.25
            sq.scale.z = 0.01
            sq.color.r = sq.color.g = sq.color.b = 0.9
            sq.color.a = 1.0
            ma.markers.append(sq)
            t = Marker(type=Marker.TEXT_VIEW_FACING, ns='ids', id=i)
            t.header.frame_id = self.g.frame
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 0.03
            t.scale.z = 0.2
            t.color.a = 1.0                     # black text on the white tag square
            t.text = str(i)
            ma.markers.append(t)
        pub.publish(ma)
        self.get_logger().info(f'tag map: {len(self.g.nodes)} nodes published as map->tag_N')


def main():
    rclpy.init()
    rclpy.spin(TagMapNode())
