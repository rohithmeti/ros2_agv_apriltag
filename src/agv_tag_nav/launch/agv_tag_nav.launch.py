import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    n = lambda exe, **kw: Node(package='agv_tag_nav', executable=exe, output='screen', parameters=[sim], **kw)
    rviz_cfg = os.path.join(get_package_share_directory('agv_tag_nav'), 'rviz', 'agv_tag_nav.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true', description='open RViz2 with the tag-nav view'),
        n('tag_map_node'), n('tag_detector'), n('tag_localizer'), n('tag_navigator'), n('safety_stop'),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg],
             parameters=[sim], output='log', condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
