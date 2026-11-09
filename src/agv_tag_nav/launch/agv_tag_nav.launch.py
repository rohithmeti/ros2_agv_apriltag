import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('agv_tag_nav')
    rviz_cfg = os.path.join(share, 'rviz', 'agv_tag_nav.rviz')
    L = LaunchConfiguration
    sim = {'use_sim_time': L('use_sim_time')}
    map_p = {'tag_map_file': L('tag_map')}
    safety = {'footprint_length': L('footprint_length'), 'footprint_width': L('footprint_width')}
    n = lambda exe, *extra: Node(package='agv_tag_nav', executable=exe, output='screen', parameters=[sim, *extra])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true', description='open RViz2 with the tag-nav view'),
        DeclareLaunchArgument('tag_map', default_value=os.path.join(share, 'config', 'tag_map.yaml'),
                              description='tag graph yaml (config/tag_map_warehouse.yaml for the warehouse)'),
        DeclareLaunchArgument('footprint_length', default_value='0.40', description='robot length for the safety stop [m]'),
        DeclareLaunchArgument('footprint_width', default_value='0.30', description='robot width incl. wheels [m]'),
        n('tag_map_node', map_p), n('tag_detector'), n('tag_localizer'), n('tag_navigator', map_p), n('safety_stop', safety),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg],
             parameters=[sim], output='log', condition=IfCondition(L('rviz'))),
    ])
