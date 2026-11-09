"""Map an environment once with agv_01 (slam_toolbox online async) + the lidar safety stop + RViz.

ros2 launch agv_tag_nav slam_mapping.launch.py            # sim must be running; do NOT run agv_tag_nav.launch.py at the same time
   rviz:=false  slam_params:=<yaml>
Drive:  Gazebo's Teleop panel (tick the keyboard box, keep the Gazebo window focused) - it is bridged to /cmd_vel_nav, or
        ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_nav
        (both go through the lidar safety stop, which is always on here)
Save:   src/agv_tag_nav/scripts/save_map.sh warehouse
slam_toolbox publishes map->odom, exactly what tag_localizer does later, so the two never run together.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('agv_tag_nav')
    L = LaunchConfiguration
    sim = {'use_sim_time': L('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam_params', default_value=os.path.join(share, 'config', 'slam_agv01.yaml')),
        DeclareLaunchArgument('footprint_length', default_value='0.60'),
        DeclareLaunchArgument('footprint_width', default_value='0.515'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             parameters=[L('slam_params'), sim]),
        Node(package='agv_tag_nav', executable='safety_stop', output='screen',
             parameters=[sim, {'footprint_length': L('footprint_length'), 'footprint_width': L('footprint_width')}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log', condition=IfCondition(L('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'slam_mapping.rviz')], parameters=[sim]),
    ])
