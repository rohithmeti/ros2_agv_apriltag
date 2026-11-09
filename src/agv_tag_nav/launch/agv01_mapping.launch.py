"""ONE command to map: simulator (agv_01 in the warehouse) + SLAM Toolbox + lidar safety stop + RViz.

ros2 launch agv_tag_nav agv01_mapping.launch.py
   layout:=ortho|grid (which world)   rviz:=false   x:= y:= yaw:= (empty = start on tag 1)   slam_delay:=20.0
Drive with Gazebo's Teleop panel (left side; tick the keyboard box, click the 3D view, use the keys).
Save the map:  src/agv_tag_nav/scripts/save_map.sh warehouse
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    L = LaunchConfiguration
    sim_launch = os.path.join(get_package_share_directory('my_bot_description'), 'launch', 'sim_agv01.launch.py')
    slam_launch = os.path.join(get_package_share_directory('agv_tag_nav'), 'launch', 'slam_mapping.launch.py')
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in
            {'layout': 'ortho', 'rviz': 'true', 'x': '', 'y': '', 'yaw': '', 'slam_delay': '20.0'}.items()]
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch),
                                   launch_arguments={'world': [TextSubstitution(text='warehouse_'), L('layout'), TextSubstitution(text='.sdf')],
                                                     'x': L('x'), 'y': L('y'), 'yaw': L('yaw')}.items())
    # SLAM starts after Gazebo is up so the first scans and the odom TF are already flowing
    slam = TimerAction(period=L('slam_delay'), actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch), launch_arguments={'rviz': L('rviz')}.items())])
    return LaunchDescription([*args, sim, slam])
