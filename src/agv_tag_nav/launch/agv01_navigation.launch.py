"""ONE command to navigate: simulator (agv_01 in the warehouse with the tag grid) + tag navigation stack + RViz.

ros2 launch agv_tag_nav agv01_navigation.launch.py
   layout:=ortho   orthogonal lanes (default)      layout:=grid   the older map with diagonal lanes (backup)
   rviz:=false   stack_delay:=25.0   x:= y:= yaw:=   (empty = start ON TAG 1)
Goal: in RViz pick "2D Goal Pose" / "Publish Point" and click a tag, or
      ros2 topic pub --once /agv/goal_node std_msgs/msg/Int32 "{data: 100}"      (-1 cancels)
Do not run together with agv01_mapping.launch.py (both publish map->odom).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    L = LaunchConfiguration
    share = get_package_share_directory('agv_tag_nav')
    sim_launch = os.path.join(get_package_share_directory('my_bot_description'), 'launch', 'sim_agv01.launch.py')
    stack_launch = os.path.join(share, 'launch', 'agv_tag_nav.launch.py')
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in
            {'layout': 'ortho', 'rviz': 'true', 'x': '', 'y': '', 'yaw': '', 'stack_delay': '25.0'}.items()]
    world = [TextSubstitution(text='warehouse_'), L('layout'), TextSubstitution(text='.sdf')]
    tag_map = [TextSubstitution(text=os.path.join(share, 'config', 'tag_map_warehouse_')), L('layout'), TextSubstitution(text='.yaml')]
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch),
                                   launch_arguments={'world': world, 'x': L('x'), 'y': L('y'), 'yaw': L('yaw')}.items())
    stack = TimerAction(period=L('stack_delay'), actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stack_launch),
        launch_arguments={'rviz': L('rviz'), 'tag_map': tag_map,
                          'footprint_length': '0.60', 'footprint_width': '0.515'}.items())])
    return LaunchDescription([*args, sim, stack])
