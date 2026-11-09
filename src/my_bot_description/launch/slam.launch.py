import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Start SLAM Toolbox (standard async mode)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': '' # Use default for now
        }.items()
    )

    # 2. Start RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # We will create this config file in a moment
        arguments=['-d', os.path.join(get_package_share_directory('my_bot_description'), 'rviz', 'slam.rviz')]
    )

    return LaunchDescription([
        slam,
        rviz
    ])
