import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Check if we want to use simulation time (essential for Gazebo later)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = get_package_share_directory('agv_v1_description')
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create the parameters dictionary
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    # Define the Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Define the RViz node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
        node_robot_state_publisher,
        node_rviz
    ])
