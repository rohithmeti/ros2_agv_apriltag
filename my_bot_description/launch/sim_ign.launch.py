import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Package Directories
    pkg_name = 'my_bot_description'
    pkg_share = get_package_share_directory(pkg_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # 2. Set Ignition Resource Path
    ign_resource_path = os.path.dirname(pkg_share)
    ign_resource_path += ":" + os.path.join(pkg_share, 'worlds')
    ign_resource_path += ":" + os.path.join(pkg_share, 'models')  # AprilTag floor markers

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        ign_resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + ign_resource_path

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', 
        value=ign_resource_path
    )

    # 3. World Argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='indoor_world.sdf',
        description='World to load'
    )

    # 4. Parse URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc_xml = doc.toxml()

    # 5. Launch Ignition Gazebo
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items(),
    )

    # 6. Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_xml, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot2', '-z', '0.1'],
        output='screen'
    )

    # TF Glue: Connects Lidar frames
    tf_glue = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'lidar_1', '--child-frame-id', 'my_robot2/base_footprint/lidar']
    )

    # Bridge: Connects ROS to Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/my_robot2/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/my_robot2/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',

            # --- ROBOT CAMERAS ---
            # front (tilted 35 deg down)
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # down (under the robot, looks at the floor tag below)
            '/camera_down/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_down/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',

            # --- NEW OVERHEAD CAMERA BRIDGE (This was missing) ---
            '/overhead_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        remappings=[
            ('/model/my_robot2/odometry', '/odom'),
            ('/model/my_robot2/tf', '/tf'),
        ],
        output='screen'
    )

    return LaunchDescription([
        set_ign_resource_path,
        world_arg,
        ign_gazebo,
        robot_state_publisher,
        spawn_entity,
        tf_glue,
        bridge
    ])
