"""agv_01 in the warehouse (with the AprilTag floor markers).

ros2 launch my_bot_description sim_agv01.launch.py
   world:=warehouse_ortho.sdf        (default; warehouse_grid.sdf = the older diagonal-lane backup)
   x:= y:= yaw:=                     leave empty = start ON TAG 1 (read from worlds/<world>.spawn.yaml)
Same topics/frames as sim_ign.launch.py: /scan /camera/image_raw /camera_down/image_raw /cmd_vel /odom, base_footprint.
"""
import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAME = 'agv_01'


def spawn_pose(pkg_share, world, ax, ay, ayaw):
    """explicit x/y/yaw win; otherwise the start pose stored next to the world (on tag 1); otherwise the origin"""
    f = os.path.join(pkg_share, 'worlds', os.path.splitext(world)[0] + '.spawn.yaml')
    d = yaml.safe_load(open(f)) if os.path.exists(f) else {}
    pick = lambda arg, key: str(arg) if str(arg) != '' else str(d.get(key, 0.0))
    return pick(ax, 'x'), pick(ay, 'y'), pick(ayaw, 'yaw')


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('my_bot_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    L = lambda k: LaunchConfiguration(k).perform(context)
    x, y, yaw = spawn_pose(pkg_share, L('world'), L('x'), L('y'), L('yaw'))
    robot_desc = xacro.process_file(os.path.join(pkg_share, 'urdf', 'agv_01.urdf.xacro')).toxml()

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', L('world')]}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
               output='screen', parameters=[{'robot_description': robot_desc, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', NAME, '-x', x, '-y', y, '-z', L('z'), '-Y', yaw])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Gazebo GUI keyboard teleop (world Teleop panel topic) -> ROS /cmd_vel_nav (-> safety_stop -> /cmd_vel)
            '/cmd_vel_teleop@geometry_msgs/msg/Twist[ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/model/{NAME}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/model/{NAME}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera_down/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_down/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        remappings=[(f'/model/{NAME}/odometry', '/odom'), (f'/model/{NAME}/tf', '/tf'), ('/cmd_vel_teleop', '/cmd_vel_nav')])
    return [gz, rsp, spawn, bridge]


def generate_launch_description():
    pkg_share = get_package_share_directory('my_bot_description')
    res = os.path.dirname(pkg_share) + ':' + os.path.join(pkg_share, 'worlds') + ':' + os.path.join(pkg_share, 'models')
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        res = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + res
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in
            {'world': 'warehouse_ortho.sdf', 'x': '', 'y': '', 'yaw': '', 'z': '0.02'}.items()]
    return LaunchDescription([SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', res), *args, OpaqueFunction(function=launch_setup)])
