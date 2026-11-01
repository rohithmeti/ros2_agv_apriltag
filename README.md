# ros2_agv - AprilTag AGV simulation (ROS 2 Humble + Ignition Gazebo Fortress)

A differential-drive AGV in an indoor room with **20 AprilTags (36h11, IDs 0-19) on the floor**,
laid out in lanes for camera-based topological navigation. The robot has a lidar, a
front camera tilted down at the floor, and a downward camera under the chassis.

Package: `my_bot_description`

## 1. Requirements

Ubuntu 22.04 + ROS 2 Humble, then:

```bash
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard ros-humble-rqt-image-view
```

(`ros-humble-ros-gz` brings Ignition Gazebo Fortress.)

## 2. Get it and build

```bash
mkdir -p ~/ros2_agv_ws/src
cd ~/ros2_agv_ws/src
git clone https://github.com/rohithmeti/ros2_agv.git
cd ~/ros2_agv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

With `--symlink-install`, edits to existing launch / urdf / world / config files
take effect without rebuilding. **New** files need a rebuild.

## 3. Launch the simulation

```bash
source ~/ros2_agv_ws/install/setup.bash
ros2 launch my_bot_description sim_ign.launch.py
```

The robot spawns at (0, 0) in the indoor room with the AprilTags on the floor.

Drive it (second terminal):

```bash
source ~/ros2_agv_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

View the cameras:

```bash
ros2 run rqt_image_view rqt_image_view     # pick /camera/image_raw or /camera_down/image_raw
```

## 4. Navigation (Nav2) - optional

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True \
  map:=$HOME/ros2_agv_ws/src/ros2_agv/my_bot_description/maps/indoor_map.yaml
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Before using Nav2:
* In `my_bot_description/maps/indoor_map.yaml`, change the `image:` line to
  `image: indoor_map.pgm` (it still holds an absolute path from the original machine).
* The robot needs a `robot_radius` of about **0.30 m** (Nav2 default is 0.22 m,
  `config/nav2_params.yaml` still says 0.12).

## 5. Topics

| Topic | Type | What |
|---|---|---|
| `/cmd_vel` | Twist | drive command (in) |
| `/odom` | Odometry | wheel odometry, frame `odom` -> `base_footprint` |
| `/scan` | LaserScan | 360 deg lidar, frame `lidar_1`, 10 Hz |
| `/camera/image_raw` | Image | **front** camera, 800x800, 30 Hz, frame `camera_front_optical_frame` |
| `/camera/camera_info` | CameraInfo | front intrinsics |
| `/camera_down/image_raw` | Image | **down** camera, 640x640, 30 Hz, frame `camera_down_optical_frame` |
| `/camera_down/camera_info` | CameraInfo | down intrinsics |
| `/joint_states` | JointState | wheel joints -> wheel TFs |
| `/tf`, `/tf_static`, `/clock` | | |

Image frames use the ROS optical convention (z forward, x right, y down), so an
AprilTag detector's poses come out correct directly.

## 6. Robot (`urdf/robot.urdf.xacro` - all sizes are xacro properties at the top)

| Part | Value |
|---|---|
| Body | 0.40 x 0.30 x 0.12 m, 15 kg, underside 0.12 m above floor |
| Drive wheels | r = 0.06 m, on the centre axle, separation 0.38 m (turns in place over a tag) |
| Casters | 2 (front + rear), r = 0.025 m, on legs |
| Lidar | on top, centred, scan height 0.26 m |
| Front camera | front face, 0.20 m high, tilted 35 deg down, 80 deg FOV - sees floor tags ahead |
| Down camera | under the centre, lens 0.11 m above floor, 115 deg FOV - sees 0.34 x 0.34 m of floor, a whole tag fits |

## 7. AprilTags on the floor

* Family **36h11**, IDs **0-19**, **0.20 m** black square (use this as the detector `size`), 0.25 m with white border.
* Layout in `config/apriltag_layout.yaml`:
  * 3 lane columns: W x=-3.30 (ids 0-5), M x=-0.22 (ids 6-11), E x=3.80 (ids 12-17)
  * 6 rows: y = -4.0, -2.4, -0.8, 0.8, 2.4, 4.0 (id = column*6 + row, row 0 = south)
  * connectors: id 18 (W<->M at -1.76, 0.8), id 19 (M<->E at 1.79, -0.8)
* yaw 0 = tag top edge faces +y (north), right edge faces +x (east).
* Flat visuals only: no collision, invisible to the lidar and the map.

Move / add tags:

```bash
# 1. edit my_bot_description/config/apriltag_layout.yaml
python3 ~/ros2_agv_ws/src/ros2_agv/my_bot_description/scripts/place_apriltags.py
# 2. restart the sim
```

New IDs above 19 also need their image in `models/apriltag_36h11_000XX/materials/textures/`.

## 8. Verified

* All topics above publish with the listed frames.
* Down camera parked over tag 8 detects id 8 centred in the image.
* Front camera 0.6 m before tag 8 detects id 8.
* Driving 0.2 m/s for 5 s: Gazebo ground truth 0.988 m, odometry 0.988 m.

## 9. Files

| File | In -> Out |
|---|---|
| `launch/sim_ign.launch.py` | world name -> Gazebo + robot + ROS bridges |
| `urdf/robot.urdf.xacro` | sizes -> robot model (links, cameras, casters) |
| `urdf/ign_plugins.gazebo.xacro` | -> diff drive, joint states, lidar + 2 camera sensors |
| `worlds/indoor_world.sdf` | room + walls + obstacles + tag includes (tag block is generated) |
| `config/apriltag_layout.yaml` | tag ids + floor positions |
| `scripts/place_apriltags.py` | layout yaml -> tag models + world tag block |
| `models/apriltag_36h11_000XX/` | one Gazebo model per tag (texture + sdf) |




# SOON WILL UPDATE WITH AGV NAVIGATION STACK !
