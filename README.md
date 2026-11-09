# ros2_agv - AprilTag AGV navigation (ROS 2 Humble + Ignition Gazebo Fortress)



https://github.com/user-attachments/assets/3f8efdcd-5bfa-43c6-ae8e-518ae40a1e5e



A differential-drive AGV that navigates **only with AprilTags**. Tags (family 36h11, ids 0-19) lie on the floor of a simulated
room. Each tag is a **node** of the AGV's internal map, each lane between two tags is an **edge**. Two cameras (front and
underneath) detect the tags, TF turns the detections into the robot's pose, Dijkstra finds the shortest path over the
nodes, and a lidar gate stops the robot for obstacles. Give a goal by tapping a tag in RViz.

Author: rohith (rohithmeti03@gmail.com)

## 1. Requirements
Ubuntu 22.04 with **ROS 2 Humble** (`ros-humble-desktop`), then:
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2 \
  ros-humble-tf2-ros-py ros-humble-teleop-twist-keyboard python3-opencv python3-numpy python3-yaml
```
`ros-humble-ros-gz` brings Ignition Gazebo Fortress. `python3-opencv` (4.5.x) provides the AprilTag detector
(`cv2.aruco`, dictionary `DICT_APRILTAG_36h11`), so no separate AprilTag library is needed.

## 2. Get the code and build
```bash
git clone https://github.com/rohithmeti/ros2_agv_apriltag.git ~/ros2_agv
cd ~/ros2_agv
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y     # optional: installs anything from the list above that is missing
colcon build --symlink-install
source install/setup.bash
```
In every new terminal: `source ~/ros2_agv/install/setup.bash`.

## 3. Launch
```bash
# terminal 1 - simulator (Gazebo: room, walls, obstacles, tags, robot)
ros2 launch my_bot_description sim_ign.launch.py

# terminal 2 - navigation stack + RViz
ros2 launch agv_tag_nav agv_tag_nav.launch.py          # rviz:=false to run without RViz
```
The robot spawns at (0, 0), where no tag is visible. It must see one tag once before it can plan, so move it onto tag 8:
```bash
ign service -s /world/indoor_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
  --timeout 2000 --req 'name: "my_robot2" position {x: -0.22 y: -0.8 z: 0.05} orientation {w: 1.0}'
```
(or drive it with `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_nav` until a tag is in view).
The stack prints `localised from tag N`.

## 4. Give a goal
* **RViz:** choose **2D Goal Pose** (or **Publish Point**) in the toolbar and click on a tag square. The nearest tag within 0.6 m is the goal.
* **Terminal:** `ros2 topic pub --once /agv/goal_node std_msgs/msg/Int32 "{data: 14}"`  (`-1` cancels), `ros2 topic echo /agv/status`.

More detail (tag layout, obstacle test, troubleshooting): **[START.md](START.md)**.

## 5. What each program does
**Navigation stack - `src/agv_tag_nav`** (Python, ROS 2 nodes started by `launch/agv_tag_nav.launch.py`)

| Program | What it does | In -> Out |
|---|---|---|
| `tag_map_node` | publishes the map of tags | `config/tag_map.yaml` -> static TF `map->tag_N`, RViz markers `/agv/graph` |
| `tag_detector` | scans both cameras for AprilTags and computes each tag's pose with solvePnP | camera images + camera_info -> TF `det_front_N` / `det_down_N`, `/agv/tag_detections`, annotated images `/agv/front/tags_debug`, `/agv/down/tags_debug` |
| `tag_localizer` | robot pose from a seen tag: `map->base = (map->tag) * inverse(base->tag seen)`; odometry carries the pose between tags | detections + TF -> TF `map->odom` |
| `tag_navigator` | Dijkstra over the tag graph, then per node: stop, turn to the lane heading, drive the lane (pure-pursuit carrot on the lane line), centre on the tag with the down camera | `/agv/goal_node`, RViz `/goal_pose` or `/clicked_point` -> `/cmd_vel_nav`, `/agv/status`, `/agv/route_viz` |
| `safety_stop` | lidar gate between navigator and wheels using the real footprint (0.40 x 0.30 m): slows, stops, or blocks a turn that would hit something | `/cmd_vel_nav` + `/scan` -> `/cmd_vel`, `/agv/obstacle`, `/agv/safety_viz` |
| `graph.py`, `geom.py` | library: graph + shortest path, pose/TF maths | - |
| `scripts/make_tag_map.py` | builds `tag_map.yaml` from the sim's tag layout | `config/apriltag_layout.yaml` -> `config/tag_map.yaml` |
| `rviz/agv_tag_nav.rviz` | RViz view: tag graph + ids, route + goal, robot, TF, footprint + stop zone, lidar, both camera views | - |

**Simulator - `src/my_bot_description`**

| File | What it is |
|---|---|
| `launch/sim_ign.launch.py` | starts Gazebo with the world, spawns the robot, ROS<->Gazebo bridges (clock, odom, cmd_vel, scan, cameras, tf) |
| `urdf/robot.urdf.xacro`, `urdf/ign_plugins.gazebo.xacro` | the AGV: body 0.40 x 0.30 m, two drive wheels + casters, lidar, front camera (tilted 35 deg down), down camera; diff-drive, joint-state, lidar and camera plugins |
| `worlds/indoor_world.sdf` | 10 x 10 m room, two chicane walls, table, cube, cylinder, the 20 tags (`lane_world.sdf` is a second world) |
| `models/apriltag_36h11_000NN/` | one Gazebo model per tag (texture + sdf) |
| `config/apriltag_layout.yaml`, `scripts/place_apriltags.py` | tag positions -> tag models + world tag block |
| `launch/rsp.launch.py`, `launch/slam.launch.py`, `config/nav2_params.yaml`, `config/my_nav_params.yaml`, `maps/` | older helpers (robot_state_publisher only, SLAM, Nav2); not used by this stack |
| `ros2_amr_ws/ros2_hmi2.py` | a separate pygame HMI script; not part of this stack and not tested here |

## 6. Software used
ROS 2 Humble (rclpy, TF2, launch), Ignition Gazebo Fortress via `ros_gz`, RViz2, OpenCV (AprilTag 36h11 detection + IPPE pose),
NumPy, PyYAML, colcon. The path planner is a small Dijkstra written in `graph.py`.

## 7. Layout
```
ros2_agv/
├── README.md  START.md  .gitignore
└── src/
    ├── my_bot_description/     simulator
    └── agv_tag_nav/            navigation stack (its README.md has topics + measured results)
```
