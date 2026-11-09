# ros2_agv - AprilTag AGV navigation (ROS 2 Humble + Gazebo Fortress)



https://github.com/user-attachments/assets/473d22f0-fb7c-41f8-824c-9dae403e28df



A differential-drive AGV that navigates **only with AprilTags on the floor**. Every tag is a **node** of the AGV's internal map, every
lane between two tags is an **edge**. Two cameras (one in front, one underneath) detect the tags, TF turns each detection into the robot's
position, Dijkstra finds the shortest route over the tags, a lidar gate stops the robot in front of obstacles. You give a goal by clicking a
tag in RViz. No lidar map, no AMCL, no Nav2 is needed for the navigation itself.

| | What | Where | Start with |
|---|---|---|---|
| **Version 2** (this branch, recommended) | AGV `agv_01` (0.60 x 0.515 m) in a 30 x 50 m Gazebo warehouse, **282 tags / 444 orthogonal lanes** (no diagonals) | `src/` | `ros2 launch agv_tag_nav agv01_navigation.launch.py` |
| **Version 1** (kept for reference) | first small AGV in a 10 x 10 m room, 20 tags | `old_agv/` | `ros2 launch agv_v1_description sim_ign.launch.py` |

Git branches: `main` = Version 2 (+ Version 1 inside `old_agv/`), `old` = the repository exactly as it was before Version 2.

---

## Contents
1. [Prerequisites](#1-prerequisites) · 2. [Install and build](#2-install-and-build) · 3. [Run Version 2](#3-run-version-2) · 4. [Run Version 1](#4-run-version-1)
5. [Map a new environment (SLAM)](#5-map-a-new-environment-slam) · 6. [Make a tag map for your own environment](#6-make-a-tag-map-for-your-own-environment)
7. [How it works](#7-how-it-works) · 8. [Measured results](#8-measured-results) · 9. [Repository layout](#9-repository-layout) · 10. [Troubleshooting](#10-troubleshooting) · 11. [Known limits](#11-known-limits)

---

## 1. Prerequisites
* **Ubuntu 22.04** with **ROS 2 Humble** (`ros-humble-desktop`), see the [official install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
* A computer that can run Gazebo with cameras and a lidar: a GPU is recommended, 8 GB RAM or more.
* **Internet on the first run of Version 2**: the warehouse world uses models from Gazebo Fuel (shelves, pallets, building, about 15 MB) that Gazebo downloads once and caches in `~/.ignition/fuel`.
* Python packages and ROS packages from the next step. No separate AprilTag library is needed: the detector uses OpenCV's built-in AprilTag 36h11 dictionary (`cv2.aruco`).

## 2. Install and build
```bash
# 1. system packages
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep \
  ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2 \
  ros-humble-tf2-ros-py ros-humble-teleop-twist-keyboard \
  ros-humble-slam-toolbox ros-humble-nav2-map-server \
  python3-opencv python3-numpy python3-scipy python3-pil python3-yaml
# ros-humble-ros-gz brings Ignition Gazebo Fortress. slam-toolbox / nav2-map-server are only needed for section 5.

# 2. get the code (any folder works; the commands below assume ~/ros2_agv)
git clone https://github.com/rohithmeti/ros2_agv_apriltag.git ~/ros2_agv
cd ~/ros2_agv

# 3. build (builds all three packages: agv_tag_nav, my_bot_description (v2), agv_v1_description (v1))
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
In **every new terminal** run `source ~/ros2_agv/install/setup.bash` first. With `--symlink-install`, edits to existing python / launch / yaml / rviz files apply on the next launch; **new files need a rebuild**.
(Optional: `sudo rosdep init; rosdep update; rosdep install --from-paths src old_agv --ignore-src -y` installs anything from the list above that is missing.)

## 3. Run Version 2
### One command
```bash
ros2 launch agv_tag_nav agv01_navigation.launch.py
```
This opens Gazebo (the warehouse, the AGV standing on **tag 1 in the middle of the floor**), then after about 25 s the navigation stack and RViz. Wait for
`localised from tag 1 (down)` in the terminal. The first start can take 1-2 minutes (Gazebo loads the world and 282 tag textures; Fuel models download once).

### Give a goal
* **RViz (easiest):** in the top toolbar choose **2D Goal Pose** (or **Publish Point**) and click on a tag. The nearest tag within 0.6 m becomes the goal, a green line shows the route, an orange ball the goal. Clicking again while it drives re-plans.
* **Terminal:** `ros2 topic pub --once /agv/goal_node std_msgs/msg/Int32 "{data: 253}"` (`-1` cancels). Status: `ros2 topic echo /agv/status`.
* **Drive by hand:** in Gazebo's left panel (Teleop) tick the keyboard box, click the 3D view and use the keys. The commands pass through the lidar safety stop (speed capped at 0.5 m/s).

Good demo goals, where the shortest route is **not** the straight line: tags around the racks, e.g. `253` (north-east corner, 29 m away) or a tag on the far side of a shelf row.
Status values: `TURN` (rotating to the lane), `DRIVE`, `CENTRE` (down camera centring on the tag), `BLOCKED` (lidar sees an obstacle, waits), `ARRIVED`.

### Options
```bash
ros2 launch agv_tag_nav agv01_navigation.launch.py layout:=grid     # the older tag map that also has diagonal lanes (backup)
ros2 launch agv_tag_nav agv01_navigation.launch.py rviz:=false      # without RViz
ros2 launch agv_tag_nav agv01_navigation.launch.py x:=0.0 y:=-2.0 yaw:=1.5708   # other start pose; it MUST be on a tag
```
The start pose is stored next to each world (`worlds/<world>.spawn.yaml`, default: on tag 1). Only one of the navigation / mapping launches may run at a time (both publish `map -> odom`).

## 4. Run Version 1
```bash
# terminal 1: the old simulator (10 x 10 m room, 20 tags)
ros2 launch agv_v1_description sim_ign.launch.py
# terminal 2: the same navigation stack with Version 1 settings (default map = 20 tags, footprint 0.40 x 0.30 m)
ros2 launch agv_tag_nav agv_tag_nav.launch.py
# the robot spawns at (0,0) where no tag is visible: put it on tag 8 once
ign service -s /world/indoor_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 \
  --req 'name: "my_robot2" position {x: -0.22 y: -0.8 z: 0.05} orientation {w: 1.0}'
ros2 topic pub --once /agv/goal_node std_msgs/msg/Int32 "{data: 14}"
```
More about Version 1 in [`old_agv/README.md`](old_agv/README.md).

## 5. Map a new environment (SLAM)
Used to get an occupancy map of a world once, so the tag grid can be designed on it.
```bash
ros2 launch agv_tag_nav agv01_mapping.launch.py     # Gazebo + SLAM Toolbox + safety stop + RViz (do not run together with the navigation launch)
```
Drive with Gazebo's Teleop panel (see above) slowly and finish loops. Save the map:
```bash
src/agv_tag_nav/scripts/save_map.sh my_world        # -> src/agv_tag_nav/maps/my_world.pgm + .yaml + pose graph
```

## 6. Make a tag map for your own environment
`src/agv_tag_nav/scripts/make_tag_ortho.py` turns an occupancy map (`.pgm` + `.yaml`) and a Gazebo world into a tag grid:
```bash
python3 src/agv_tag_nav/scripts/make_tag_ortho.py --map src/agv_tag_nav/maps/warehouse.yaml \
   --world-in src/my_bot_description/worlds/tugbot_warehouse.sdf --name my_layout
```
* finds the **centre line of every aisle** in the map and puts a line of tags along it, plus a regular lattice for open floor;
* tags sit on the crossings; **lanes only join adjacent crossings**, so they are always parallel or perpendicular, never diagonal, never crossing mid-lane;
* a lane exists only where the robot (0.515 m wide) fits with margin, so tags on two sides of a rack are not connected;
* writes `config/tag_map_<name>.yaml` (tag id -> world x, y), `worlds/<name>.sdf` (the world with every tag on the floor), the start pose and the tag textures. Rebuild afterwards, then `... agv01_navigation.launch.py layout:=<name>`.

`scripts/drift_check.py` compares the stack's position estimate with Gazebo's ground truth while the robot drives (`drift_check.py 253 100 1`, or no arguments to just watch while you click goals in RViz).

---

## 7. How it works

### 7.1 Programs (package `agv_tag_nav` is the navigation stack)
| Program | Responsible for | In -> Out |
|---|---|---|
| `tag_map_node` | publishes where every tag is | tag map yaml -> static TF `map -> tag_<id>`, RViz markers |
| `tag_detector` | looks for tags in both cameras and measures each tag's pose | camera images + camera_info -> TF `camera -> det_<cam>_<id>` |
| `tag_localizer` | position of the robot from a seen tag | detections + TF -> TF `map -> odom` |
| `tag_navigator` | route planning and driving | goal tag -> Dijkstra over the graph -> `/cmd_vel_nav` |
| `safety_stop` | lidar protection with the real footprint | `/cmd_vel_nav` + `/scan` -> `/cmd_vel` |
| `graph.py`, `geom.py` | library: graph + shortest path, pose maths | |

The simulation packages: `my_bot_description` (Version 2: robot model `agv_01`, warehouse worlds, tag textures) and `agv_v1_description` (Version 1).

### 7.2 The frames (TF tree)
```
map ──(tag_localizer)──> odom ──(wheel odometry)──> base_footprint ──> base_link ──> camera_front_link ──> camera_front_optical_frame ──> det_front_<id>
 │                                                                              └──> camera_down_link  ──> camera_down_optical_frame  ──> det_down_<id>
 └──(static, from the tag map)──> tag_<id>
```
`map` is the world frame (the same as Gazebo's world). `tag_<id>` are the **known** positions of the tags. `det_<cam>_<id>` is the tag **as measured by a camera**.
`odom -> base_footprint` is the wheel odometry (smooth, but it drifts). The localizer's whole job is the one transform `map -> odom` that removes that drift.

### 7.3 How the camera TF is calculated
1. The detector finds the four corners of a tag in the image (OpenCV AprilTag 36h11 dictionary, sub-pixel corners) and reads its ID.
2. With the camera intrinsics from `camera_info` and the **known tag size (0.20 m black square)** it solves the perspective-n-point problem (`solvePnP`, IPPE square solver). The result is the tag's pose in the camera's optical frame, `T_cam_tag` (z forward, x right, y down).
3. It publishes this as the TF `camera_optical_frame -> det_<cam>_<id>`. The cameras are bolted to the robot, so TF chains it for free: `base_footprint -> ... -> det_<cam>_<id>` is the pose of the tag as seen **from the robot's centre**, `T_base_tag`.

### 7.4 How an AprilTag gives the robot its position
* **A 36h11 tag only encodes an ID (0 ... 586).** It does not contain coordinates. The coordinates live in the **tag map table** (`config/tag_map_*.yaml`: `id -> x, y, yaw` in the world frame) that every program loads. The simulated tags are placed in Gazebo from the same table, so the table and the floor always agree. On a real site you survey the tags to those coordinates (or write the surveyed coordinates into the table).
* Seeing tag `k`, the localizer knows two things: where the tag **is** (`T_map_tag`, from the table) and where it is **relative to the robot** (`T_base_tag`, measured). Therefore the robot is at
  `T_map_base = T_map_tag * inverse(T_base_tag)`.
* Only x, y and heading are used (flat floor). Then `T_map_odom = T_map_base * inverse(T_odom_base)` is published as TF `map -> odom`. Between tags the robot position is `map -> odom -> base_footprint`, i.e. the last correction plus wheel odometry.
* The checks that keep it from jumping around: the camera **underneath** sees a tag only from a few cm away, so its fix is exact and is always accepted; the **front** camera fix is used only within 2.5 m, blended in at 50 %, a jump larger than 0.6 m is ignored unless it repeats 8 times, and tags seen at a grazing angle are rejected (the tag's normal must point up). A detection whose TF has not arrived yet is retried for 1 s.

### 7.5 Navigation
* The graph is the tag map: nodes = tags, edges = lanes. The planner is Dijkstra on lane length, so the route follows lanes around racks.
* At every node the AGV follows the AGV rules: **stop, turn on the spot to the lane heading, drive the lane, centre on the next tag.** While driving it steers to a point 0.35 m ahead on the lane line (pure pursuit on the lane). Inside 0.3 m of the node it centres on the tag using the down camera (stop tolerance 2 cm; if the tag is not visible it falls back to the map position, 3 cm).
* A click in RViz picks the nearest tag. A new goal replaces the old one.

### 7.6 Safety stop
Between the navigator and the wheels, `safety_stop` checks the lidar against the **real footprint** (0.60 x 0.515 m including the wheels, +5 cm each side): in a corridor ahead of the nose it slows down and stops (stop distance = 0.08 m + 0.25 s * speed + speed^2 / 1.2), it blocks a turn that would sweep the body into an obstacle, caps the speed (0.5 m/s, 1.0 rad/s) and stops if the lidar or the commands go silent for 0.5 s. Everything that moves the robot (navigator, keyboard) goes through it.

## 8. Measured results
All numbers are from the Gazebo simulation, position error against Gazebo's ground truth. Small samples, not a long-run test.
* Start on tag 1: localised at once, error 0.2 cm.
* Out and back, 29 m each way over 16 tags (Version 2, tag 1 -> tag 253 -> tag 1): position estimate vs ground truth **mean 0.5 cm, max 4.7 cm** (outbound) and 4.6 cm (return), heading max 2.4 deg, stopped **1.8 cm** and **1.9 cm** from the goal tags; the return trip ended on tag 1 again with no accumulated drift.
* Another ~2 minutes of goals clicked by hand in RViz (Version 2): mean 0.6 cm, max 5.4 cm; no delocalisation.
* Version 1 (earlier tests): stops within 1.8 - 1.9 cm of the goal tag over routes of 4 m and 12 m.
* Once, during development, the position estimate drifted by about 1 m after the robot pushed against a shelf corner. The localizer now always accepts the exact fix from the camera underneath, so the next tag the robot drives over repairs such an error; this repair has not been verified with a real collision.

## 9. Repository layout
```
ros2_agv/
├── README.md  START.md  LICENSE  .gitignore
├── src/                              VERSION 2
│   ├── agv_tag_nav/                  the navigation stack (nodes, launch files, tag maps, RViz views, tools) - also used by Version 1
│   │   ├── agv_tag_nav/              python nodes (tag_detector, tag_localizer, tag_navigator, safety_stop, ...)
│   │   ├── launch/                   agv01_navigation (everything), agv01_mapping (SLAM), agv_tag_nav (stack only), slam_mapping
│   │   ├── config/                   tag maps: warehouse_ortho (default), warehouse_grid (diagonal backup), tag_map (Version 1), slam params
│   │   ├── maps/                     warehouse occupancy map + previews of the tag grids
│   │   ├── rviz/  scripts/           RViz views; make_tag_ortho.py, drift_check.py, save_map.sh, ...
│   └── my_bot_description/           Version 2 simulation: agv_01 model, warehouse worlds, tag textures
└── old_agv/                          VERSION 1 (old AGV, old 10 x 10 m world, 20 tags) -> old_agv/README.md
    └── agv_v1_description/
```

## 10. Troubleshooting
| Symptom | Fix |
|---|---|
| Gazebo opens but nothing moves / no sensor data | wait: the first start loads the world and downloads models; check `ros2 topic hz /scan` |
| `not localised yet (no tag seen)` | the robot must see one tag first. Version 2 starts on tag 1; for another pose use `x:= y:= yaw:=` on a tag |
| robot stops, status `BLOCKED` | the lidar sees something in the corridor ahead (RViz shows the red stop zone) |
| two simulators / strange behaviour after a crash | stop leftovers first: `src/agv_tag_nav/scripts/stop_all.sh` (it stops all Gazebo/ROS processes of your user), then start again |
| a goal does nothing | click closer than 0.6 m to a tag; only one goal is active, a new click replaces it |
| `Unable to find or download file` for a world | `colcon build --symlink-install` again (new files need a rebuild) and `source install/setup.bash` |

## 11. Known limits
* Needs one tag in view to start (no global search); between tags it trusts the wheel odometry.
* The camera underneath sees only about 6 cm around a tag centre, the front camera about 1 m, so tags are 1 - 5 m apart.
* The single 2D lidar plane (about 0.29 m high) cannot see obstacles lower than that, such as pallets.
* Lanes are one lane per pair of tags and two-way; no multi-robot traffic management; no blocked-lane re-planning (it waits).
* Simulation only; nothing here has been tried on a real robot.
* The lane-maker GUI (a separate project) is not part of this repository.

## License
Apache-2.0, see [LICENSE](LICENSE).
