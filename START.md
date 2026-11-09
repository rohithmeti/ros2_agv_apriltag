# START - AprilTag AGV navigation simulation (ROS 2 Humble + Ignition Gazebo Fortress)

Everything is in ONE workspace, `~/ros2_agv` (apt packages to install first: see README.md section 1):
* `src/my_bot_description`  the simulator: robot (2 cameras + lidar), room, 20 floor tags
* `src/agv_tag_nav`         the navigation stack (tag graph, detector, localizer, navigator, safety stop, RViz)

## 1. Get the code and build (once, and after adding new files)
```bash
git clone https://github.com/rohithmeti/ros2_agv_apriltag.git ~/ros2_agv     # skip if you already have the folder
cd ~/ros2_agv
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
`--symlink-install`: edits to existing python / launch / yaml / rviz files apply on the next launch, no rebuild.
In every new terminal: `source ~/ros2_agv/install/setup.bash`.

## 2. Start the simulation  (terminal 1)
```bash
source ~/ros2_agv/install/setup.bash
ros2 launch my_bot_description sim_ign.launch.py
```
Gazebo opens with the room, walls, obstacles and the tags on the floor. The robot spawns at (0, 0).

## 3. Start the navigation stack + RViz  (terminal 2)
```bash
source ~/ros2_agv/install/setup.bash
ros2 launch agv_tag_nav agv_tag_nav.launch.py            # add  rviz:=false  to run without RViz
```
This starts: tag map, tag detector, localizer, navigator, safety stop, and RViz with the tag-nav view.

## 4. Put the robot on a tag (needed once)
The AGV only knows where it is after a camera sees a tag. The spawn point (0, 0) has no tag in view, so first do ONE of:

**a) Move it in the sim to tag 8** (easiest; tag 8 is at x=-0.22, y=-0.8, robot facing east):
```bash
ign service -s /world/indoor_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
  --timeout 2000 --req 'name: "my_robot2" position {x: -0.22 y: -0.8 z: 0.05} orientation {w: 1.0}'
```
**b) Drive it** with the keyboard until a tag shows in the camera view:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_nav
```
(`cmd_vel_nav` so the lidar safety stop also protects manual driving.)

Check: terminal 2 prints `localised from tag N (...)`. Until then goals are refused with "not localised yet".

## 5. Give a goal
**In RViz (simple tap on a tag):** pick **2D Goal Pose** (or **Publish Point**) in the top toolbar and click on a tag square.
The nearest tag within 0.6 m becomes the goal; a click elsewhere only prints a warning. A new click while driving re-plans.
A green line shows the route, an orange sphere the goal.

**From a terminal:**
```bash
ros2 topic pub --once /agv/goal_node std_msgs/msg/Int32 "{data: 14}"    # go to tag 14
ros2 topic pub --once /agv/goal_node std_msgs/msg/Int32 "{data: -1}"    # cancel / stop
ros2 topic echo /agv/status                                              # state, route, next tag, pose
```
Status states: `IDLE` `TURN` (rotating to the lane) `DRIVE` `CENTRE` (down camera centring on the tag)
`BLOCKED` (lidar obstacle - waits, resumes by itself when clear) `ARRIVED`.

## 6. Tags and lanes
```
  W lane      M lane      E lane          y
   5           11          17            +4.0
   4           10          16            +2.4
   3 --18----- 9           15            +0.8      18 = W<->M connector (above chicane wall 1)
   2           8 ---19---- 14            -0.8      19 = M<->E connector (below chicane wall 2)
   1           7           13            -2.4
   0           6           12            -4.0
 x=-3.3      x=-0.22      x=3.8
```
Shortest path is by distance over the lanes (`config/tag_map.yaml`), so e.g. 0 -> 17 goes 0,1,2,3,18,9,8,19,14,15,16,17.
Make a lane one-way by setting `is_bidirectional: false` on its edge in `tag_map.yaml`, then relaunch.

## 7. Try the obstacle stop
Drop a box in lane 8 -> 19 (remove it later with the second command):
```bash
ign service -s /world/indoor_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 3000 \
  --req 'sdf: "<sdf version=\"1.6\"><model name=\"test_box\"><static>true</static><pose>0.9 -0.8 0.15 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.3 0.3 0.3</size></box></geometry><material><ambient>1 0 0 1</ambient></material></visual></link></model></sdf>"'
ign service -s /world/indoor_world/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 3000 --req 'name: "test_box" type: 2'
```
Send the goal to 14 from tag 8: the robot stops ~0.25 m before the box (status `BLOCKED`, red stop zone in RViz) and continues after you remove it.

## 8. Stop everything
`Ctrl+C` in terminal 2, then in terminal 1.

## 9. Troubleshooting
| Symptom | Cause / fix |
|---|---|
| "not localised yet (no tag seen)" | no tag has been seen: step 4 |
| goal click does nothing, warning "not on a tag" | click closer than 0.6 m to a tag square |
| robot does not move, status `BLOCKED` | lidar sees something in the corridor ahead (check the red zone in RViz) |
| robot does not move, no status | `/cmd_vel_nav` -> `/cmd_vel` needs `safety_stop` running and lidar `/scan` publishing; no lidar for 0.5 s = stop |
| "tag fix waiting ..." warnings | normal: the localizer waits for TF of a detection and retries |
| robot believes it is somewhere else (drifted; seen once, cause not found) | cancel (`-1`) and put it on a tag again (step 4); a large correction is accepted only after ~8 consistent fixes, so give it a second |
| RViz camera panels empty | check that `/agv/front/tags_debug` and `/agv/down/tags_debug` are publishing (`ros2 topic hz`) |
