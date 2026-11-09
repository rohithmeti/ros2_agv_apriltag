# START - command cheat-sheet (details and explanations: [README.md](README.md))

Every new terminal: `source ~/ros2_agv/install/setup.bash`. First time only: install the packages and build, see README section 2.

## Version 2 (warehouse, 282 tags, orthogonal lanes)
| What | Command |
|---|---|
| everything (sim + navigation + RViz) | `ros2 launch agv_tag_nav agv01_navigation.launch.py` |
| without RViz / older diagonal map | `... agv01_navigation.launch.py rviz:=false` / `layout:=grid` |
| goal from RViz | toolbar **2D Goal Pose** or **Publish Point**, click a tag |
| goal from the terminal | `ros2 topic pub --once /agv/goal_node std_msgs/msg/Int32 "{data: 253}"`  (`-1` cancels) |
| status | `ros2 topic echo /agv/status` |
| drive by hand | Gazebo left panel (Teleop): tick the keyboard box, click the 3D view, use the keys |
| map a world with SLAM | `ros2 launch agv_tag_nav agv01_mapping.launch.py`, then `src/agv_tag_nav/scripts/save_map.sh <name>` |
| check drift vs Gazebo ground truth | `python3 src/agv_tag_nav/scripts/drift_check.py 253 100 1` (goal tags) or no arguments = just watch |
| build a tag map for another map/world | `python3 src/agv_tag_nav/scripts/make_tag_ortho.py --map <map.yaml> --world-in <world.sdf> --name <name>` |
| stop everything | `src/agv_tag_nav/scripts/stop_all.sh` |

The AGV always starts on **tag 1**, which is in the middle of the warehouse. Another start pose: `x:=.. y:=.. yaw:=..` (must be on a tag).

## Version 1 (old: 10 x 10 m room, 20 tags)
```bash
ros2 launch agv_v1_description sim_ign.launch.py            # terminal 1
ros2 launch agv_tag_nav agv_tag_nav.launch.py               # terminal 2
ign service -s /world/indoor_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 \
  --req 'name: "my_robot2" position {x: -0.22 y: -0.8 z: 0.05} orientation {w: 1.0}'      # put the robot on tag 8 once
ros2 topic pub --once /agv/goal_node std_msgs/msg/Int32 "{data: 14}"
```

## Tips
* Only one goal is active: a new click replaces the old one. Wait for `localised from tag N` before sending goals.
* New files need `colcon build --symlink-install` again; edits to existing files do not.
