# Version 1 - the first AprilTag AGV (kept for reference)

The original setup: a small AGV (`my_robot2`, 0.40 x 0.30 m, front camera + down camera + lidar) in a 10 x 10 m room with two chicane walls,
a table, a cube and a cylinder, and **20 AprilTags** (ids 0 - 19) on the floor in three lane columns joined by two connector tags.
The navigation stack is the same package as in Version 2 (`src/agv_tag_nav`); only the settings differ (default tag map `tag_map.yaml`, footprint 0.40 x 0.30 m).

Package `agv_v1_description` (renamed from `my_bot_description`; it is built together with everything else by `colcon build` in the repository root):
`urdf/` robot model, `worlds/indoor_world.sdf` (the room with the tags) and `lane_world.sdf`, `models/apriltag_36h11_000NN` (the 20 tags),
`config/apriltag_layout.yaml` (tag positions) + `scripts/place_apriltags.py` (writes the tags into the world), `maps/indoor_map.*`,
`launch/sim_ign.launch.py`, and older helpers (`slam.launch.py`, `rsp.launch.py`, Nav2 files, `ros2_amr_ws/ros2_hmi2.py`: a separate pygame HMI that is not part of the stack).

## Run
```bash
source ~/ros2_agv/install/setup.bash
ros2 launch agv_v1_description sim_ign.launch.py            # terminal 1: simulator
ros2 launch agv_tag_nav agv_tag_nav.launch.py               # terminal 2: navigation stack + RViz
# the robot spawns at (0,0) where no tag is visible: put it on tag 8 once
ign service -s /world/indoor_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 \
  --req 'name: "my_robot2" position {x: -0.22 y: -0.8 z: 0.05} orientation {w: 1.0}'
ros2 topic pub --once /agv/goal_node std_msgs/msg/Int32 "{data: 14}"
```
Tag layout: lane columns W x = -3.3 (ids 0-5), M x = -0.22 (6-11), E x = 3.8 (12-17), rows y = -4.0 ... 4.0; connectors 18 (W-M) and 19 (M-E).
Click a tag in RViz (2D Goal Pose / Publish Point) or publish the goal as above. The explanation of the stack is in the main [README](../README.md).

Known leftovers of Version 1: `maps/indoor_map.yaml` and `config/nav2_params.yaml` are Nav2 files that the AprilTag stack does not use; the robot radius in `nav2_params.yaml` (0.12 m) does not match the robot (about 0.30 m).
