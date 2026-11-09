# agv_tag_nav - AprilTag navigation stack (ROS 2 Humble)

The navigation stack of the repository. **How it works, how to install and how to run: see the main [README](../../README.md).**
It runs unchanged for Version 2 (`agv_01`, warehouse) and Version 1 (`old_agv`).

| Folder / file | What |
|---|---|
| `agv_tag_nav/tag_map_node.py`, `tag_detector.py`, `tag_localizer.py`, `tag_navigator.py`, `safety_stop.py` | the five nodes (map TFs, camera tag detection, localisation, route + driving, lidar safety stop) |
| `agv_tag_nav/graph.py`, `geom.py` | tag graph + Dijkstra, pose / TF maths |
| `launch/agv01_navigation.launch.py` | Version 2: simulator + stack + RViz (`layout:=ortho` default, `grid` = old diagonal map) |
| `launch/agv01_mapping.launch.py`, `slam_mapping.launch.py` | SLAM Toolbox mapping (with / without the simulator) |
| `launch/agv_tag_nav.launch.py` | the stack only (+ RViz); arguments `tag_map`, `footprint_length`, `footprint_width`, `rviz`. Defaults are Version 1's |
| `config/tag_map_warehouse_ortho.yaml` | Version 2 tag map: 282 tags, 444 lanes, all axis-aligned |
| `config/tag_map_warehouse_grid.yaml` | older warehouse map with diagonal lanes (backup) |
| `config/tag_map.yaml` | Version 1 map (20 tags) |
| `config/slam_agv01.yaml` | SLAM Toolbox parameters for `agv_01` |
| `maps/` | warehouse occupancy map (`warehouse.pgm/.yaml`, already in the Gazebo world frame) and previews of the tag grids |
| `rviz/` | RViz views (`agv_tag_nav.rviz`, `slam_mapping.rviz`) |
| `scripts/make_tag_ortho.py` | occupancy map + world -> orthogonal tag grid (map yaml, world with tags, textures, start pose) |
| `scripts/make_tag_grid.py` | the older generator (square lattice + diagonals) |
| `scripts/make_tag_map.py` | Version 1: tag layout -> `tag_map.yaml` |
| `scripts/drift_check.py` | position estimate vs Gazebo ground truth while driving or watching |
| `scripts/save_map.sh`, `gz_floorplan.py`, `stop_all.sh` | save the SLAM map; ground-truth occupancy map from the running simulator (experimental, only tested on synthetic data); stop all leftover processes |

Topics: goal `/agv/goal_node` (Int32 tag id, -1 cancels) or RViz `/goal_pose` / `/clicked_point`; status `/agv/status`; commands `/cmd_vel_nav` -> `safety_stop` -> `/cmd_vel`;
tag detections `/agv/tag_detections`; annotated camera images `/agv/front/tags_debug`, `/agv/down/tags_debug`; route `/agv/route_viz`; obstacle `/agv/obstacle`.
