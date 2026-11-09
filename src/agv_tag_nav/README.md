# agv_tag_nav - AprilTag-guided AGV navigation (ROS 2 Humble)

No Nav2 and no lidar map. The AGV's internal map is a **graph of AprilTags**
(node = tag id, edge = lane) and the tags on the floor are the physical guideline.
Tested on the sim robot in `../my_bot_description` (front camera + down camera + lidar).

## How it works
```
camera images --tag_detector--> TF  camera -> det_<cam>_<id>        (tag pose seen by the camera)
                                  |
tag_map.yaml --tag_map_node--> TF  map -> tag_<id>  (static)         (where each tag really is)
                                  |
                  tag_localizer:  map->base = (map->tag) * inverse(base->det)   => TF map -> odom
                                  |  (odometry carries the pose between tags; every tag seen corrects it)
/agv/goal_node (tag id) --tag_navigator--> Dijkstra on the graph -> /cmd_vel_nav
        per node: stop, TURN to the lane heading, DRIVE the lane, CENTRE on the tag with the down camera
/cmd_vel_nav + /scan --safety_stop--> /cmd_vel   (footprint-aware lidar stop / slow / no-rotate gate)
```

No local planner (RPP/DWB/TEB) on purpose: an AGV stays on its lane and waits for obstacles instead of steering off the
tags that localise it. The lane follower is already a pure-pursuit carrot on the lane line; `safety_stop` does the regulation.

**How to start it: see [START.md](../../START.md).**

## Files
| File | In -> Out |
|---|---|
| `config/tag_map.yaml` | the internal map: nodes (id, x, y, yaw) + edges (`is_bidirectional`) |
| `scripts/make_tag_map.py` | sim `apriltag_layout.yaml` -> `tag_map.yaml` (run once; edit the yaml after) |
| `graph.py` | yaml -> graph + Dijkstra shortest path (distance weights, one-way edges respected) |
| `tag_map_node.py` | yaml -> static TF `map->tag_N`, RViz markers `/agv/graph` |
| `tag_detector.py` | both camera images -> TF `det_front_N` / `det_down_N`, `/agv/tag_detections`, debug images (OpenCV aruco 36h11 + IPPE pose) |
| `tag_localizer.py` | detections + TF -> TF `map->odom` (down camera gain 1.0, front 0.5, front only < 2.5 m) |
| `tag_navigator.py` | `/agv/goal_node`, RViz `/goal_pose` or `/clicked_point` (nearest tag within 0.6 m) -> `/cmd_vel_nav`, `/agv/status` (state IDLE/TURN/DRIVE/CENTRE/BLOCKED/ARRIVED), `/agv/route_viz` |
| `safety_stop.py` | `/cmd_vel_nav` + `/scan` -> `/cmd_vel`, `/agv/obstacle`, `/agv/safety_viz`. Footprint 0.40 x 0.30 m (+5 cm sides), stop distance = 0.08 + v*0.25 + v^2/1.2, slows over 0.4 m before it, blocks rotation that would sweep a scan point, stops if lidar/commands go silent 0.5 s |
| `rviz/agv_tag_nav.rviz` | Fixed frame `map`, top-down: tag graph + ids, route + goal, robot, TF, front/down camera views with detected tags drawn |

## Run
See [START.md](../../START.md) (build, launch, how to put the robot on a tag, how to give goals).

## Measured (sim, 2026-11-09)
* localised from the front camera 0.36 m from a tag: exact (to 1 cm, ground truth).
* 8 -> 19 -> 14: stopped 1.9 cm from tag 14.   14 -> 0 (9 nodes, both connectors, 12 m): stopped 1.8 cm from tag 0.
* Safety: 0.3 m box placed in lane 8->19, goal 14: stopped 0.25 m before it, status BLOCKED; box removed -> resumed, arrived at 14 (1.8 cm).
* Sim wheel slip makes the odometry turn ~10% short; the down camera on the tag fixes the heading while turning in place.

## Known limits / next
* Needs one tag in view to start (no global search). Between tags it trusts odometry.
* Down camera sees only +-0.1 m around a tag; the front camera sees tags up to ~1 m reliably.
* Seen once: after the robot hit a wall (before the safety node existed) the pose estimate diverged (~0.6 m, 45 deg). Cause not found.
* Rotation blocking and the RViz stop-zone display are implemented but not tested; reversing is limited to 5 cm/s (no rear scan check).
* No blocked-edge re-plan (stops and waits), no goal heading at the last node, no multi-robot traffic.
