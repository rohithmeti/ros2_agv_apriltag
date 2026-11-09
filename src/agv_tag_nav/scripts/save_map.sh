#!/bin/bash
# Save the map slam_toolbox is building:   save_map.sh <name> [dir]
#   <dir>/<name>.pgm + <name>.yaml        occupancy map (design the tag graph on this)
#   <dir>/<name>.posegraph + .data        slam_toolbox pose graph (re-open / extend the map later)
# default dir: <repo>/src/agv_tag_nav/maps
set -e
NAME=${1:?usage: save_map.sh <name> [dir]}
DIR=${2:-$(cd "$(dirname "$0")/../maps" && pwd)}
mkdir -p "$DIR"
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f "$DIR/$NAME" --ros-args -p use_sim_time:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$DIR/$NAME'}"
echo "saved: $DIR/$NAME.{pgm,yaml,posegraph,data}"
