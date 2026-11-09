#!/bin/bash
# Stop every leftover simulation / SLAM / navigation process (pkill -f ros2 alone misses Gazebo and the bridges).
for pat in "ros2 launch" "ign gazebo" "ruby /usr/bin/ign" "parameter_bridge" "robot_state_publisher" "static_transform_publisher" \
           "slam_toolbox" "lib/rviz2/rviz2" "agv_tag_nav/lib/agv_tag_nav" "my_bot_description"; do
  pkill -TERM -f "$pat" 2>/dev/null
done
sleep 3
pkill -KILL -f "ign gazebo" 2>/dev/null
echo "stopped"
