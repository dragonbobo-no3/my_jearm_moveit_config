#!/bin/bash
# clean_restart.sh
# 完全清理 ros2_control_node 和 controllers，然后重新启动 launch
# 使用临时 ROS_DOMAIN_ID 隔离旧的 DDS context
# 
# Usage: 
#   cd ~/ros2_ws
#   source install/setup.bash
#   bash src/my_jearm_moveit_config/scripts/clean_restart.sh

set -e

echo "=== Step 1: Killing all ROS processes ==="
pkill -9 -f "ros2_control_node|controller_manager|spawner|move_group|rviz2|robot_state_publisher|static_transform_publisher|joint_state" 2>/dev/null || true
sleep 1

echo ""
echo "=== Step 2: Stopping and restarting ros2 daemon ==="
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start
sleep 2

echo ""
echo "=== Step 3: Clearing old DDS state by switching domain ID (temporary) ==="
# 使用临时的 domain ID（199）来隔离旧的 DDS context
export ROS_DOMAIN_ID=199
sleep 1
echo "Temporary ROS_DOMAIN_ID set to 199"

echo ""
echo "=== Step 4: Starting fresh launch with isolated domain ==="
echo "Starting: ROS_DOMAIN_ID=199 ros2 launch my_jearm_moveit_config je_arm_control_stack.launch.py"
echo ""

# 用临时 domain 启动，这会创建一个全新的 DDS context，不会受旧节点影响
ROS_DOMAIN_ID=199 ros2 launch my_jearm_moveit_config je_arm_control_stack.launch.py

