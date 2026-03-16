#!/bin/bash
# check_dangling.sh
# 检查是否有孤儿进程（旧 ros2_control_node 或 spawner）未清理
# Usage: bash scripts/check_dangling.sh

echo "=== Checking for dangling ROS processes ==="
echo ""

echo "1. ros2 nodes (active):"
ros2 node list 2>/dev/null || echo "  (No nodes or ros2 daemon not running)"
echo ""

echo "2. Controller Manager listeners (port 8001/8000):"
ss -ltnp 2>/dev/null | egrep '8000|8001' || echo "  (No 8000/8001 listeners)"
echo ""

echo "3. Process list (ros2_control, controller_manager, spawner):"
ps aux | egrep 'ros2_control_node|controller_manager|spawner' | grep -v grep || echo "  (No matching processes)"
echo ""

echo "4. Current /joint_states publisher:"
ros2 topic info /joint_states -v 2>/dev/null | head -20 || echo "  (Topic not available or no subscriber)"
echo ""

echo "5. Current jearm_controller state:"
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}' 2>/dev/null | head -30 || echo "  (Controller_manager not responding)"
echo ""

echo "=== Summary ==="
echo "If you see old ros2_control_node or spawner processes above, run:"
echo "  bash scripts/cleanup_ros.sh"
echo ""
echo "If you want to cleanly restart, run:"
echo "  bash scripts/clean_restart.sh"

