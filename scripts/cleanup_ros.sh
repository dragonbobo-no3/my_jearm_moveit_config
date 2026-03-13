#!/usr/bin/env bash
# Standalone cleanup script — run this if ros2 launch leaves zombie processes.
#
# Usage: ./scripts/cleanup_ros.sh

set -euo pipefail

PROCS=(
  "ros2_control_node"
  "move_group"
  "rviz2"
  "robot_state_publisher"
  "static_transform_publisher"
  "spawner"
  "joint_state_pub"
  "je_arm_control_stack"   # in case wrapper itself is orphaned
)

echo "=== Killing stale ROS2 processes ==="
for proc in "${PROCS[@]}"; do
  count=$(pgrep -fc "$proc" 2>/dev/null || echo 0)
  if [[ "$count" -gt 0 ]]; then
    pkill -9 -f "$proc" 2>/dev/null && echo "  killed ($count): $proc"
  fi
done

echo ""
echo "=== Resetting ros2 daemon ==="
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true

echo ""
echo "=== Remaining ROS2-related processes ==="
pgrep -a -f "ros2|rviz|move_group|ros2_control" 2>/dev/null || echo "  (none)"

echo ""
echo "Done."
