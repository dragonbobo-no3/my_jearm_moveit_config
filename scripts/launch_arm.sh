#!/usr/bin/env bash
# Wrapper script for je_arm_control_stack.launch.py
# Guarantees all child processes are killed on exit (Ctrl+C, kill, or error)
#
# Usage:
#   ./scripts/launch_arm.sh
#   ./scripts/launch_arm.sh --ros-args -p some_param:=value

set -euo pipefail

# --- Process names to kill on shutdown ---
ROS_PROCS=(
  "ros2_control_node"
  "move_group"
  "rviz2"
  "robot_state_publisher"
  "static_transform_publisher"
  "spawner"           # controller_manager spawner
  "joint_state_pub"
)

cleanup() {
  echo ""
  echo "[launch_arm] Shutting down — killing child processes..."

  # 1. Send SIGINT to the whole process group (so ros2 launch can clean up first)
  kill -INT -$$  2>/dev/null || true
  sleep 2

  # 2. Force-kill any survivors by name
  for proc in "${ROS_PROCS[@]}"; do
    pkill -9 -f "$proc" 2>/dev/null && echo "  killed: $proc" || true
  done

  # 3. Reset ros2 daemon to flush stale node registry
  ros2 daemon stop 2>/dev/null || true
  ros2 daemon start 2>/dev/null || true

  echo "[launch_arm] Cleanup complete."
}

# Trap Ctrl+C, kill, and script exit
trap cleanup INT TERM EXIT

# --- Launch ---
echo "[launch_arm] Starting control stack..."
ros2 launch my_jearm_moveit_config je_arm_control_stack.launch.py "$@" &
LAUNCH_PID=$!

# Wait for the launch process; if it dies, cleanup runs via EXIT trap
wait "$LAUNCH_PID"
