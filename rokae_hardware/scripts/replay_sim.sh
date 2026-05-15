#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash rokae_hardware/scripts/replay_sim.sh <bag_path> [rate] [with_clock]
# Example:
#   bash rokae_hardware/scripts/replay_sim.sh ~/rosbags/pro7_sim_2026-04-07_12-00-00
#   bash rokae_hardware/scripts/replay_sim.sh ~/rosbags/sr4_sim_2026-04-07_12-00-00 0.5
#   bash rokae_hardware/scripts/replay_sim.sh ~/rosbags/er7_sim_2026-04-07_12-00-00 1.0 true
#
# Notes:
#   1) Start sim first with use_sim_time (mode=sim).
#   2) Then run this script in another terminal.

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[replay_sim] ros2 command not found. Please source your ROS 2 workspace first."
  exit 1
fi

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <bag_path> [rate] [with_clock]"
  exit 1
fi

BAG_PATH="$1"
RATE="${2:-1.0}"
WITH_CLOCK="${3:-false}"
if [[ ! -d "$BAG_PATH" ]]; then
  echo "[replay_sim] Bag path not found: $BAG_PATH"
  exit 1
fi

echo "[replay_sim] Playing bag: $BAG_PATH"
echo "[replay_sim] Rate: $RATE"
echo "[replay_sim] with_clock: $WITH_CLOCK"
echo "[replay_sim] Replaying command topics only."

PLAY_ARGS=(-r "$RATE" --topics
  /arm_controller/joint_trajectory
  /arm_controller/follow_joint_trajectory/_action/goal
  /arm_controller/follow_joint_trajectory/_action/cancel_goal
  /arm_controller/follow_joint_trajectory/_action/feedback
  /arm_controller/follow_joint_trajectory/_action/status
  /arm_controller/follow_joint_trajectory/_action/result
  /position_joint_trajectory_controller/joint_trajectory
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/goal
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/feedback
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/status
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/result
)

if [[ "$WITH_CLOCK" == "true" || "$WITH_CLOCK" == "1" ]]; then
  PLAY_ARGS=(--clock "${PLAY_ARGS[@]}")
fi

ros2 bag play "$BAG_PATH" "${PLAY_ARGS[@]}"

