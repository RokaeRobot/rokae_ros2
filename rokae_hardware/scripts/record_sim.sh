#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash rokae_hardware/scripts/record_sim.sh [bag_output_dir] [robot_type]
# Example:
#   bash rokae_hardware/scripts/record_sim.sh ~/rosbags/pro7_sim_$(date +%F_%H-%M-%S) Pro7
#   bash rokae_hardware/scripts/record_sim.sh "" SR4
#
# Start sim first (recommended command):
#   ros2 launch rokae_hardware gazebo_moveit.launch.py \
#     mode:=sim robot_type:=<CR7|CR12|CR18|CR20|ER3|ER7|AR5L|AR5R|Pro3|Pro7|SR3|SR4|SR5> \
#     world:=empty.world enable_gui:=false enable_movej:=false

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[record_sim] ros2 command not found. Please source your ROS 2 workspace first."
  exit 1
fi

ROBOT_TYPE="${2:-robot}"
ROBOT_TYPE_LOWER="$(echo "$ROBOT_TYPE" | tr '[:upper:]' '[:lower:]')"
if [[ -n "${1:-}" ]]; then
  BAG_DIR="$1"
else
  BAG_DIR="$HOME/rosbags/${ROBOT_TYPE_LOWER}_sim_$(date +%F_%H-%M-%S)"
fi
PARENT_DIR="$(dirname "$BAG_DIR")"
mkdir -p "$PARENT_DIR"
if [[ -e "$BAG_DIR" ]]; then
  BAG_DIR="${BAG_DIR}_$(date +%F_%H-%M-%S)"
fi

echo "[record_sim] Output dir: $BAG_DIR"
echo "[record_sim] Robot type: $ROBOT_TYPE"
echo "[record_sim] Recording recommended sim topics..."
echo "[record_sim] Press Ctrl-C to stop."

# Wait for controller command topics to appear to avoid missing early commands.
echo "[record_sim] Waiting for controller topics..."
for _ in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -qE '^/arm_controller/joint_trajectory$|^/position_joint_trajectory_controller/joint_trajectory$'; then
    break
  fi
  sleep 1
done

ros2 bag record --include-hidden-topics -o "$BAG_DIR" \
  /joint_states \
  /arm_controller/joint_trajectory \
  /arm_controller/follow_joint_trajectory/_action/goal \
  /arm_controller/follow_joint_trajectory/_action/cancel_goal \
  /arm_controller/follow_joint_trajectory/_action/feedback \
  /arm_controller/follow_joint_trajectory/_action/status \
  /arm_controller/follow_joint_trajectory/_action/result \
  /position_joint_trajectory_controller/joint_trajectory \
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/goal \
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal \
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/feedback \
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/status \
  /position_joint_trajectory_controller/follow_joint_trajectory/_action/result \
  /tf \
  /tf_static \
  /clock

