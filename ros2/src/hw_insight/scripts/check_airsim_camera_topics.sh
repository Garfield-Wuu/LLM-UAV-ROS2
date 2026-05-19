#!/usr/bin/env bash
# Phase 0: 预检 AirSim 相机与里程计话题（不启动 YOLO）
# 用法：
#   cd /home/hw/hw-ros2/ros2 && source /opt/ros/humble/setup.bash && source install/setup.bash
#   bash src/hw_insight/scripts/check_airsim_camera_topics.sh
#
# 可选：第一个参数为超时秒数（默认 8），每个话题最多等待这么久收一条消息。

set -euo pipefail

TIMEOUT="${1:-8}"

TOPICS=(
  "/airsim_node/PX4/CameraDepth1/Scene"
  "/airsim_node/PX4/CameraDepth1/DepthPlanar"
  "/airsim_node/PX4/CameraDepth1/camera_info"
  "/airsim_node/PX4/odom_local_ned"
)

echo "=== YOLO-World AirSim 在线测试 · 阶段 0 话题预检 ==="
echo "超时: ${TIMEOUT}s / 话题"
echo ""

for t in "${TOPICS[@]}"; do
  echo "--- $t ---"
  if timeout "${TIMEOUT}" ros2 topic echo "$t" --once >/dev/null 2>&1; then
    echo "OK: 收到至少一条消息"
  else
    echo "FAIL: 在 ${TIMEOUT}s 内未收到消息（确认 AirSim + airsim_node 已启动，相机名与 settings 一致）"
  fi
  echo ""
done

echo "提示: 若 Scene 无数据，检查 AirSim settings 中 CameraDepth1 与 ImageType 0 (RGB)。"
echo "若仅测 2D 检测，DepthPlanar / camera_info 缺失时可跳过阶段 2 的 3D 部分。"
