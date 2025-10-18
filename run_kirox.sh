#!/usr/bin/env bash
# Kirox Robot GUI 啟動腳本（含 ROS2 環境、自動日誌輸出）

set -euo pipefail

# === 基本設定 ===
VENV_DIR="$HOME/venvs/ros2"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_DIR="$HOME/ros2_ws"
PKG_NAME="kirox_robot"
LAUNCH_FILE="kirox_robot.launch.py"
LOG_FILE="$HOME/kirox_gui.log"

# === 準備環境 ===
echo "[$(date '+%F %T')] 啟動中..." >> "$LOG_FILE" 2>&1

# 啟動虛擬環境
if [[ -f "${VENV_DIR}/bin/activate" ]]; then
  source "${VENV_DIR}/bin/activate"
  echo "已載入虛擬環境 ${VENV_DIR}" >> "$LOG_FILE"
else
  echo "⚠️ 找不到虛擬環境：${VENV_DIR}" >> "$LOG_FILE"
fi

# 載入 ROS2
if [[ -f "${ROS_SETUP}" ]]; then
  source "${ROS_SETUP}"
  echo "已載入 ROS2 Humble" >> "$LOG_FILE"
else
  echo "❌ 找不到 ROS2 環境檔：${ROS_SETUP}" >> "$LOG_FILE"
fi

export PYTHONNOUSERSITE=1
source "${WS_DIR}/install/setup.bash" >> "$LOG_FILE" 2>&1 || {
  echo "❌ 找不到 install/setup.bash，請先 colcon build" >> "$LOG_FILE"
  exit 1
}

# === 啟動 ROS2 Launch ===
echo "[$(date '+%F %T')] 啟動 ${PKG_NAME}/${LAUNCH_FILE}" >> "$LOG_FILE" 2>&1
exec ros2 launch "${PKG_NAME}" "${LAUNCH_FILE}" >> "$LOG_FILE" 2>&1
