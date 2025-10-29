#!/usr/bin/env bash
# 只啟動（不建置、不清理）版本

set -e -o pipefail   # 不用 -u，避免 ROS setup.bash 讀未宣告變數時中斷

# === 基本設定 ===
VENV_DIR="$HOME/venvs/ros2"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_DIR="$HOME/ros2_ws"
export PYTHONNOUSERSITE=1

# 1) 啟動 venv
if [[ -f "${VENV_DIR}/bin/activate" ]]; then
  source "${VENV_DIR}/bin/activate"
fi

# 2) 載入 ROS2 Humble
source "${ROS_SETUP}"

# 3) 載入已建置好的工作區
source "${WS_DIR}/install/setup.bash"

# 4) （可選）檢視可執行檔的 shebang，純檢查用
if [[ -f "${WS_DIR}/install/kirox_robot/lib/kirox_robot/robotface" ]]; then
  head -n1 "${WS_DIR}/install/kirox_robot/lib/kirox_robot/robotface" || true
fi

# 5) 啟動（不做建置、不清理）
exec ros2 launch kirox_robot kirox_robot.launch.py
