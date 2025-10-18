#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String, Float32, Bool

# --- 推理用（與訓練相同結構） ---
import numpy as np
import torch
import torch.nn as nn


# ---------------- 模型定義（與 train.py 一致） ----------------
class SimpleNN(nn.Module):
    def __init__(self):
        super(SimpleNN, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(3, 16),
            nn.ReLU(),
            nn.Linear(16, 8),
            nn.ReLU(),
            nn.Linear(8, 1),
            nn.Sigmoid()  # 二元分類機率輸出
        )

    def forward(self, x):
        return self.net(x)


class RobotBrainNode(Node):
    """
    功能：
      - 接收三個分數 (vision/vad/oww)，在收到 vad_end 事件時做一次推理。
      - 輸出：
          tri/prob (Float32)
          tri/pred (Bool)
          brain/triggered (Bool)
          face/animation (String) ← 當 pred==1 時送出 "loading"
    """

    def __init__(self):
        super().__init__('robot_brain_node')

        # ---------------- 直接抓共享資料夾（無需啟動參數） ----------------
        pkg_share = get_package_share_directory('kirox_robot')
        models_dir = os.path.join(pkg_share, 'models')
        self.model_path  = os.path.join(models_dir, 'logreg_model_v2.best.pth')
        self.scaler_path = os.path.join(models_dir, 'scaler.json')
        self.config_path = os.path.join(models_dir, 'config.json')

        # ---------------- 狀態（最新分數） ----------------
        self.vision_score: Optional[float] = None
        self.vad_score: Optional[float] = None
        self.oww_score: Optional[float] = None

        # ---------------- QoS（latched） ----------------
        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---------------- 訂閱分數/事件 ----------------
        self.create_subscription(Float32, 'vision/q_bar_score', self._on_qbar, latched_qos)
        self.create_subscription(String,  'ears/vad_score',     self._on_vad_score, latched_qos)
        self.create_subscription(Float32, 'ears/oww_score',     self._on_oww_score, latched_qos)
        self.create_subscription(String,  'ears/vad_end',       self._on_vad_end, 10)

        # ---------------- 輸出 ----------------
        self.prob_pub    = self.create_publisher(Float32, 'tri/prob', 10)
        self.pred_pub    = self.create_publisher(Bool,    'tri/pred', 10)
        self.trigger_pub = self.create_publisher(Bool,    'brain/triggered', 10)
        self.face_pub    = self.create_publisher(String,  'face/animation', 10)

        # ---------------- 載入模型/標準化/閾值 ----------------
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model, self.mu, self.sigma, self.threshold = self._load_all()
        self.model.eval()

        self.get_logger().info(
            f'tri_inference 啟動：\n'
            f'  model= {self.model_path}\n'
            f'  scaler={self.scaler_path}\n'
            f'  config={self.config_path}\n'
            f'  threshold={self.threshold:g}\n'
            f'  device={self.device}'
        )

    # ---- Callbacks ----
    def _on_qbar(self, msg: Float32):
        self.vision_score = float(msg.data)

    def _on_vad_score(self, msg: String):
        try:
            data = json.loads(msg.data) if msg.data else {}
            score = data.get("score", None)
            self.vad_score = float(score) if score is not None else None
        except Exception as e:
            self.get_logger().warn(f'解析 vad_score 失敗：{e}')
            self.vad_score = None

    def _on_oww_score(self, msg: Float32):
        self.oww_score = float(msg.data)

    def _on_vad_end(self, msg: String):
        # 三分數齊備才推理
        if self.vision_score is None or self.vad_score is None or self.oww_score is None:
            self.get_logger().warn(
                f'分數不足，略過推理：vision={self.vision_score} vad={self.vad_score} oww={self.oww_score}'
            )
            return

        # 形成輸入，套用標準化
        tri = np.array([self.vision_score, self.vad_score, self.oww_score], dtype=np.float32)
        tri_std = (tri - self.mu) / self.sigma
        x = torch.tensor(tri_std[None, :], dtype=torch.float32, device=self.device)

        with torch.no_grad():
            prob = float(self.model(x).item())
            pred = prob >= self.threshold

        # 發佈
        self.prob_pub.publish(Float32(data=prob))
        self.pred_pub.publish(Bool(data=pred))
        self.trigger_pub.publish(Bool(data=pred))

        # 若 pred==1，同步驅動表情
        if pred:
            self.face_pub.publish(String(data="loading"))
            self.get_logger().info("pred==1 → 發佈 face/animation = 'loading'")

        self.get_logger().info(
            f'推理完成: vision={self.vision_score:.6f} vad={self.vad_score:.6f} '
            f'oww={self.oww_score:.6f} -> prob={prob:.4f}, pred={int(pred)}'
        )

    # ---- Helpers ----
    def _load_all(self):
        # 1) 載入模型權重
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'模型檔不存在：{self.model_path}')
            raise FileNotFoundError(self.model_path)

        model = SimpleNN().to(self.device)
        state = torch.load(self.model_path, map_location=self.device)
        model.load_state_dict(state)

        # 2) 載入標準化參數
        if not os.path.exists(self.scaler_path):
            self.get_logger().error(f'找不到 scaler.json：{self.scaler_path}')
            raise FileNotFoundError(self.scaler_path)

        with open(self.scaler_path, "r") as f:
            s = json.load(f)
        mu = np.array(s["mu"], dtype=np.float32)
        sigma = np.array(s["sigma"], dtype=np.float32)
        sigma[sigma == 0] = 1.0  # 防零除

        # 3) 載入最佳閾值（config 不存在則預設 0.5）
        if not os.path.exists(self.config_path):
            self.get_logger().warn(f'找不到 config.json：{self.config_path}，改用預設 0.5 閾值')
            best_thr = 0.5
        else:
            with open(self.config_path, "r") as f:
                cfg = json.load(f)
            best_thr = float(cfg.get("best_threshold", 0.5))

        return model, mu, sigma, best_thr


def main():
    rclpy.init()
    node = RobotBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
