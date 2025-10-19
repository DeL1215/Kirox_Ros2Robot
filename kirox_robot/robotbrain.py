#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from typing import Optional
import time
import threading
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String, Float32

# --- 推理用（與訓練相同結構） ---
import json
from ament_index_python.packages import get_package_share_directory

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
      - 準備就緒條件：需等 ears/ready 與 vision/ready 皆為 True，且模型載入完成，再發布 brain/ready=True（latched）。
      - 輸出：
          tri/prob (Float32)
          tri/pred (Bool)
          brain/triggered (Bool)
          face/animation (String) ← 當 pred==1 時送出 "loading"
          brain/ready (Bool, Transient Local)
    """

    def __init__(self):
        super().__init__('robot_brain_node')

        # ---------------- 共享資料夾 ----------------
        pkg_share = get_package_share_directory('kirox_robot')
        models_dir = os.path.join(pkg_share, 'models')
        self.model_path = os.path.join(models_dir, 'logreg_model_v2.best.pth')
        self.scaler_path = os.path.join(models_dir, 'scaler.json')
        self.config_path = os.path.join(models_dir, 'config.json')

        # 狀態
        self._ready_flags: Dict[str, bool] = {}
        self._all_systems_ready = False

        # ---------------- 狀態（最新分數） ----------------
        self.vision_score: Optional[float] = None
        self.vad_score: Optional[float] = None
        self.oww_score: Optional[float] = None

        # 需要等待的節點
        # 修正：'eyes' 節點發布的是 'vision/ready'
        # 新增：'face' 節點，確保 UI 也已就緒，避免 brain 過早發布 ready 信號
        self.nodes_to_wait = ["ears", "vision", "face"]

        # ---------------- QoS（latched） ----------------
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.latched_qos = latched_qos # 讓 wait_for_ready 可以存取

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

        # ---------------- 訂閱與發布 ----------------
        # 輸出
        self.prob_pub = self.create_publisher(Float32, 'tri/prob', 10)
        self.pred_pub = self.create_publisher(Bool, 'tri/pred', 10)
        self.trigger_pub = self.create_publisher(Bool, 'brain/triggered', 10)
        self.face_pub = self.create_publisher(String, 'face/animation', 10)
        self.ready_pub = self.create_publisher(Bool, "brain/ready", self.latched_qos)

        # 訂閱分數/事件
        self.create_subscription(Float32, 'vision/q_bar_score', self._on_qbar, latched_qos)
        self.create_subscription(String, 'ears/vad_score', self._on_vad_score, latched_qos)
        self.create_subscription(Float32, 'ears/oww_score', self._on_oww_score, latched_qos)
        self.create_subscription(String, 'ears/vad_end', self._on_vad_end, 10)

        # 訂閱所有依賴節點的 ready 信號
        for name in self.nodes_to_wait:
            topic_name = f"{name}/ready"
            self.create_subscription(Bool, topic_name, lambda msg, n=name: self._on_dependency_ready(msg, n), self.latched_qos)
        self.get_logger().info(f"Brain 正在等待 {self.nodes_to_wait} 節點就緒...")

    # ---- Score Callbacks ----
    def _on_qbar(self, msg: Float32):
        self.vision_score = float(msg.data)

    def _on_vad_score(self, msg: String):
        try:
            data = json.loads(msg.data) if msg.data else {}
            score = data.get("score", 0.0) # 預設為 0.0
            self.vad_score = float(score)
        except Exception as e:
            self.get_logger().warn(f'解析 vad_score 失敗：{e}')
            self.vad_score = 0.0 # 解析失敗也給 0.0

    def _on_oww_score(self, msg: Float32):
        self.oww_score = float(msg.data)

    def _on_dependency_ready(self, msg: Bool, node_name: str):
        if not msg.data:
            return
        if not self._ready_flags.get(node_name):
            self._ready_flags[node_name] = True
            self.get_logger().info(f"✅ 收到 '{node_name}/ready' 就緒信號。")

            # 檢查是否所有依賴都已就緒
            if all(self._ready_flags.get(name) for name in self.nodes_to_wait):
                if not self._all_systems_ready:
                    self._all_systems_ready = True
                    self.get_logger().info("🎉 所有系統已就緒！RobotBrain 開始運作。")
                    self.ready_pub.publish(Bool(data=True))

    def _on_vad_end(self, msg: String):
        # 系統未就緒或分數不齊全，則不推理
        if not self._all_systems_ready:
            return
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
        # 檢查是否可能已錯過 latched 訊息
        # (此處的 wait_for_ready 簡化版不包含此檢查，但保留函式簽名以便擴充)

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


def main(args=None):
    rclpy.init(args=args)
    node = RobotBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
