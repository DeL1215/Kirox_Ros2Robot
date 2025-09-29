#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32, Bool
from kirox_robot.logreg.train import LogisticRegression

# --- 推理用（與訓練相同結構） ---
import torch
import torch.nn as nn


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

        # 參數
        self.declare_parameter(
            'model_path',
            '/home/del1215/ros2_ws/src/kirox_robot/kirox_robot/models/logreg_model.pth'
        )
        self.declare_parameter('threshold', 0.5)

        self.model_path: str = self.get_parameter('model_path').get_parameter_value().string_value
        th_param = self.get_parameter('threshold').get_parameter_value()
        self.threshold: float = float(
            getattr(th_param, 'double_value', 0.5) or getattr(th_param, 'integer_value', 0.5)
        )

        # 狀態（最新分數）
        self.vision_score: Optional[float] = None
        self.vad_score: Optional[float] = None
        self.oww_score: Optional[float] = None

        # QoS（latched）
        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 訂閱分數
        self.create_subscription(Float32, 'vision/q_bar_score', self._on_qbar, latched_qos)
        self.create_subscription(String,  'ears/vad_score',     self._on_vad_score, latched_qos)
        self.create_subscription(Float32, 'ears/oww_score',     self._on_oww_score, latched_qos)
        # 事件
        self.create_subscription(String,  'ears/vad_end',       self._on_vad_end, 10)

        # 輸出
        self.prob_pub    = self.create_publisher(Float32, 'tri/prob', 10)
        self.pred_pub    = self.create_publisher(Bool,    'tri/pred', 10)
        self.trigger_pub = self.create_publisher(Bool,    'brain/triggered', 10)
        self.face_pub    = self.create_publisher(String,  'face/animation', 10)  # ★ 新增

        # 載入模型
        self.model = self._load_model(self.model_path)
        self.model.eval()

        self.get_logger().info(f'tri_inference 啟動，使用模型：{self.model_path}，threshold={self.threshold:g}')

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
        if self.vision_score is None or self.vad_score is None or self.oww_score is None:
            self.get_logger().warn(
                f'分數不足，略過推理：vision={self.vision_score} vad={self.vad_score} oww={self.oww_score}'
            )
            return

        x = torch.tensor([[self.vision_score, self.vad_score, self.oww_score]], dtype=torch.float32)

        with torch.no_grad():
            prob = float(self.model(x).item())
            pred = prob >= self.threshold

        # 發佈
        self.prob_pub.publish(Float32(data=prob))
        self.pred_pub.publish(Bool(data=pred))
        self.trigger_pub.publish(Bool(data=pred))

        # ★ 如果 pred==1，送出 face/animation = "loading"
        if pred:
            self.face_pub.publish(String(data="loading"))
            self.get_logger().info("pred==1 → 發佈 face/animation = 'loading'")

        self.get_logger().info(
            f'推理完成: vision={self.vision_score:.6f} vad={self.vad_score:.6f} '
            f'oww={self.oww_score:.6f} -> prob={prob:.4f}, pred={int(pred)}'
        )

    # ---- Helpers ----
    def _load_model(self, model_path: str) -> nn.Module:
        model = LogisticRegression()
        if not os.path.exists(model_path):
            self.get_logger().error(f'模型檔不存在：{model_path}')
            raise FileNotFoundError(model_path)
        state = torch.load(model_path, map_location='cpu')
        model.load_state_dict(state)
        return model


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
