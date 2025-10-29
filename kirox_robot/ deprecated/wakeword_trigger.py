#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
oww_node.py — openWakeWord ROS2 節點（精簡版）
規則：
- 訂閱 ears/vad_start → 啟動麥克風串流與 OWW 推論
- 訂閱 ears/vad_end   → 停止麥克風串流與 OWW 推論
輸出（latched）:
- /ears/oww_score (std_msgs/Float32)  只送 score

參數（--ros-args 可覆蓋）：
- model_path (string)  預設：/home/del1215/ros2_ws/src/kirox_robot/kirox_robot/models/kirox.onnx
- device_name (string) 預設：default  （可用 "pulse" 或裝置名稱/索引）
- rate (int)           預設：16000
- frame_ms (int)       預設：80

僅供列印觀察（不影響發佈）：
- thresh (double)      預設：0.5
- debounce_n (int)     預設：2
- log_interval_s (double) 預設：0.5
"""

import os
import time
import threading
import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String as StringMsg
from openwakeword.model import Model


class WakeWordTriggerNode(Node):
    def __init__(self):
        super().__init__('wakeword_trigger_node')

        # 參數
        package_share_dir = get_package_share_directory('kirox_robot')
        default_model_path = os.path.join(package_share_dir, 'models', 'kirox.onnx')
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('device_name', 'pulse')
        self.declare_parameter('rate', 16000)
        self.declare_parameter('frame_ms', 80)
        self.declare_parameter('thresh', 0.5)
        self.declare_parameter('debounce_n', 2)
        self.declare_parameter('log_interval_s', 0.5)

        p = self.get_parameter
        self.model_path = p('model_path').get_parameter_value().string_value
        self.device_name = p('device_name').get_parameter_value().string_value
        self.rate = int(p('rate').get_parameter_value().integer_value)
        self.frame_ms = int(p('frame_ms').get_parameter_value().integer_value)
        self.thresh = float(p('thresh').get_parameter_value().double_value)
        self.debounce_n = int(p('debounce_n').get_parameter_value().integer_value)
        self.log_interval_s = float(p('log_interval_s').get_parameter_value().double_value)

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f'模型不存在: {self.model_path}')

        # 發佈者（latched）
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_score = self.create_publisher(Float32, 'ears/oww_score', latched)

        # 訂閱 robotears 訊號
        self.create_subscription(StringMsg, 'ears/vad_start', self._on_vad_start, 10)
        self.create_subscription(StringMsg, 'ears/vad_end', self._on_vad_end, 10)

        # OWW 模型
        self.model = Model(
            wakeword_models=[self.model_path],
            enable_speex_noise_suppression=False
        )

        # 以靜音推一次拿到 key
        block = int(self.rate * (self.frame_ms / 1000.0))
        if block <= 0:
            raise ValueError(f"frame_ms 太小：{self.frame_ms}")
        probe = self.model.predict(np.zeros(block, dtype=np.int16))
        if not probe:
            raise RuntimeError('openWakeWord 模型載入失敗：predict 回傳空字典')
        self._score_key = next(iter(probe.keys()))
        self.get_logger().info(f"[oww] score key = {self._score_key}")

        # 串流狀態
        self._stream = None
        self._stream_lock = threading.Lock()

        # 列印/觸發狀態
        self._above_cnt = 0
        self._max_score = 0.0
        self._last_print = 0.0

        # 初始發 0.0（latched）
        self.pub_score.publish(Float32(data=0.0))
        self.get_logger().info("[oww] node ready")

    # --- 同步 robotears ---
    def _on_vad_start(self, _msg: StringMsg):
        self.get_logger().info("recv ears/vad_start -> start stream")
        self._start_stream()

    def _on_vad_end(self, _msg: StringMsg):
        self.get_logger().info("recv ears/vad_end -> stop stream")
        self._stop_stream()

    # --- 開/關串流（精簡版，直接用 device_name/rate/block） ---
    def _start_stream(self):
        with self._stream_lock:
            if self._stream is not None:
                self.get_logger().info("audio stream already running; skip")
                return
            block = int(self.rate * (self.frame_ms / 1000.0))
            self._above_cnt = 0
            self._max_score = 0.0
            self._last_print = 0.0
            try:
                self._stream = sd.RawInputStream(
                    device=self.device_name,
                    channels=1,
                    samplerate=self.rate,
                    blocksize=block,
                    dtype='int16',
                    callback=self._audio_cb
                )
                self._stream.__enter__()
                self.get_logger().info(f"[audio] started device={self.device_name} rate={self.rate} block={block}")
            except Exception as e:
                self._stream = None
                self.get_logger().error(f"[audio] start failed: {e}")

    def _stop_stream(self):
        with self._stream_lock:
            if self._stream is None:
                self.get_logger().info("audio stream not running; skip")
                return
            try:
                self._stream.__exit__(None, None, None)
                self.get_logger().info("[audio] stopped")
            except Exception as e:
                self.get_logger().warn(f"[audio] stop error: {e}")
            finally:
                self._stream = None

    # --- 音訊 callback ---
    def _audio_cb(self, indata, frames, t, status):
        if status:
            print("SD status:", status)

        data = np.frombuffer(indata, dtype=np.int16)
        scores = self.model.predict(data)
        s = float(scores.get(self._score_key, 0.0))

        # 發佈 score（Float32）
        self.pub_score.publish(Float32(data=s))

        # 列印/觸發提示（觀察用）
        if s > self._max_score:
            self._max_score = s
        now = time.time()
        if (now - self._last_print) >= self.log_interval_s:
            self._last_print = now
            print(f"[Wakeword] score={s:.3f}, max={self._max_score:.3f}")
        if s > self.thresh:
            self._above_cnt += 1
            if self._above_cnt >= self.debounce_n:
                print(f"[TRIGGER] {self._score_key} score={s:.3f} (≥ {self.thresh})")
                self._above_cnt = 0
        else:
            if self._above_cnt > 0:
                self._above_cnt -= 1

    # --- 關閉 ---
    def destroy_node(self):
        self._stop_stream()
        super().destroy_node()


def main():
    rclpy.init()
    node = WakeWordTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
