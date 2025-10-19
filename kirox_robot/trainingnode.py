#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import json
import queue
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32

# --- Tkinter 小對話框 ---
import tkinter as tk
from tkinter import messagebox

# 僅保留四欄：三個分數 + 輸出(0/1)
CSV_PATH = "/home/jetson/ros2_ws/src/Kirox_Ros2Robot/kirox_robot/logreg/tri_scores.csv"


class SimpleProbe(Node):
    """
    訂閱：
      - vision/q_bar_score : Float32 (latched)
      - ears/vad_score     : String  (latched, JSON -> score)
      - ears/oww_score     : Float32 (latched)
      - ears/vad_end       : String  (事件，JSON 可能含 path/ts/…)
    收到 vad_end：把目前三分數推到 GUI 佇列，彈窗詢問「觸發=1 / 不觸發=0 / 取消(不紀錄)」。
    """
    def __init__(self, gui_queue: queue.Queue):
        super().__init__('simple_probe')

        self.gui_queue = gui_queue

        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.vision_score = None
        self.vad_score = None
        self.oww_score = None

        # 可能由 vad_end JSON 提供（此版已不寫入 CSV）
        self.latest_meta = {}

        # 訂閱分數（latched）
        self.create_subscription(Float32, 'vision/q_bar_score', self._on_qbar, latched_qos)
        self.create_subscription(String,  'ears/vad_score',     self._on_vad_score, latched_qos)
        self.create_subscription(Float32, 'ears/oww_score',     self._on_oww_score, latched_qos)

        # 事件（一般 QoS）
        self.create_subscription(String,  'ears/vad_end',       self._on_vad_end, 10)

        self.get_logger().info('simple_probe 啟動，等待訊息中…')

    def _on_qbar(self, msg: Float32):
        self.vision_score = float(msg.data)

    def _on_vad_score(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.vad_score = float(data.get("score", None))
        except Exception:
            self.vad_score = None

    def _on_oww_score(self, msg: Float32):
        self.oww_score = float(msg.data)

    def _on_vad_end(self, msg: String):
        # 解析 meta（目前不寫入 CSV，但保留以便你需要）
        try:
            self.latest_meta = json.loads(msg.data) if msg.data else {}
        except Exception:
            self.latest_meta = {}

        # 推一筆標註任務到 GUI 佇列
        item = {
            "vision_score": self.vision_score,
            "vad_score": self.vad_score,
            "oww_score": self.oww_score,
            "meta": self.latest_meta
        }
        self.gui_queue.put(item)

        # 終端列印一次（便於觀察）
        print(f'>>> vision_score={self.vision_score}  vad_score={self.vad_score}  oww_score={self.oww_score}')


# ---- GUI：負責彈窗與寫 CSV ----
class LabelGUI:
    def __init__(self, root: tk.Tk, gui_queue: queue.Queue):
        self.root = root
        self.queue = gui_queue
        self.root.title("SimpleProbe 標註器")
        self.root.geometry("320x60")
        self.root.resizable(False, False)

        # 隱藏主視窗，只用 messagebox
        self.root.withdraw()

        # 準備 CSV：僅四欄
        os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)
        if not os.path.exists(CSV_PATH):
            with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["vision_score", "vad_score", "oww_score", "output"])

        # 啟動輪詢
        self.root.after(200, self._poll_queue)

    def _poll_queue(self):
        try:
            while True:
                item = self.queue.get_nowait()
                self._handle_item(item)
        except queue.Empty:
            pass
        self.root.after(200, self._poll_queue)

    def _fmt(self, x):
        return "N/A" if x is None else f"{x:.3f}"

    def _to_csv_field(self, x):
        # 寫入 CSV 的實值；None → 空字串
        return "" if x is None else f"{x:.6f}"

    def _handle_item(self, item: dict):
        v = item.get("vision_score", None)
        a = item.get("vad_score", None)
        o = item.get("oww_score", None)

        # 改用 askyesnocancel：Yes=1, No=0, Cancel(None)=不紀錄
        msg = (
            f"vision_score: {self._fmt(v)}\n"
            f"vad_score   : {self._fmt(a)}\n"
            f"oww_score   : {self._fmt(o)}\n\n"
            f"是否標註為『觸發(=1)』？\n"
            f"（否=0；取消或按視窗 X = 不紀錄）"
        )
        res = messagebox.askyesnocancel("標註資料", msg)
        # res ∈ {True, False, None}
        if res is None:
            print("[skip] 使用者按下取消/視窗X，本筆不紀錄。")
            return

        label = 1 if res is True else 0

        # 寫入 CSV（僅四欄）
        with open(CSV_PATH, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([
                self._to_csv_field(v),
                self._to_csv_field(a),
                self._to_csv_field(o),
                label
            ])

        print(f"[saved] vision={v}  vad={a}  oww={o}  output={label} -> {CSV_PATH}")


def main():
    # 建立 GUI 佇列
    gui_queue = queue.Queue()

    # ---- 啟動 ROS2（背景執行緒 spin） ----
    rclpy.init()
    node = SimpleProbe(gui_queue=gui_queue)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # ---- 啟動 Tk（主執行緒）----
    root = tk.Tk()
    app = LabelGUI(root, gui_queue)

    def on_close():
        # 關閉主程式（與單筆標註無關）
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
