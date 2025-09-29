#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobotFace (Layered, 16:9 Letterbox, ROS2-controlled)
- 無音訊偵測：僅透過 ROS2 topic 控制
  - face/animation : std_msgs/String → 切換主動畫
  - face/speaking  : std_msgs/Bool   → 嘴巴動畫開/關
- 16:9 舞台（黑底），視窗任意大小時維持等比，左右/上下 letterbox
- 圖層化：BackgroundLayer(主動畫) + MouthLayer(嘴巴動畫)
- 動畫快取：避免重複載入 GIF，降低 CPU/IO
- Esc 離開；F11 全螢幕切換
"""

from __future__ import annotations
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString, Bool as RosBool

from PySide6.QtCore import Qt, QRect, QSize, QTimer
from PySide6.QtGui import QMovie, QColor, QPalette, QKeyEvent
from PySide6.QtWidgets import QApplication, QWidget, QLabel

# ========== 使用者可調整區 ==========
# 主動畫對應表：name → (path, speed, size, offset)
# size = (w,h) 或 None(自適應舞台)，offset = (dx, dy) 以舞台中心為原點
ANIM_MAP: Dict[str, Tuple[Optional[str], float, Optional[Tuple[int,int]], Tuple[int,int]]] = {
    "empty":   ("kirox_robot/kirox_robot/face_ui/assets/Todo.gif", 1.0, None, (0, 0)),  # 無主動畫 (name): (path, speed, size, offset)
    "loading": ("kirox_robot/kirox_robot/face_ui/assets/Todo.gif", 0.8, (600, 600), (0, 0)),
    "idle":    ("kirox_robot/kirox_robot/face_ui/assets/Todo.gif",   1.0, None, (0, 0)),
    "happy":   ("kirox_robot/kirox_robot/face_ui/assets/Todo.gif",  1.2, (420, 420), (0, -40)),
    "angry":   ("kirox_robot/kirox_robot/face_ui/assets/Todo.gif",  0.9, (420, 420), (0,  40)),
    "talk":    ("kirox_robot/kirox_robot/face_ui/assets/Todo.gif",   1.0, None, (0, 0)),
}
DEFAULT_ANIM = "empty"

# 嘴巴動畫（speaking=True 時顯示播放；False 停止隱藏）
MOUTH_GIF_PATH = "kirox_robot/kirox_robot/face_ui/assets/Todo.gif"
MOUTH_SPEED    = 1.0
MOUTH_SIZE     = (380, 160)   # 依你的 GIF 調整比例
MOUTH_OFFSET   = (0, 180)     # 置中下方些，單位像素
# ===================================


@dataclass
class LayerSpec:
    path: Optional[Path]
    speed: float
    size: Optional[Tuple[int, int]]  # (w,h) or None
    offset: Tuple[int, int]          # (dx,dy)


class MovieCache:
    """簡單的 QMovie 快取，避免重複載入檔案。"""
    def __init__(self):
        self._cache: Dict[Path, QMovie] = {}

    def get(self, path: Path) -> QMovie:
        mv = self._cache.get(path)
        if mv is None:
            mv = QMovie(str(path))
            self._cache[path] = mv
        return mv


class Stage(QWidget):
    """16:9 舞台容器：自動 letterbox，提供兩個疊放圖層。"""
    def __init__(self, aspect_w=16, aspect_h=9):
        super().__init__()
        self.aspect_w, self.aspect_h = aspect_w, aspect_h

        # 黑底
        pal = self.palette()
        pal.setColor(QPalette.Window, QColor(0, 0, 0))
        self.setAutoFillBackground(True)
        self.setPalette(pal)

        # 舞台（實際 16:9 區域）
        self.stage = QWidget(self)
        stage_pal = self.stage.palette()
        stage_pal.setColor(QPalette.Window, QColor(0, 0, 0))
        self.stage.setAutoFillBackground(True)
        self.stage.setPalette(stage_pal)

        # 圖層：主動畫（底層）與嘴巴（上層）
        self.bg_label = QLabel(self.stage)
        self.bg_label.setAlignment(Qt.AlignCenter)
        self.bg_label.setStyleSheet("background: transparent;")

        self.mouth_label = QLabel(self.stage)
        self.mouth_label.setAlignment(Qt.AlignCenter)
        self.mouth_label.setStyleSheet("background: transparent;")

        # Z 順序：背景在下，嘴巴在上
        self.bg_label.lower()
        self.mouth_label.raise_()

        self._bg_spec: Optional[LayerSpec] = None
        self._mouth_spec: Optional[LayerSpec] = None
        self._bg_movie: Optional[QMovie] = None
        self._mouth_movie: Optional[QMovie] = None

        self._movie_cache = MovieCache()

    # ---- 尺寸與 letterbox 計算 ----
    def resizeEvent(self, _):
        self._layout_stage_16_9()
        # 視窗改變時，依目前 spec 重新套用縮放與定位
        self._apply_layer_geometry(self.bg_label, self._bg_spec, self._bg_movie)
        self._apply_layer_geometry(self.mouth_label, self._mouth_spec, self._mouth_movie)

    def _layout_stage_16_9(self):
        win_w, win_h = self.width(), self.height()
        target_w = win_w
        target_h = int(win_w * self.aspect_h / self.aspect_w)
        if target_h > win_h:
            target_h = win_h
            target_w = int(win_h * self.aspect_w / self.aspect_h)
        x, y = (win_w - target_w) // 2, (win_h - target_h) // 2
        self.stage.setGeometry(QRect(x, y, target_w, target_h))
        # 預設讓兩個 label 先佔滿舞台；具體大小由 _apply_layer_geometry 控制
        self.bg_label.setGeometry(0, 0, target_w, target_h)
        self.mouth_label.setGeometry(0, 0, target_w, target_h)

    # ---- 圖層控制 API ----
    def set_background(self, spec: LayerSpec | None):
        # 停止並清掉舊動畫
        if self._bg_movie:
            self._bg_movie.stop()
            self.bg_label.clear()
            self._bg_movie = None
        self._bg_spec = spec
        if spec is None or spec.path is None:
            return
        if not spec.path.exists():
            # 若檔案不存在，忽略（保持空白）
            return
        mv = self._movie_cache.get(spec.path)
        mv.stop()
        mv.setSpeed(int(max(0.1, float(spec.speed)) * 100))
        self.bg_label.setMovie(mv)
        self._bg_movie = mv
        self._apply_layer_geometry(self.bg_label, spec, mv)
        mv.start()

    def set_mouth(self, spec: LayerSpec | None, play: bool):
        # 停止並清掉舊動畫
        if self._mouth_movie:
            self._mouth_movie.stop()
            self.mouth_label.clear()
            self._mouth_movie = None
        self._mouth_spec = spec
        if not play or spec is None or spec.path is None:
            return
        if not spec.path.exists():
            return
        mv = self._movie_cache.get(spec.path)
        mv.stop()
        mv.setSpeed(int(max(0.1, float(spec.speed)) * 100))
        self.mouth_label.setMovie(mv)
        self._mouth_movie = mv
        self._apply_layer_geometry(self.mouth_label, spec, mv)
        mv.start()

    # ---- 幾何與縮放 ----
    def _apply_layer_geometry(self, label: QLabel, spec: Optional[LayerSpec], mv: Optional[QMovie]):
        if spec is None or mv is None:
            return
        st = self.stage.geometry()
        if spec.size:
            w, h = spec.size
            mv.setScaledSize(QSize(w, h))
            x = (st.width() - w) // 2 + spec.offset[0]
            y = (st.height() - h) // 2 + spec.offset[1]
            label.setGeometry(x, y, w, h)
        else:
            mv.setScaledSize(QSize(st.width(), st.height()))
            label.setGeometry(0, 0, st.width(), st.height())


class MainWindow(QWidget):
    """外層視窗：黑底 + 內嵌 16:9 舞台；提供 Esc/F11 快捷鍵。"""
    def __init__(self):
        super().__init__()
        pal = self.palette()
        pal.setColor(QPalette.Window, QColor(0, 0, 0))
        self.setAutoFillBackground(True)
        self.setPalette(pal)
        self.stage = Stage()
        self.stage.setParent(self)
        self.showFullScreen()

    def resizeEvent(self, _):
        self.stage.setGeometry(0, 0, self.width(), self.height())

    def keyPressEvent(self, e: QKeyEvent):
        if e.key() == Qt.Key_Escape:
            QApplication.quit()
        elif e.key() == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()


class RobotFaceNode(Node):
    """ROS2 節點：訂閱 face/animation 與 face/speaking，驅動兩個圖層。"""
    def __init__(self, ui: MainWindow):
        super().__init__("robot_face_node")
        self.ui = ui

        # 初始化主動畫為 DEFAULT_ANIM
        self._apply_animation(DEFAULT_ANIM)

        # 嘴巴圖層 spec（固定）
        self._mouth_spec = LayerSpec(
            path=Path(MOUTH_GIF_PATH),
            speed=MOUTH_SPEED,
            size=MOUTH_SIZE,
            offset=MOUTH_OFFSET
        )
        # 初始關閉嘴巴
        self.ui.stage.set_mouth(self._mouth_spec, play=False)

        # ROS2 訂閱
        self.create_subscription(RosString, "face/animation", self._on_anim, 10)
        self.create_subscription(RosBool,   "face/speaking",  self._on_speaking, 10)

    # ---- 回呼 ----
    def _on_anim(self, msg: RosString):
        name = msg.data.strip()
        self._apply_animation(name if name in ANIM_MAP else DEFAULT_ANIM)

    def _on_speaking(self, msg: RosBool):
        self.ui.stage.set_mouth(self._mouth_spec, play=bool(msg.data))

    # ---- 動畫套用 ----
    def _apply_animation(self, name: str):
        path, speed, size, offset = ANIM_MAP.get(name, ANIM_MAP[DEFAULT_ANIM])
        spec = None if path is None else LayerSpec(Path(path), speed, size, offset)
        self.ui.stage.set_background(spec)


def main(argv=None):
    rclpy.init(args=argv)
    app = QApplication(sys.argv)

    win = MainWindow()

    node = RobotFaceNode(win)

    # 以 Qt 計時器驅動非阻塞 spin
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(10)

    app.exec()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
