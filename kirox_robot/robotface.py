#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_face_node.py — RobotFace (Qt-only, ROS2-controlled, touch-first, horizontal icons)
"""

from __future__ import annotations
import os
import sys
import signal
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Dict, Tuple, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString, Bool as RosBool

from PySide6.QtCore import Qt, QRect, QSize, QTimer
from PySide6.QtGui import QMovie, QColor, QPalette, QKeyEvent, QFont, QIcon
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QToolButton, QFrame, QVBoxLayout, QHBoxLayout,
    QMessageBox, QGraphicsDropShadowEffect, QSizePolicy, QGridLayout
)

# ---------- 資源 ----------
ROOT_DIR = Path(__file__).resolve().parent
ASSETS = ROOT_DIR / "face_ui" / "assets"

ICONS = {
    "back":   ASSETS / "back.png",
    "reload": ASSETS / "reload.png",
    "wifi":   ASSETS / "wifi.png",
    "dev":    ASSETS / "dev.png",
    "gear":   ASSETS / "gear.png",
}

# ---------- 動畫可調 ----------
ANIM_MAP: Dict[str, Tuple[Optional[Path], float, Optional[Tuple[int,int]], Tuple[int,int]]] = {
    "empty":   (None,              1.0, None,        (0, 0)),
    "loading": (ASSETS/"Todo.gif", 0.8, (600, 600),  (0, 0)),
    "idle":    (ASSETS/"Todo.gif", 1.0, None,        (0, 0)),
    "happy":   (ASSETS/"Todo.gif", 1.2, (420, 420),  (0,-40)),
    "angry":   (ASSETS/"Todo.gif", 0.9, (420, 420),  (0, 40)),
    "talk":    (ASSETS/"Todo.gif", 1.0, None,        (0, 0)),
}
DEFAULT_ANIM = "empty"

MOUTH_GIF_PATH = ASSETS / "Todo.gif"
MOUTH_SPEED    = 1.0
MOUTH_SIZE     = (380, 160)
MOUTH_OFFSET   = (0, 180)


@dataclass
class LayerSpec:
    path: Optional[Path]
    speed: float
    size: Optional[Tuple[int, int]]
    offset: Tuple[int, int]


class MovieCache:
    def __init__(self):
        self._cache: Dict[Path, QMovie] = {}
    def get(self, path: Path) -> QMovie:
        mv = self._cache.get(path)
        if mv is None:
            mv = QMovie(str(path))
            self._cache[path] = mv
        return mv


class Stage(QWidget):
    """16:9 舞台 with letterbox；背景 + 嘴巴兩層"""
    def __init__(self, aspect_w=16, aspect_h=9):
        super().__init__()
        self.aspect_w, self.aspect_h = aspect_w, aspect_h

        pal = self.palette()
        pal.setColor(QPalette.Window, QColor(0, 0, 0))
        self.setAutoFillBackground(True)
        self.setPalette(pal)

        self.stage = QWidget(self)
        self.stage.setAutoFillBackground(True)
        sp = self.stage.palette()
        sp.setColor(QPalette.Window, QColor(0,0,0))
        self.stage.setPalette(sp)

        self.bg_label = QLabel(self.stage)
        self.bg_label.setAlignment(Qt.AlignCenter)
        self.bg_label.setStyleSheet("background: transparent;")

        self.mouth_label = QLabel(self.stage)
        self.mouth_label.setAlignment(Qt.AlignCenter)
        self.mouth_label.setStyleSheet("background: transparent;")

        self.bg_label.lower()
        self.mouth_label.raise_()

        self._movie_cache = MovieCache()
        self._bg_spec: Optional[LayerSpec] = None
        self._mouth_spec: Optional[LayerSpec] = None
        self._bg_movie: Optional[QMovie] = None
        self._mouth_movie: Optional[QMovie] = None

    def resizeEvent(self, _):
        self._layout_16_9()
        self._apply(self.bg_label, self._bg_spec, self._bg_movie)
        self._apply(self.mouth_label, self._mouth_spec, self._mouth_movie)

    def _layout_16_9(self):
        ww, wh = self.width(), self.height()
        tw = ww
        th = int(ww * self.aspect_h / self.aspect_w)
        if th > wh:
            th = wh
            tw = int(wh * self.aspect_w / self.aspect_h)
        x, y = (ww - tw)//2, (wh - th)//2
        self.stage.setGeometry(QRect(x, y, tw, th))
        self.bg_label.setGeometry(0, 0, tw, th)
        self.mouth_label.setGeometry(0, 0, tw, th)

    def set_background(self, spec: Optional[LayerSpec]):
        if self._bg_movie:
            self._bg_movie.stop()
            self.bg_label.clear()
            self._bg_movie = None
        self._bg_spec = spec
        if not spec or not spec.path or not spec.path.exists():
            return
        mv = self._movie_cache.get(spec.path)
        mv.stop()
        mv.setSpeed(int(max(0.1, float(spec.speed)) * 100))
        self.bg_label.setMovie(mv)
        self._bg_movie = mv
        self._apply(self.bg_label, spec, mv)
        mv.start()

    def set_mouth(self, spec: Optional[LayerSpec], play: bool):
        if self._mouth_movie:
            self._mouth_movie.stop()
            self.mouth_label.clear()
            self._mouth_movie = None
        self._mouth_spec = spec
        if not play or not spec or not spec.path or not spec.path.exists():
            return
        mv = self._movie_cache.get(spec.path)
        mv.stop()
        mv.setSpeed(int(max(0.1, float(spec.speed)) * 100))
        self.mouth_label.setMovie(mv)
        self._mouth_movie = mv
        self._apply(self.mouth_label, spec, mv)
        mv.start()

    def _apply(self, label: QLabel, spec: Optional[LayerSpec], mv: Optional[QMovie]):
        if not spec or not mv:
            return
        st = self.stage.geometry()
        if spec.size:
            w, h = spec.size
            mv.setScaledSize(QSize(w, h))
            x = (st.width() - w)//2 + spec.offset[0]
            y = (st.height() - h)//2 + spec.offset[1]
            label.setGeometry(x, y, w, h)
        else:
            mv.setScaledSize(QSize(st.width(), st.height()))
            label.setGeometry(0, 0, st.width(), st.height())


# ------------------------ 設定疊層（自適應網格 / 上圖下字 / 無標題） ------------------------
class SettingsOverlay(QWidget):
    """
    暗幕 + 玻璃卡片；自適應網格的 4 顆圓角方形按鈕（上方 icon、下方名稱）。
    會依視窗寬度在 4/3/2/1 欄之間切換，確保小螢幕不擠。
    """
    def __init__(self, on_back, on_reload, on_network, on_dev_exit, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: rgba(10, 10, 12, 170);")
        self.setVisible(False)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(24, 24, 24, 24)
        outer.setSpacing(0)
        outer.setAlignment(Qt.AlignCenter)

        # 卡片容器
        self.card = QFrame(self)
        self.card.setObjectName("card")
        self.card.setStyleSheet("""
            QFrame#card {
                background: rgba(255,255,255,0.92);
                border: 1px solid rgba(255,255,255,0.70);
                border-radius: 24px;
            }
        """)
        shadow = QGraphicsDropShadowEffect(self.card)
        shadow.setBlurRadius(48)
        shadow.setOffset(0, 16)
        shadow.setColor(QColor(0, 0, 0, 120))
        self.card.setGraphicsEffect(shadow)

        self.card_layout = QVBoxLayout(self.card)
        self.card_layout.setContentsMargins(24, 20, 24, 20)
        self.card_layout.setSpacing(0)

        self.grid = QGridLayout()
        self.grid.setContentsMargins(8, 8, 8, 8)
        self.grid.setHorizontalSpacing(18)
        self.grid.setVerticalSpacing(18)
        self.card_layout.addLayout(self.grid)
        outer.addWidget(self.card)

        # 建立四顆按鈕（統一樣式）
        self.buttons: List[QToolButton] = []
        def build_tool(text: str, icon_path: Path, cb) -> QToolButton:
            btn = QToolButton(self.card)
            btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            if icon_path.exists():
                btn.setIcon(QIcon(str(icon_path)))
                btn.setText(text)
            else:
                emojis = {"返回":"⬅️","重載配置":"🔄","網路設定":"📶","開發者模式":"🧰"}
                btn.setText(f"{emojis.get(text,'🔘')}\n{text}")
            btn.clicked.connect(cb)
            btn.setStyleSheet("""
                QToolButton {
                    background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                        stop:0 rgba(250,250,252,1),
                        stop:1 rgba(236,238,242,1));
                    color: #0e0e0f;
                    border: 1px solid rgba(0,0,0,0.08);
                    border-radius: 18px;
                    font-size: 18px;
                    font-weight: 600;
                    padding: 10px 10px 12px 10px;
                }
                QToolButton:hover { background: rgba(255,255,255,1.0); border: 1px solid rgba(0,0,0,0.18); }
                QToolButton:pressed { background: rgba(243,244,247,1.0); border: 1px solid rgba(0,0,0,0.22); }
            """)
            eff = QGraphicsDropShadowEffect(btn)
            eff.setBlurRadius(24)
            eff.setOffset(0, 10)
            eff.setColor(QColor(0, 0, 0, 70))
            btn.setGraphicsEffect(eff)
            self.buttons.append(btn)
            return btn

        self.btn_back    = build_tool("返回",     ICONS["back"],   on_back)
        self.btn_reload  = build_tool("重載配置", ICONS["reload"], on_reload)
        self.btn_network = build_tool("網路設定", ICONS["wifi"],   on_network)
        self.btn_dev     = build_tool("開發者模式", ICONS["dev"],    on_dev_exit)

        # 先擺一次，後續在 resizeEvent 自適應重排
        self._relayout()

    def show_overlay(self):
        self.setVisible(True)
        self.raise_()
        self._relayout()

    def hide_overlay(self):
        self.setVisible(False)

    def resizeEvent(self, _):
        self._relayout()

    def _relayout(self):
        # 清空 grid
        while self.grid.count():
            item = self.grid.takeAt(0)
            w = item.widget()
            if w:
                self.grid.removeWidget(w)

        # 依寬度決定欄數與按鈕大小
        w = max(1, self.width())
        # 以 240px 為單鍵基準寬，留出左右內距與間距
        col = max(1, min(4, int((w - 48) // 240)))
        if col == 0: col = 1

        # icon 與最小尺寸隨 DPI/寬度調整
        icon_px = 120
        min_w = 200
        min_h = 220
        if w < 1100:
            icon_px = 96
            min_w = 180
            min_h = 200
        if w < 800:
            icon_px = 84
            min_w = 160
            min_h = 180
        if w < 640:
            icon_px = 72
            min_w = 148
            min_h = 168

        for b in self.buttons:
            b.setIconSize(QSize(icon_px, icon_px))
            b.setMinimumSize(min_w, min_h)

        # 逐行逐列擺放
        for i, b in enumerate(self.buttons):
            r, c = divmod(i, col)
            self.grid.addWidget(b, r, c)


class SettingsButton(QToolButton):
    """右上角齒輪（QToolButton）"""
    def __init__(self, parent: QWidget, on_click):
        super().__init__(parent)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)
        self.setFixedSize(68, 68)
        self.setIconSize(QSize(36, 36))
        if ICONS["gear"].exists():
            self.setIcon(QIcon(str(ICONS["gear"])))
            self.setText("")
        else:
            self.setText("⚙")
        self.clicked.connect(on_click)
        self.setStyleSheet("""
            QToolButton {
                background-color: rgba(255, 255, 255, 180);
                color: #0e0e0f;
                border-radius: 34px;
                border: 1px solid rgba(0,0,0,0.08);
                font-size: 30px;
            }
            QToolButton:hover { background-color: rgba(255,255,255,220); }
            QToolButton:pressed { background-color: rgba(240,240,240,220); }
        """)

    def update_position(self):
        p = self.parent()
        if p:
            x = p.width() - self.width() - 18
            y = 18
            self.move(x, y)

    def showEvent(self, e):
        self.update_position()
        super().showEvent(e)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        pal = self.palette()
        pal.setColor(QPalette.Window, QColor(0, 0, 0))
        self.setPalette(pal)
        self.setAutoFillBackground(True)

        self.stage = Stage()
        self.stage.setParent(self)

        self.btn = SettingsButton(self, on_click=lambda: None)
        self.btn.raise_()
        self.btn.show()

        self.overlay = SettingsOverlay(
            on_back      = lambda: None,
            on_reload    = lambda: None,
            on_network   = lambda: None,
            on_dev_exit  = lambda: None,
            parent=self
        )
        self.overlay.setGeometry(0, 0, self.width(), self.height())
        self.overlay.lower()
        self.showFullScreen()

    def resizeEvent(self, _):
        self.stage.setGeometry(0, 0, self.width(), self.height())
        self.btn.update_position()
        self.overlay.setGeometry(0, 0, self.width(), self.height())

    def keyPressEvent(self, e: QKeyEvent):
        if e.key() == Qt.Key_Escape:
            QApplication.quit()
        elif e.key() == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()


# ------------------------ 主節點 ------------------------
class RobotFaceNode(Node):
    def __init__(self, ui: MainWindow):
        super().__init__("robot_face_node")
        self.ui = ui

        # 嘴巴 spec
        self._mouth_spec = LayerSpec(MOUTH_GIF_PATH, MOUTH_SPEED, MOUTH_SIZE, MOUTH_OFFSET)
        self.ui.stage.set_mouth(self._mouth_spec, play=False)

        # 初始動畫
        self._apply_animation(DEFAULT_ANIM)

        # ROS I/O
        self.create_subscription(RosString, "face/animation", self._on_anim, 10)
        self.create_subscription(RosBool,   "face/speaking",  self._on_speaking, 10)

        self.setting_mode_pub = self.create_publisher(RosBool, "system/setting_mode", 10)
        self.reload_pub       = self.create_publisher(RosBool, "system/reload_config", 10)

        # 顯/隱 overlay
        self._local_setting_mode = False
        self.ui.btn.clicked.connect(lambda: self._toggle_settings(True))

        # 非阻塞 ROS spin
        self._qt_timer = QTimer()
        self._qt_timer.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0.0))
        self._qt_timer.start(10)

    # ---- 動畫/說話 ----
    def _on_anim(self, msg: RosString):
        name = msg.data.strip()
        self._apply_animation(name if name in ANIM_MAP else DEFAULT_ANIM)

    def _on_speaking(self, msg: RosBool):
        self.ui.stage.set_mouth(self._mouth_spec, play=bool(msg.data))

    def _apply_animation(self, name: str):
        path, speed, size, offset = ANIM_MAP.get(name, ANIM_MAP[DEFAULT_ANIM])
        spec = None if path is None else LayerSpec(path, speed, size, offset)
        if spec and spec.path and not spec.path.exists():
            self.get_logger().warn(f"[RobotFace] 動畫檔不存在：{spec.path}")
            spec = None
        self.ui.stage.set_background(spec)

    # ---- 設定疊層 ----
    def _toggle_settings(self, show: bool):
        if show:
            if not self._local_setting_mode:
                self.setting_mode_pub.publish(RosBool(data=True))
                self._local_setting_mode = True
            self._connect_overlay_buttons()
            self.ui.overlay.show_overlay()
        else:
            if self._local_setting_mode:
                self.setting_mode_pub.publish(RosBool(data=False))
                self._local_setting_mode = False
            self.ui.overlay.hide_overlay()

    def _connect_overlay_buttons(self):
        # 先斷開舊連線避免重複觸發
        for btn, slot in [
            (self.ui.overlay.btn_back,    self._action_back),
            (self.ui.overlay.btn_reload,  self._action_reload),
            (self.ui.overlay.btn_network, self._action_network),
            (self.ui.overlay.btn_dev,     self._action_dev_exit),
        ]:
            try:
                btn.clicked.disconnect()
            except Exception:
                pass
            btn.clicked.connect(slot)

    # 四顆按鈕的動作
    def _action_back(self):
        self._toggle_settings(False)

    def _action_reload(self):
        self.reload_pub.publish(RosBool(data=True))
        QMessageBox.information(self.ui, "重載設定", "已發送重載指令。設定將於下一回合生效。")
        self._toggle_settings(False)

    def _action_network(self):
        QMessageBox.information(self.ui, "網路設定", "網路設定功能尚未實作。")

    def _action_dev_exit(self):
        self.get_logger().warn("[DevMode] 將結束所有 ROS2 相關進程並退出")
        self._kill_ros2_launch_processes()
        QApplication.quit()

    # ---- 系統層操作：結束 ros2 進程（先 TERM 再 KILL；避開自身） ----
    def _kill_ros2_launch_processes(self):
        my_pid = os.getpid()
        patterns = [
            r"ros2 launch",
            r"ros2 run",
            r"launch\.py",
            r"kirox_robot",
            r"robotears", r"roboteyes", r"robotbrain", r"robotbody", r"robotface"
        ]

        # 1) 溫和結束：TERM
        for pat in patterns:
            try:
                subprocess.run(
                    ["bash", "-lc", f"pgrep -f \"{pat}\" | grep -v -w {my_pid} | xargs -r -n1 kill -TERM"],
                    check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
            except Exception:
                pass

        # 小等一下
        try:
            subprocess.run(["bash", "-lc", "sleep 0.7"], check=False)
        except Exception:
            pass

        # 2) 二次清理：尚存者再 TERM 一次（防殭屍）
        for pat in patterns:
            try:
                subprocess.run(
                    ["bash", "-lc", f"pgrep -f \"{pat}\" | grep -v -w {my_pid} | xargs -r -n1 kill -TERM"],
                    check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
            except Exception:
                pass

        try:
            subprocess.run(["bash", "-lc", "sleep 0.5"], check=False)
        except Exception:
            pass

        # 3) 強制：KILL
        for pat in patterns:
            try:
                subprocess.run(
                    ["bash", "-lc", f"pgrep -f \"{pat}\" | grep -v -w {my_pid} | xargs -r -n1 kill -KILL"],
                    check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
            except Exception:
                pass

        # 4) 預防殘存的 ros2/launch 殼層
        final_cmds = [
            "pkill -f '^python3 .*ros2.*launch' || true",
            "pkill -f '^/usr/bin/python3 .*launch.*\\.py' || true",
            "pkill -f 'ros2 launch' || true",
            "pkill -f 'ros2 run' || true",
        ]
        for c in final_cmds:
            try:
                subprocess.run(["bash", "-lc", c], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass


def main(argv=None):
    # 觸控合成，確保觸控能點
    QApplication.setAttribute(Qt.AA_SynthesizeTouchForUnhandledMouseEvents, True)
    QApplication.setAttribute(Qt.AA_SynthesizeMouseForUnhandledTouchEvents, True)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)

    rclpy.init(args=argv)
    app = QApplication(sys.argv)
    app.setFont(QFont("SF Pro Text", 12))

    win = MainWindow()
    node = RobotFaceNode(win)

    def _cleanup():
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    app.aboutToQuit.connect(_cleanup)

    app.exec()


if __name__ == "__main__":
    main()
