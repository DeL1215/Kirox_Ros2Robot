#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_face_node.py — RobotFace (Qt-only, ROS2-controlled, touch-first)

要點：
- run_cmd_async：非阻塞執行 nmcli，並完整 print log（指令、exit、stdout/stderr）。
- NetworkOverlay：
  * 取裝置：不再使用 awk，改在 Python 解析 nmcli dev 輸出（排除 p2p）。
  * 掃描流程：先 rescan，再延遲 list；必要時第二次 list 合併（避免只剩目前連線）。
  * Debug 區：顯示指令與摘要（裝置、AP 數量、前幾個 SSID）。
  * Loading 常駐，只 show/hide，不 delete。
  * 解析列表用 split(":", 3)，容忍隱藏 SSID；connected 轉成 bool，避免 setEnabled(str)。
- 移除不支援的 CSS filter。
- 其餘 UI/ROS2 維持前版行為。
"""

from __future__ import annotations
import os
import sys
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Dict, Tuple, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString, Bool as RosBool

from PySide6.QtCore import Qt, QRect, QSize, QTimer, QThread, Signal, QObject
from PySide6.QtGui import QMovie, QColor, QPalette, QKeyEvent, QFont, QIcon, QPainter
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QToolButton, QFrame, QVBoxLayout, QHBoxLayout,
    QGraphicsDropShadowEffect, QSizePolicy, QScrollArea, QPushButton, QLineEdit,
    QGridLayout, QSpacerItem, QDialog, QScroller 
)


# ---------- 資源 ----------
ROOT_DIR = Path(__file__).resolve().parent
ASSETS = ROOT_DIR / "assets" / "face_ui"
ASSETS_ICON = ROOT_DIR / "assets" / "icon"

ICONS = {
    "back":   ASSETS_ICON / "back.png",
    "reload": ASSETS_ICON / "reload.png",
    "wifi":   ASSETS_ICON / "wifi.png",
    "dev":    ASSETS_ICON / "dev.png",
    "gear":   ASSETS_ICON / "gear.png",
    "noconnection": ASSETS_ICON / "No_connection.png",
    "signal0": ASSETS_ICON / "signal-0.png",
    "signal1": ASSETS_ICON / "signal-1.png",
    "signal2": ASSETS_ICON / "signal-2.png",
    "signal3": ASSETS_ICON / "signal-3.png",
    "lock":    ASSETS_ICON / "lock.png",
    "unlock":  ASSETS_ICON / "unlock.png",
}

# ---------- 動畫可調 ----------
ANIM_MAP: Dict[str, Tuple[Optional[Path], float, Optional[Tuple[int,int]], Tuple[int,int]]] = {
    "empty":   (None,                 1.0, None,        (0, 0)),
    "loading": (ASSETS/"EmojiThinking.gif", 0.9, (300, 300),  (0, 0)),
    "loading2": (ASSETS/"Loading2.gif", 0.9, (40, 40),  (0, 0)),
    "idle":    (ASSETS/"Todo.gif",    1.0, None,        (0, 0)),
    "happy":   (ASSETS/"Todo.gif",    1.2, (420, 420),  (0,-40)),
    "angry":   (ASSETS/"Todo.gif",    0.9, (420, 420),  (0, 40)),
    "talk":    (ASSETS/"Todo.gif",    1.0, None,        (0, 0)),
}
DEFAULT_ANIM = "empty"

MOUTH_GIF_PATH = ASSETS / "Todo.gif"
MOUTH_SPEED    = 1.0
MOUTH_SIZE     = (380, 160)
MOUTH_OFFSET   = (0, 180)


SHOW_NET_DEBUG = False  # ← 量產/使用者模式請關閉


@dataclass
class LayerSpec:
    path: Optional[Path]
    speed: float
    size: Optional[Tuple[int, int]]
    offset: Tuple[int, int]


# ======================== 玻璃 Dialog / Toast ========================
class GlassDialog(QDialog):
    def __init__(self, parent=None, title=""):
        super().__init__(parent)
        self.setModal(True)
        self.setWindowFlag(Qt.FramelessWindowHint, True)
        self.setAttribute(Qt.WA_TranslucentBackground, True)

        base = QFrame(self)
        base.setObjectName("glass")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(base)

        v = QVBoxLayout(base)
        v.setContentsMargins(24, 24, 24, 24)
        v.setSpacing(16)

        if title:
            lbl = QLabel(title)
            lbl.setStyleSheet("background:transparent; color:#0e0e0f; font-size:22px; font-weight:700;")
            v.addWidget(lbl)

        self.content_layout = QVBoxLayout()
        self.content_layout.setSpacing(12)
        v.addLayout(self.content_layout)

        btns = QHBoxLayout()
        btns.addStretch(1)
        self.btn_cancel = QPushButton("取消")
        self.btn_ok = QPushButton("確定")
        for b in (self.btn_cancel, self.btn_ok):
            b.setCursor(Qt.PointingHandCursor)
            b.setFixedHeight(42)
            b.setStyleSheet("""
                QPushButton {
                    background: rgba(255,255,255,0.96);
                    border:1px solid rgba(0,0,0,0.08);
                    border-radius:12px;
                    padding:8px 16px;
                    font-weight:600;
                }
                QPushButton:hover { border:1px solid rgba(0,0,0,0.18); }
                QPushButton:pressed { background: rgba(240,240,242,0.96); }
            """)
        btns.addWidget(self.btn_cancel)
        btns.addWidget(self.btn_ok)
        v.addLayout(btns)

        base.setStyleSheet("""
            QFrame#glass {
                background: rgba(255,255,255,0.90);
                border: 1px solid rgba(255,255,255,0.65);
                border-radius: 24px;
            }
        """)
        eff = QGraphicsDropShadowEffect(base)
        eff.setBlurRadius(48)
        eff.setOffset(0, 16)
        eff.setColor(QColor(0, 0, 0, 120))
        base.setGraphicsEffect(eff)

        self.resize(520, 280)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)


class GlassToast(QWidget):
    def __init__(self, parent: QWidget, text: str, ms=1800):
        super().__init__(parent)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("""
            QWidget {
                background: rgba(30,30,32,0.9);
                color: #fff;
                border-radius: 14px;
            }
        """)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(16, 12, 16, 12)
        lbl = QLabel(text)
        lbl.setStyleSheet("background:transparent; color:#fff; font-weight:600;")
        lay.addWidget(lbl)
        self.adjustSize()
        self.show()
        self._place()
        QTimer.singleShot(ms, self.close)

    def _place(self):
        if not self.parent():
            return
        p = self.parent()
        x = p.width() - self.width() - 24
        y = 24
        self.setGeometry(x, y, self.width(), self.height())

    def resizeEvent(self, _):
        self._place()


# ======================== 後台命令（完整列印 log） ========================
class CmdWorker(QObject):
    finished = Signal(int, str, str)  # exit_code, stdout, stderr

    def __init__(self, cmd: str, timeout: float = 8.0, env: Optional[dict] = None):
        super().__init__()
        self.cmd = cmd
        self.timeout = timeout
        self.env = env or {}
        self.env.setdefault("LANG", "C")
        self.env.setdefault("LC_ALL", "C")
        self.env.setdefault("TERM", "dumb")
        self.env.setdefault("PATH", os.environ.get("PATH", "/usr/sbin:/usr/bin:/sbin:/bin"))

    def run(self):
        print(f"[NET] exec: {self.cmd}")
        try:
            proc = subprocess.run(
                ["bash", "-lc", self.cmd],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                check=False, text=True, timeout=self.timeout, env={**os.environ, **self.env}
            )
            print(f"[NET] exit={proc.returncode}")
            print("[NET] stdout:"); print(proc.stdout)
            print("[NET] stderr:"); print(proc.stderr)
            self.finished.emit(proc.returncode, proc.stdout.strip(), proc.stderr.strip())
        except subprocess.TimeoutExpired as e:
            print(f"[NET] timeout after {self.timeout}s")
            so = (e.stdout.decode("utf-8", "ignore").strip() if isinstance(e.stdout, (bytes, bytearray)) else (e.stdout or "")).strip()
            se = f"timeout after {self.timeout}s: {e}"
            self.finished.emit(124, so, se)
        except Exception as e:
            self.finished.emit(1, "", str(e))

            
def run_cmd_async(parent: QWidget, cmd: str, callback, timeout: float = 8.0):
    """
    在背景執行 shell 指令；結果回到 parent 所在的主執行緒呼叫 callback。
    - Worker 只做 run(); 不觸碰 UI。
    - 透過一個駐留在 parent thread 的 invoker.Signal，把結果排入主執行緒。
    - 結束後安全地關閉/回收 QThread。
    """
    from PySide6.QtCore import QObject, Signal, QTimer, QThread

    # 建立工作緒與 worker（不把 thread 掛在 QWidget 底下，避免 thread affinity 混淆）
    th = QThread()
    th.setObjectName("CmdThread")
    worker = CmdWorker(cmd, timeout=timeout)
    worker.moveToThread(th)

    # 建立一個駐留在「parent 所在執行緒」的轉發器，負責把結果丟回主執行緒
    class _Invoker(QObject):
        sig = Signal(int, str, str)  # (exit_code, stdout, stderr)

    invoker = _Invoker(parent)  # parent=UI 物件 → 這個 invoker 會住在 UI thread

    def _cleanup():
        # 在主執行緒上收尾（避免在 worker thread 上關 thread）
        try:
            th.quit()
            th.wait()
        except Exception:
            pass
        finally:
            th.deleteLater()

    def _on_main_thread(ec: int, so: str, se: str):
        # 一定在 UI thread 執行
        try:
            callback(ec, so, se)
        finally:
            # 延後一個事件循環再做釋放，保證 callback 內如有再開新任務也不會撞到
            QTimer.singleShot(0, _cleanup)

    # invoker 在主執行緒；把它連到真正要跑的 callback
    invoker.sig.connect(_on_main_thread)

    # worker 完成時（在 worker thread 發出 finished），改由 invoker 在主執行緒 emit
    worker.finished.connect(lambda ec, so, se: invoker.sig.emit(ec, so, se))

    # 啟動
    def _start():
        print(f"[NET] exec: {cmd}")
        worker.run()

    print(f"[NET] $ {cmd}")
    th.started.connect(_start)
    th.start()


def nmcli_quote(s: str) -> str:
    s = (s or "").replace('"', '\\"')
    return f'"{s}"'


# ======================== 觸控小鍵盤 ========================
class VirtualKeyboardDialog(GlassDialog):
    def __init__(self, parent=None, title="輸入密碼"):
        super().__init__(parent, title)
        wrap = QVBoxLayout()
        wrap.setSpacing(10)

        self.edit = QLineEdit()
        self.edit.setEchoMode(QLineEdit.Password)
        self.edit.setFixedHeight(46)
        self.edit.setStyleSheet("""
            QLineEdit {
                background: rgba(255,255,255,0.98);
                border:1px solid rgba(0,0,0,0.12);
                border-radius: 12px;
                padding: 8px 12px;
                font-size: 18px;
            }
        """)
        wrap.addWidget(self.edit)

        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(8)

        rows = [
            list("1234567890"),
            list("qwertyuiop"),
            list("asdfghjkl"),
            list("zxcvbnm"),
        ]
        self._upper = False

        def add_key(r, c, ch, w=1):
            btn = QPushButton(ch)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setFixedHeight(44)
            btn.setMinimumWidth(44 * w + (w - 1) * 6)
            btn.setStyleSheet("""
                QPushButton {
                    background: rgba(255,255,255,0.96);
                    border:1px solid rgba(0,0,0,0.08);
                    border-radius:10px;
                    font-size:18px; font-weight:600;
                }
                QPushButton:hover { border:1px solid rgba(0,0,0,0.18); }
                QPushButton:pressed { background: rgba(240,240,242,0.96); }
            """)
            btn.clicked.connect(lambda: self._on_key(ch))
            grid.addWidget(btn, r, c, 1, w)

        for i, ch in enumerate(rows[0]):
            add_key(0, i, ch)
        for ri, line in enumerate(rows[1:], start=1):
            grid.addItem(QSpacerItem(16, 1), ri, 0)
            for i, ch in enumerate(line):
                add_key(ri, i + 1, ch)

        ctrl_row = 5
        self.shift_btn = QPushButton("Shift")
        self.shift_btn.setFixedHeight(44)
        self.shift_btn.setCursor(Qt.PointingHandCursor)
        self.shift_btn.setStyleSheet(self._ctrl_style())
        self.shift_btn.clicked.connect(self._toggle_upper)
        grid.addWidget(self.shift_btn, ctrl_row, 0, 1, 2)

        def ctrl(text, cb, col, span=1):
            b = QPushButton(text)
            b.setCursor(Qt.PointingHandCursor)
            b.setFixedHeight(44)
            b.setStyleSheet(self._ctrl_style())
            b.clicked.connect(cb)
            grid.addWidget(b, ctrl_row, col, 1, span)

        ctrl("符號", self._symbols, 2, 2)
        ctrl("空白", lambda: self._on_key(" "), 4, 3)
        ctrl("退格", self._backspace, 7, 2)
        ctrl("清空", self._clear, 9, 2)
        ctrl("顯示/隱藏", self._toggle_echo, 11, 2)

        wrap.addLayout(grid)
        self.content_layout.addLayout(wrap)

        self.btn_cancel.clicked.connect(self.reject)
        self.btn_ok.clicked.connect(self.accept)

    def _ctrl_style(self):
        return """
            QPushButton {
                background: rgba(245,246,250,0.96);
                border:1px solid rgba(0,0,0,0.08);
                border-radius:10px;
                font-size:16px; font-weight:700;
            }
            QPushButton:hover { border:1px solid rgba(0,0,0,0.18); }
            QPushButton:pressed { background: rgba(235,236,240,0.96); }
        """

    def _on_key(self, ch: str):
        self.edit.insert(ch.upper() if self._upper and ch.isalpha() else ch)

    def _toggle_upper(self):
        self._upper = not self._upper
        self.shift_btn.setText("SHIFT ▲" if self._upper else "Shift")

    def _symbols(self):
        dlg = GlassDialog(self, "插入符號")
        grid = QGridLayout()
        sym = list("~`!@#$%^&*()-_=+[]{}\\|;:'\",.<>/?")
        row, col = 0, 0
        for s in sym:
            b = QPushButton(s)
            b.setCursor(Qt.PointingHandCursor)
            b.setFixedSize(44, 44)
            b.setStyleSheet(self._ctrl_style())
            b.clicked.connect(lambda _, ch=s: (self.edit.insert(ch), None))
            grid.addWidget(b, row, col)
            col += 1
            if col >= 10:
                col = 0
                row += 1
        dlg.content_layout.addLayout(grid)
        dlg.btn_ok.setText("完成")
        dlg.btn_cancel.setText("關閉")
        dlg.btn_ok.clicked.connect(dlg.accept)
        dlg.btn_cancel.clicked.connect(dlg.reject)
        dlg.exec()

    def _backspace(self):
        txt = self.edit.text()
        self.edit.setText(txt[:-1])

    def _clear(self):
        self.edit.clear()

    def _toggle_echo(self):
        if self.edit.echoMode() == QLineEdit.Password:
            self.edit.setEchoMode(QLineEdit.Normal)
        else:
            self.edit.setEchoMode(QLineEdit.Password)

    def get_text(self) -> str:
        return self.edit.text()


# ======================== 舞台 ========================
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


# ======================== 設定疊層 ========================
class SettingsOverlay(QWidget):
    TARGET_BTN_H = 210
    MIN_BTN_W = 160
    MAX_BTN_W = 360

    def __init__(self, on_back, on_reload, on_network, on_dev_exit, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: rgba(10, 10, 12, 170);")
        self.setVisible(False)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(24, 24, 24, 24)
        outer.setAlignment(Qt.AlignCenter)

        self.card = QFrame(self)
        self.card.setObjectName("card")
        self.card.setStyleSheet("""
            QFrame#card {
                background: rgba(255,255,255,0.96);
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
        self.card_layout.setContentsMargins(24, 24, 24, 24)
        self.card_layout.setAlignment(Qt.AlignCenter)

        self.hbox = QHBoxLayout()
        self.hbox.setSpacing(40)
        self.hbox.setAlignment(Qt.AlignLeft)
        self.card_layout.addLayout(self.hbox)
        outer.addWidget(self.card, alignment=Qt.AlignCenter)

        self.buttons: List[QToolButton] = []

        def build_tool(text: str, icon_path: Path, cb) -> QToolButton:
            btn = QToolButton(self.card)
            btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            if icon_path.exists():
                btn.setIcon(QIcon(str(icon_path)))
                btn.setText(text)
            else:
                emojis = {"返回": "⬅️", "重載設定": "🔄", "網路設定": "📶", "開發者模式": "🧰"}
                btn.setText(f"{emojis.get(text, '🔘')}\n{text}")
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
                    padding-top: 22px;
                    padding-bottom: 14px;
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

        self.btn_back    = build_tool("返回", ICONS["back"], on_back)
        self.btn_reload  = build_tool("重載設定", ICONS["reload"], on_reload)
        self.btn_network = build_tool("網路設定", ICONS["wifi"], on_network)
        self.btn_dev     = build_tool("開發者模式", ICONS["dev"], on_dev_exit)

        for b in self.buttons:
            self.hbox.addWidget(b)

        self._apply_sizes()

    def _dpi(self) -> float:
        screen = self.screen() or QApplication.primaryScreen()
        return max(0.75, min(2.0, screen.logicalDotsPerInch() / 96.0)) if screen else 1.0

    def _apply_sizes(self):
        cw = int(self.width() * 0.92)
        ch = int(self.height() * 0.80)
        dpi = self._dpi()

        self.card.setMinimumSize(int(720 * dpi), int(360 * dpi))
        self.card.setMaximumSize(cw, ch)
        self.card_layout.activate()

        inner_w = int(self.card.contentsRect().width())
        total_btns = 4
        gap = int(self.hbox.spacing() or 40)
        min_gap = 16

        min_w = int(self.MIN_BTN_W * dpi)
        max_w = int(self.MAX_BTN_W * dpi)

        usable_w = max(0, inner_w - (total_btns - 1) * gap)
        raw_w = usable_w // total_btns
        btn_w = max(min_w, min(max_w, raw_w))
        btn_h = int(self.TARGET_BTN_H * dpi)

        total_width = total_btns * btn_w + (total_btns - 1) * gap
        if total_width > inner_w:
            gap = max(min_gap, (inner_w - total_btns * btn_w) // (total_btns - 1))
            self.hbox.setSpacing(int(gap))
            total_width = total_btns * btn_w + (total_btns - 1) * gap
            if total_width > inner_w:
                btn_w = max(80, (inner_w - (total_btns - 1) * gap) // total_btns)
                total_width = total_btns * btn_w + (total_btns - 1) * gap

        icon_px = int(max(64 * dpi, min(160 * dpi, btn_w * 0.46))) & ~1
        for b in self.buttons:
            b.setFixedSize(int(btn_w), int(btn_h))
            b.setIconSize(QSize(icon_px, icon_px))

        remain = max(0, inner_w - total_width)
        left_margin  = remain // 2
        right_margin = remain - left_margin
        self.hbox.setContentsMargins(int(left_margin), 0, int(right_margin), 0)

    def show_overlay(self):
        self.setVisible(True)
        self.raise_()
        self._apply_sizes()

    def hide_overlay(self):
        self.setVisible(False)

    def resizeEvent(self, _):
        self._apply_sizes()


# ======================== 網路設定頁 ========================
class NetworkItem(QWidget):
    def __init__(self, ssid: str, security: str, signal: int, connected: bool,
                 on_connect, on_disconnect, parent=None):
        super().__init__(parent)
        self.ssid = ssid
        self.security = security
        self.signal = signal
        self.connected = bool(connected)

        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("""
            QWidget {
                background: rgba(255,255,255,0.98);
                border:1px solid rgba(0,0,0,0.08);
                border-radius:16px;
            }
            QWidget:hover { border:1px solid rgba(0,0,0,0.16); }
        """)
        h = QHBoxLayout(self)
        h.setContentsMargins(18, 16, 18, 16)   # ← 邊距放大
        h.setSpacing(12)

        sig_icon = self._signal_icon(signal)
        sig_label = QLabel()
        sig_label.setStyleSheet("background:transparent;")
        if sig_icon.exists():
            sig_label.setPixmap(QIcon(str(sig_icon)).pixmap(24, 24))
        else:
            sig_label.setText("📶")
            sig_label.setStyleSheet("background:transparent; font-size:20px;")
        h.addWidget(sig_label)

        v = QVBoxLayout()
        name = QLabel(ssid or "(隱藏 SSID)")
        name.setStyleSheet("background:transparent; font-weight:700; font-size:16px; color:#0e0e0f;")
        sub = QLabel(f"{'加密' if self._secured() else '開放'} · 訊號 {signal}%")
        sub.setStyleSheet("background:transparent; color:#555; font-size:12px;")
        v.addWidget(name)
        v.addWidget(sub)
        h.addLayout(v, 1)

        lock = QLabel()
        lock.setStyleSheet("background:transparent;")
        lock_icon = ICONS["lock"] if self._secured() else ICONS["unlock"]
        if lock_icon.exists():
            lock.setPixmap(QIcon(str(lock_icon)).pixmap(18, 18))
        else:
            lock.setText("🔒" if self._secured() else "🔓")
        h.addWidget(lock)

        self.btn = QPushButton("連線" if not self.connected else "已連線")
        self.btn.setCursor(Qt.PointingHandCursor)
        self.btn.setFixedHeight(44)  # ← 由 34 提升到 44
        self.btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                    stop:0 rgba(74, 144, 226, 1),
                    stop:1 rgba(52, 120, 212, 1));
                color: white; border:none; border-radius: 10px;
                font-weight:700; padding: 0 14px;
            }
            QPushButton:disabled { background: rgba(180,180,185,1); }
        """)
        h.addWidget(self.btn)

        self.btn_dis = QPushButton("斷線")
        self.btn_dis.setCursor(Qt.PointingHandCursor)
        self.btn_dis.setFixedHeight(44)  # ← 由 34 提升到 44
        self.btn_dis.setStyleSheet("""
            QPushButton {
                background: rgba(240,241,245,1);
                color: #0e0e0f; border:1px solid rgba(0,0,0,0.08);
                border-radius:10px; font-weight:700;
                padding: 0 14px;
            }
            QPushButton:hover { border:1px solid rgba(0,0,0,0.18); }
            QPushButton:pressed { background: rgba(230,231,235,1); }
        """)
        h.addWidget(self.btn_dis)

        self.btn.setEnabled(bool(not self.connected))
        self.btn_dis.setEnabled(bool(self.connected))

        self.btn.clicked.connect(lambda: on_connect(self))
        self.btn_dis.clicked.connect(lambda: on_disconnect(self))

    def _secured(self) -> bool:
        return bool(self.security and self.security.lower() not in ("--", "none"))

    def _signal_icon(self, sig: int) -> Path:
        if sig >= 75: return ICONS["signal3"]
        if sig >= 45: return ICONS["signal2"]
        if sig >= 15: return ICONS["signal1"]
        return ICONS["signal0"]

    def set_connected(self, ok: bool):
        ok = bool(ok)
        self.connected = ok
        self.btn.setEnabled(not ok)
        self.btn_dis.setEnabled(ok)
        self.btn.setText("已連線" if ok else "連線")


class NetworkOverlay(QWidget):
    """
    Wi-Fi 設定（無 awk 版）：所有 nmcli 輸出在 Python 內解析；強化掃描/逾時/診斷 log。
    不改 UI。
    """
    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: rgba(12, 12, 14, 180);")
        self.setVisible(False)

        self._current_dev = ""
        self._is_scanning = False
        self._tail_spacer: Optional[QSpacerItem] = None
        self._debug_last_cmds: List[str] = []

        outer = QVBoxLayout(self)
        outer.setContentsMargins(24, 24, 24, 24)
        outer.setSpacing(12)

        card = QFrame(self)
        card.setObjectName("netcard")
        card.setStyleSheet("""
            QFrame#netcard {
                background: rgba(255,255,255,0.96);
                border: 1px solid rgba(255,255,255,0.70);
                border-radius: 24px;
            }
        """)
        eff = QGraphicsDropShadowEffect(card)
        eff.setBlurRadius(48); eff.setOffset(0, 16); eff.setColor(QColor(0,0,0,120))
        card.setGraphicsEffect(eff)
        outer.addWidget(card, 1)

        v = QVBoxLayout(card)
        v.setContentsMargins(20, 16, 20, 20)
        v.setSpacing(10)

        top = QHBoxLayout()
        title = QLabel("網路設定")
        title.setStyleSheet("background:transparent; font-size:22px; font-weight:800; color:#0e0e0f;")
        top.addWidget(title)
        top.addStretch(1)

        self.btn_refresh = QPushButton("重新整理")
        self.btn_refresh.setCursor(Qt.PointingHandCursor)
        self.btn_refresh.setFixedHeight(34)
        self.btn_refresh.setStyleSheet("""
            QPushButton {
                background: rgba(245,246,250,1);
                border:1px solid rgba(0,0,0,0.08);
                border-radius:10px; font-weight:700; padding: 0 12px;
            }
            QPushButton:hover { border:1px solid rgba(0,0,0,0.16); }
            QPushButton:pressed { background: rgba(235,236,240,1); }
        """)
        self.btn_close = QPushButton("返回")
        self.btn_close.setCursor(Qt.PointingHandCursor)
        self.btn_close.setFixedHeight(34)
        self.btn_close.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                    stop:0 rgba(250,250,252,1),
                    stop:1 rgba(236,238,242,1));
                border:1px solid rgba(0,0,0,0.08); border-radius:10px;
                font-weight:700; padding: 0 12px;
            }
            QPushButton:hover { border:1px solid rgba(0,0,0,0.16); }
            QPushButton:pressed { background: rgba(243,244,247,1); }
        """)
        top.addWidget(self.btn_refresh)
        top.addWidget(self.btn_close)
        v.addLayout(top)

        self.lbl_status = QLabel("目前連線：讀取中…")
        self.lbl_status.setStyleSheet("background:transparent; color:#333; font-size:13px;")
        v.addWidget(self.lbl_status)

        self.lbl_debug = QLabel("")
        self.lbl_debug.setWordWrap(True)
        self.lbl_debug.setStyleSheet("background:transparent; color:#7a7a7a; font-size:12px;")
        if SHOW_NET_DEBUG:
            v.addWidget(self.lbl_debug)
        else:
            self.lbl_debug.hide()

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("""
            QScrollArea { border:none; background:transparent; }
            QScrollArea > QWidget > QWidget { background:transparent; }
            QScrollBar:vertical {
                width: 26px; background: transparent; margin: 6px 4px 6px 4px; border-radius: 12px;
            }
            QScrollBar::handle:vertical { min-height: 40px; border-radius: 12px; background: rgba(0,0,0,0.22); }
            QScrollBar::handle:vertical:hover { background: rgba(0,0,0,0.32); }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
        """)
        self.list_container = QWidget()
        self.list_container.setStyleSheet("background:transparent;")
        self.list_layout = QVBoxLayout(self.list_container)
        self.list_layout.setContentsMargins(4, 4, 4, 4)
        self.list_layout.setSpacing(8)
        scroll.setWidget(self.list_container)

        QScroller.grabGesture(scroll.viewport(), QScroller.TouchGesture)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)
        v.addWidget(scroll, 1)

        self.loading = QLabel(self.list_container)
        self.loading.setAlignment(Qt.AlignCenter)
        self.loading.setStyleSheet("background:transparent; color:#666;")
        self.loading_movie = QMovie(str(ANIM_MAP.get("loading2", (None,))[0])) if ANIM_MAP.get("loading2", (None,))[0] else None
        if self.loading_movie:
            self.loading.setMovie(self.loading_movie)
            self.loading_movie.setScaledSize(QSize(96, 96))

        tip = QLabel("提示：若未找到 Wi-Fi，請確認 NetworkManager（nmcli）啟用、rfkill 未封鎖，且 iface 有被管理。")
        tip.setWordWrap(True)
        tip.setStyleSheet("background:transparent; color:#666; font-size:12px;")
        v.addWidget(tip)

        self.btn_close.clicked.connect(self.hide_overlay)
        self.btn_refresh.clicked.connect(self.refresh)

        QTimer.singleShot(100, self.refresh)

    # ---------- log helpers ----------
    def _log(self, *args):
        if SHOW_NET_DEBUG:
            print("[robotface-UI][NET]", *args, flush=True)

    def _set_debug(self, text: str):
        if SHOW_NET_DEBUG:
            self.lbl_debug.setText(text or "")

    def _remember(self, cmd: str):
        if SHOW_NET_DEBUG:
            self._debug_last_cmds.append(cmd)

    @staticmethod
    def _split_nmcli(line: str) -> List[str]:
        out, buf, esc = [], [], False
        for ch in line:
            if esc:
                buf.append(ch); esc = False
            else:
                if ch == '\\': esc = True
                elif ch == ':': out.append(''.join(buf)); buf = []
                else: buf.append(ch)
        out.append(''.join(buf))
        return out

    # ---------- lifecycle ----------
    def show_overlay(self):
        self.setVisible(True)
        self.raise_()
        self.setGeometry(0, 0, self.parent().width(), self.parent().height())

    def hide_overlay(self):
        self.setVisible(False)

    def resizeEvent(self, _):
        self.setGeometry(0, 0, self.parent().width(), self.parent().height())

    # ---------- flow ----------
    def refresh(self):
        if self._is_scanning:
            return
        self._is_scanning = True
        self._debug_last_cmds.clear()
        self._set_debug("開始掃描…")
        self._clear_list()
        self._show_loading(True)
        self._toast("正在掃描 Wi-Fi…")
        self._log("=== refresh ===")

        # (A) 先列出裝置（無 awk）
        cmd_dev = "nmcli -t -e yes -f DEVICE,TYPE,STATE dev status"
        self._remember(cmd_dev); self._log("$", cmd_dev)
        run_cmd_async(self, cmd_dev, self._on_device, timeout=5.0)

        # (Z) 設置逾時守門員
        QTimer.singleShot(12000, self._scan_timeout_guard)  # 12s

    def _pick_wifi_if(self, text: str) -> str:
        cand_connected, cand_disconnected = "", ""
        for ln in (text or "").splitlines():
            parts = self._split_nmcli(ln.strip())
            if len(parts) < 3: continue
            dev, typ, state = parts[0], parts[1], parts[2]
            if typ != "wifi": continue
            if "p2p" in dev:  # 排除 P2P
                continue
            if "connected" in state:
                cand_connected = dev
            elif "disconnected" in state and not cand_connected:
                cand_disconnected = dev
        return cand_connected or cand_disconnected

    def _on_device(self, ec, out, err):
        self._log("device ec=", ec, "stdout:\n"+(out or ""), "stderr:\n"+(err or ""))
        self._current_dev = self._pick_wifi_if(out if ec == 0 else "")
        self._log("device picked:", self._current_dev or "(none)")

        # (B) 目前連線 SSID
        cmd_cur = "nmcli -t -e yes -f ACTIVE,SSID dev wifi"
        self._remember(cmd_cur); self._log("$", cmd_cur)
        run_cmd_async(self, cmd_cur, self._on_current, timeout=4.0)

        # (C) 先 rescan（能指定 ifname 就指定）
        cmd_rescan = f"nmcli dev wifi rescan ifname {nmcli_quote(self._current_dev)}" if self._current_dev else "nmcli dev wifi rescan"
        self._remember(cmd_rescan); self._log("$", cmd_rescan)
        run_cmd_async(self, cmd_rescan, self._after_rescan, timeout=7.0)

    def _on_current(self, ec, out, err):
        cur = ""
        if ec == 0 and out is not None:
            for ln in out.splitlines():
                parts = self._split_nmcli(ln.strip())
                if len(parts) >= 2 and parts[0] == "yes":
                    cur = parts[1]; break
        self.lbl_status.setText(f"目前連線：{cur}" if cur else "目前連線：無")
        self._log("current SSID ec=", ec, "→", repr(cur), "err=", err or "")

    def _after_rescan(self, ec, out, err):
        self._log("rescan ec=", ec, "out=", out or "", "err=", err or "")
        def _list():
            base = "nmcli -t -e yes -f IN-USE,SSID,SECURITY,SIGNAL dev wifi list"
            cmd_list = f'{base} ifname {nmcli_quote(self._current_dev)}' if self._current_dev else base
            self._remember(cmd_list); self._log("$", cmd_list)
            run_cmd_async(self, cmd_list, self._on_scan_first, timeout=8.0)
        QTimer.singleShot(1200, _list)

    def _on_scan_first(self, ec, out, err):
        lines = [ln for ln in (out or "").splitlines() if ln.strip()]
        self._log("list#1 ec=", ec, "nlines=", len(lines), "err=", err or "")
        if ec == 0 and len(lines) < 2:
            # 第二次不指定 ifname 再掃一次，合併
            cmd_scan2 = "nmcli -t -e yes -f IN-USE,SSID,SECURITY,SIGNAL dev wifi list"
            primary_out = out or ""
            self._remember(cmd_scan2); self._log("$", cmd_scan2)
            run_cmd_async(
                self, cmd_scan2,
                lambda e2, o2, s2: self._on_scan_merge(primary_out=primary_out, ec=e2, out=o2, err=s2),
                timeout=8.0
            )
        else:
            self._on_scan(ec, out, err)

    def _on_scan_merge(self, primary_out: str, ec: int, out: str, err: str):
        merged = []
        for raw in (primary_out, out or ""):
            for ln in raw.splitlines():
                ln = ln.rstrip("\n")
                if ln and ln not in merged:
                    merged.append(ln)
        self._log("list#merge ec=", ec, "merged_n=", len(merged))
        self._on_scan(0 if merged else ec, "\n".join(merged), err)

    def _scan_timeout_guard(self):
        if not self._is_scanning:
            return
        self._is_scanning = False
        self._show_loading(False)
        self._toast("掃描逾時。已停止等待。", ms=2200)
        self._log("TIMEOUT: scan")
        # 自動跑診斷（貼在 console 與 debug 區）
        self._run_diag_bundle()

    def _run_diag_bundle(self):
        cmds = [
            "nmcli -v",
            "nmcli general status",
            "nmcli radio all",
            "nmcli dev status",
            "rfkill list",
            "ip link",
            "iw dev",
            "nmcli -t -e yes -f IN-USE,SSID,SECURITY,SIGNAL dev wifi list",
        ]
        out_lines = []
        def _mk_cb(title):
            def _cb(ec, so, se):
                out_lines.append(f"$ {title}\n# ec={ec}\n{so}\n{se}\n")
                if len(out_lines) == len(cmds):
                    blob = "\n".join(out_lines)
                    self._log("=== DIAG ===\n" + blob)
                    self._set_debug((self.lbl_debug.text() + "\n\n[診斷]\n" + blob)[:4000])
            return _cb
        for c in cmds:
            self._remember(c); self._log("$", c)
            run_cmd_async(self, c, _mk_cb(c), timeout=6.0)

    def _on_scan(self, ec, out, err):
        self._is_scanning = False
        self._show_loading(False)
        self._clear_list()

        if ec != 0:
            msg = (err or out or "").strip()
            if "not found" in (err or "").lower():
                msg = "找不到 nmcli，請先安裝 NetworkManager。"
            elif "device not managed" in ((out or "") + (err or "")).lower():
                msg = "Wi-Fi 介面未由 NetworkManager 管理（unmanaged）。"
            self._set_debug(f"掃描失敗：{msg}")
            self._toast(f"掃描失敗：{msg or '未知錯誤'}", ms=2800)
            self._log("scan FAILED:", msg)
            return

        current = self._current_ssid_text()
        items, seen = [], set()

        for ln in (out or "").splitlines():
            parts = self._split_nmcli(ln)
            if len(parts) < 4:
                parts = ln.split(":")
                if len(parts) < 4: continue
            inuse = (parts[0].strip() == "*")
            ssid  = (parts[1] or "").strip()
            sec   = (parts[2] or "--").strip()
            try: sig = int((parts[3] or "0").strip())
            except: sig = 0
            key = (ssid, sec)
            if key in seen: continue
            seen.add(key)
            connected = bool(inuse or (ssid and ssid == current))
            item = NetworkItem(
                ssid=ssid, security=sec, signal=sig, connected=connected,
                on_connect=self._connect_flow, on_disconnect=self._disconnect_flow
            )
            self.list_layout.addWidget(item); items.append(item)

        preview = ", ".join([(it.ssid or "(隱藏)") for it in items[:6]])
        self._set_debug(
            "dev=" + (self._current_dev or "(未偵測)") + f"；AP 共 {len(items)} 筆" +
            (f"；前幾個：{preview}" if items else "") +
            ("\n指令：\n" + "\n".join(f"- {c}" for c in self._debug_last_cmds))
        )
        self._log("summary ->\n" + (self.lbl_debug.text() or ""))

        if not items:
            hint = QLabel("找不到 Wi-Fi 熱點。可能無無線介面、被 rfkill 封鎖，或未由 NetworkManager 管理。")
            hint.setStyleSheet("background:transparent; color:#444; font-weight:600;")
            self.list_layout.addWidget(hint)

        if not self._tail_spacer:
            self._tail_spacer = QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.list_layout.addItem(self._tail_spacer)

    # ---------- UI helpers ----------
    def _clear_list(self):
        if not self.list_layout: return
        for i in reversed(range(self.list_layout.count())):
            it = self.list_layout.takeAt(i)
            w = it.widget()
            if w is None: continue
            if w is self.loading: self.loading.hide()
            else: w.deleteLater()

    def _show_loading(self, on: bool):
        if on:
            if self.loading_movie: self.loading_movie.start()
            else: self.loading.setText("掃描中…")
            already = any(self.list_layout.itemAt(i).widget() is self.loading
                          for i in range(self.list_layout.count()))
            if not already:
                self.list_layout.insertWidget(0, self.loading, alignment=Qt.AlignCenter)
            self.loading.show()
        else:
            if self.loading_movie: self.loading_movie.stop()
            self.loading.hide()

    def _current_ssid_text(self) -> str:
        t = self.lbl_status.text()
        return t.split("：", 1)[-1].strip() if "：" in t else ""

    # ---------- connect / disconnect ----------
    def _connect_flow(self, item: NetworkItem):
        ssid = item.ssid or ""
        if ssid == "":
            self._toast("此 AP 為隱藏 SSID，暫不支援。", ms=2200); return
        if item.security and item.security.lower() not in ("--", "none"):
            kb = VirtualKeyboardDialog(self, f"連線到「{ssid}」")
            if kb.exec() != QDialog.Accepted: return
            pw = kb.get_text()
            if not pw: self._toast("未輸入密碼。", ms=1500); return
            self._toast("嘗試連線中…")
            cmd = f"nmcli dev wifi connect {nmcli_quote(ssid)} password {nmcli_quote(pw)}"
        else:
            self._toast("嘗試連線中…")
            cmd = f"nmcli dev wifi connect {nmcli_quote(ssid)}"
        self._remember(cmd); self._log("$", cmd)
        run_cmd_async(self, cmd, lambda ec, out, err: self._on_connect_result(ec, out, err, item), timeout=15.0)

    def _on_connect_result(self, ec, out, err, item: NetworkItem):
        self._log("connect ec=", ec, "out=", out or "", "err=", err or "")
        if ec == 0:
            item.set_connected(True)
            self._toast(f"已連線到「{item.ssid or '(隱藏)'}」", ms=1800)
            self.refresh()
        else:
            self._toast(err or out or "連線失敗", ms=2600)

    def _disconnect_flow(self, item: NetworkItem):
        self._toast("正在斷線…")
        if self._current_dev:
            cmd = f"nmcli dev disconnect {nmcli_quote(self._current_dev)}"
        else:
            cmd = "nmcli -t -e yes -f DEVICE,TYPE dev status | awk -F: '$2==\"wifi\"{print $1; exit}' | xargs -r -n1 -I{} nmcli dev disconnect {}"
        self._remember(cmd); self._log("$", cmd)
        run_cmd_async(self, cmd, self._on_disconnected, timeout=8.0)

    def _on_disconnected(self, ec, out, err):
        self._log("disconnect ec=", ec, "out=", out or "", "err=", err or "")
        if ec == 0:
            self._toast("已斷線", ms=1600); self.refresh()
        else:
            self._toast(err or "斷線失敗", ms=2400)

    def _toast(self, text: str, ms=1800):
        GlassToast(self, text, ms)



# ======================== 齒輪（半透明玻璃） ========================
class SettingsButton(QToolButton):
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
                background-color: rgba(255, 255, 255, 140);
                color: #0e0e0f;
                border-radius: 34px;
                border: 1px solid rgba(255,255,255,0.7);
            }
            QToolButton:hover  { background-color: rgba(255, 255, 255, 170); }
            QToolButton:pressed{ background-color: rgba(245,245,245, 190); }
        """)
        eff = QGraphicsDropShadowEffect(self)
        eff.setBlurRadius(24)
        eff.setOffset(0, 6)
        eff.setColor(QColor(0, 0, 0, 90))
        self.setGraphicsEffect(eff)

    def update_position(self):
        p = self.parent()
        if p:
            x = p.width() - self.width() - 18
            y = 18
            self.move(x, y)

    def showEvent(self, e):
        self.update_position()
        super().showEvent(e)


# ======================== MainWindow ========================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        pal = self.palette()
        pal.setColor(QPalette.Window, QColor(0, 0, 0))
        self.setPalette(pal)
        self.setAutoFillBackground(True)

        self.stage = Stage()
        self.stage.setParent(self)

        self.btn = SettingsButton(self, on_click=self._on_settings_btn)
        self.btn.raise_()
        self.btn.show()

        self.overlay = SettingsOverlay(
            on_back      = self._on_overlay_back,
            on_reload    = self._on_overlay_reload,
            on_network   = self._on_overlay_network,
            on_dev_exit  = self._on_overlay_dev_exit,
            parent=self
        )
        self.overlay.setGeometry(0, 0, self.width(), self.height())
        self.overlay.lower()

        self.net_overlay = NetworkOverlay(self)
        self.net_overlay.setVisible(False)
        self.showFullScreen()

    def resizeEvent(self, _):
        self.stage.setGeometry(0, 0, self.width(), self.height())
        self.btn.update_position()
        self.overlay.setGeometry(0, 0, self.width(), self.height())
        if self.net_overlay.isVisible():
            self.net_overlay.setGeometry(0, 0, self.width(), self.height())

    def keyPressEvent(self, e: QKeyEvent):
        if e.key() == Qt.Key_Escape:
            QApplication.quit()
        elif e.key() == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()

    def _on_settings_btn(self):
        self.overlay.show_overlay()

    def _on_overlay_back(self):
        self.overlay.hide_overlay()

    def _on_overlay_reload(self):
        self.overlay.hide_overlay()
        GlassToast(self, "已發送重載指令", 1400)

    def _on_overlay_network(self):
        self.overlay.hide_overlay()
        self.net_overlay.show_overlay()

    def _on_overlay_dev_exit(self):
        self.overlay.hide_overlay()
        # 實際結束流程交由 ROS2 節點


# ======================== ROS2 節點 ========================
class RobotFaceNode(Node):
    def __init__(self, ui: MainWindow):
        super().__init__("robot_face_node")
        self.ui = ui

        self._mouth_spec = LayerSpec(MOUTH_GIF_PATH, MOUTH_SPEED, MOUTH_SIZE, MOUTH_OFFSET)
        self.ui.stage.set_mouth(self._mouth_spec, play=False)

        self._apply_animation(DEFAULT_ANIM)

        self.create_subscription(RosString, "face/animation", self._on_anim, 10)
        self.create_subscription(RosBool,   "face/speaking",  self._on_speaking, 10)

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.setting_mode_pub = self.create_publisher(RosBool, "system/setting_mode", latched_qos)
        self.reload_pub       = self.create_publisher(RosBool, "system/reload_config", latched_qos)


        self.ui.btn.clicked.connect(lambda: self._toggle_settings(True))
        self.ui._on_overlay_back = lambda: self._toggle_settings(False)
        self.ui._on_overlay_reload = self._action_reload
        self.ui._on_overlay_network = self._action_network
        self.ui._on_overlay_dev_exit = self._action_dev_exit

        self._qt_timer = QTimer()
        self._qt_timer.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0.0))
        self._qt_timer.start(10)

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

    def _toggle_settings(self, show: bool):
        if show:
            self.setting_mode_pub.publish(RosBool(data=True))
            self.ui.overlay.show_overlay()
        else:
            self.setting_mode_pub.publish(RosBool(data=False))
            self.ui.overlay.hide_overlay()

    def _action_reload(self):
        self.reload_pub.publish(RosBool(data=True))
        GlassToast(self.ui, "已發送重載指令", 1400)
        self._toggle_settings(False)

    def _action_network(self):
        # 設定模式仍保持啟用
        self.setting_mode_pub.publish(RosBool(data=True))
        self.ui.overlay.hide_overlay()
        self.ui.net_overlay.show_overlay()


    def _action_dev_exit(self):
        self.get_logger().warn("[DevMode] 將結束所有 ROS2 相關進程並退出")
        self._kill_ros2_launch_processes()
        QApplication.quit()

    def _kill_ros2_launch_processes(self):
        my_pid = os.getpid()
        patterns = [
            r"ros2 launch", r"ros2 run", r"launch\.py", r"colcon.*launch",
            r"kirox_robot", r"robotears", r"roboteyes", r"robotbrain", r"robotbody", r"robotface",
        ]

        def _kill_where(sig: str):
            for pat in patterns:
                try:
                    subprocess.run(
                        ["bash", "-lc", f"pgrep -f \"{pat}\" | grep -v -w {my_pid} | xargs -r -n1 -I{{}} kill -{sig} {{}}"],
                        check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                    )
                except Exception:
                    pass

        _kill_where("TERM")
        subprocess.run(["bash", "-lc", "sleep 0.8"], check=False)
        _kill_where("TERM")
        subprocess.run(["bash", "-lc", "sleep 0.5"], check=False)
        _kill_where("KILL")

        final_cmds = [
            "pkill -f '^python3 .*ros2.*launch' || true",
            "pkill -f '^/usr/bin/python3 .*launch.*\\.py' || true",
            "pkill -f 'ros2 launch' || true",
            "pkill -f 'ros2 run' || true",
            "pkill -f 'kirox_robot' || true",
            "pkill -f 'robotears|roboteyes|robotbrain|robotbody|robotface' || true",
        ]
        for c in final_cmds:
            try:
                subprocess.run(["bash", "-lc", c], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass


# ======================== 進入點 ========================
def main(argv=None):
    QApplication.setAttribute(Qt.AA_SynthesizeTouchForUnhandledMouseEvents, True)
    QApplication.setAttribute(Qt.AA_SynthesizeMouseForUnhandledTouchEvents, True)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)

    rclpy.init(args=argv)
    app = QApplication(sys.argv)

    try:
        app.setFont(QFont("SF Pro Text", 12))
    except Exception:
        pass

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
