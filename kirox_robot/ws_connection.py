#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import base64
import json
import os
import threading
import time
from typing import Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import CompressedImage  # 訂閱壓縮影像（JPEG bytes）

from kirox_robot.robotclient.client_api import create_instance, AsyncWSClient
from kirox_robot.robotclient.get_config import get_robot_config


class WSConnectionNode(Node):
    def __init__(self):
        super().__init__("ws_connection_async")

        # ---- 參數 ----
        self.declare_parameter("botid", "31517165-19e5-44d5-8562-350cb071d1ae")
        self.declare_parameter("tts_frames_per_buffer", 2048)
        self.declare_parameter("audio_only", False)
        self.declare_parameter("keepalive_interval_sec", 20)
        self.declare_parameter("min_round_interval_sec", 1.0)
        self.declare_parameter("auto_enable_rec_after_done", True)

        p = self.get_parameter
        self.botid = p("botid").value
        cfg = get_robot_config(self.botid)

        self.prompt_style = cfg.promptstyle
        self.tts_voice = cfg.voicename
        self.tts_lang = "zh"
        self.tts_fpb = int(p("tts_frames_per_buffer").value)
        self.rest_create_url = "https://agent.xbotworks.com/create_instance"
        self.ws_url = "wss://agent.xbotworks.com/ws"
        self.audio_only = bool(p("audio_only").value)
        self.keepalive_sec = int(p("keepalive_interval_sec").value)
        self.min_round_interval = float(p("min_round_interval_sec").value)
        self.auto_enable_rec = bool(p("auto_enable_rec_after_done").value)
        self.rgb_latest_sub = str("/vision/rgb_latest")

        # ---- ROS 介面 ----
        self.latest_img: Optional[CompressedImage] = None
        self.latest_audio_meta: Dict[str, Any] = {}

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 影像 / 音訊中繼 / 觸發
        self.create_subscription(CompressedImage, self.rgb_latest_sub, self._on_rgb_latest, latched_qos)
        self.create_subscription(String, "ears/latest_audio_meta", self._on_latest_audio_meta, latched_qos)
        self.create_subscription(Bool, "brain/triggered", self._on_triggered, 10)

        # 錄音開關（latched）
        self.rec_enable_pub = self.create_publisher(Bool, "ears/record_enable", latched_qos)
        self.rec_enable_pub.publish(Bool(data=True))  # 啟動即開啟錄音

        # NEW: 嘴巴動畫布林狀態（即時狀態，用 VOLATILE）
        speaking_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.speaking_pub = self.create_publisher(Bool, "face/speaking", speaking_qos)
        self.speaking_pub.publish(Bool(data=False))  # 啟動時關閉嘴巴

        # 狀態
        self._last_round_t = 0.0
        self._last_audio_sig: Optional[str] = None
        self._busy = False
        self._ws_ready = False  # 只有 True 才允許送回合

        # 建立/更新 instance（REST）
        self._call_create_instance()

        # 啟動 WS 客戶端（獨立事件圈）
        self._loop = asyncio.new_event_loop()
        self._client = AsyncWSClient(
            node=self,
            ws_url=self.ws_url,
            hello={
                "botid": self.botid,
                "PROMPT_STYLE": self.prompt_style,
                "TTS_VOICE": self.tts_voice,
                "TTS_LANG": self.tts_lang,
                "TTS_FRAMES_PER_BUFFER": self.tts_fpb,
            },
            keepalive_sec=self.keepalive_sec,
            on_connected=self._on_ws_connected,
            on_disconnected=self._on_ws_disconnected,
            on_play_start=self._on_play_start,
            on_play_end=self._on_play_end,
        )
        self._loop_thread = threading.Thread(
            target=self._loop.run_until_complete,
            args=(self._client.connect_forever(),),
            daemon=True,
        )
        self._loop_thread.start()

        self.get_logger().info(
            f"WSConnectionNode ready | audio_only={self.audio_only} | "
            f"min_round_interval={self.min_round_interval}s | rgb_latest_sub={self.rgb_latest_sub}"
        )

    # ---------- REST ----------
    def _call_create_instance(self):
        res = create_instance(
            botid=self.botid,
            prompt_style=self.prompt_style,
            voice_name=self.tts_voice,
            language=self.tts_lang,
            frames_per_buffer=self.tts_fpb,
            server_url=self.rest_create_url,
            timeout=10,
        )
        if res is None:
            self.get_logger().warn("create_instance 失敗（稍後重連會重試）")
        else:
            self.get_logger().info("create_instance 成功")

    # ---------- WS 狀態回呼 ----------
    def _on_ws_connected(self):
        self._ws_ready = True
        self.get_logger().info("[ws] connected")
        # 連上時復位：允許錄音、關閉嘴巴動畫
        self.rec_enable_pub.publish(Bool(data=True))
        self.speaking_pub.publish(Bool(data=False))
        self._busy = False

    def _on_ws_disconnected(self):
        self._ws_ready = False
        self.get_logger().warn("[ws] disconnected")
        # 斷線復位：允許錄音、關閉嘴巴動畫、解鎖
        self.rec_enable_pub.publish(Bool(data=True))
        self.speaking_pub.publish(Bool(data=False))
        self._busy = False

    def _on_play_start(self):
        # server 宣告即將播放 → 關錄音避免自錄，嘴巴動畫打開
        self.get_logger().info("[playback] start → disable recording")
        self.rec_enable_pub.publish(Bool(data=False))
        self.speaking_pub.publish(Bool(data=True))

    def _on_play_end(self):
        self.get_logger().info("[playback] end")
        # 播放結束 → 嘴巴動畫關閉
        self.speaking_pub.publish(Bool(data=False))
        if self.auto_enable_rec:
            self.rec_enable_pub.publish(Bool(data=True))
        self._busy = False

    # ---------- ROS Callbacks ----------
    def _on_rgb_latest(self, msg: CompressedImage):
        self.latest_img = msg
        # 可視需要紀錄大小
        # size = len(msg.data) if msg and msg.data is not None else 0

    def _on_latest_audio_meta(self, msg: String):
        try:
            self.latest_audio_meta = json.loads(msg.data) if msg.data else {}
        except Exception as e:
            self.get_logger().warn(f"解析 latest_audio_meta 失敗：{e}")

    def _on_triggered(self, msg: Bool):
        if not msg.data:
            return
        if not self._ws_ready:
            self.get_logger().info("WS 未連線，忽略此次觸發")
            return
        if self._busy:
            self.get_logger().info("忙碌中（上一輪尚未結束），忽略此次觸發")
            return
        now = time.monotonic()
        if (now - self._last_round_t) < self.min_round_interval:
            return
        self._last_round_t = now

        # 進入忙碌，交由背景執行送出流程
        self._busy = True
        threading.Thread(target=self._send_round_bg, daemon=True).start()

    # ---------- 準備資料並送出 ----------
    def _send_round_bg(self):
        image_b64 = None

        # 若啟用影像，直接包裝 CompressedImage 的 JPEG 為 data URL
        if not self.audio_only and self.latest_img is not None:
            try:
                jpeg_bytes = bytes(self.latest_img.data)
                if jpeg_bytes:
                    image_b64 = "data:image/jpeg;base64," + base64.b64encode(jpeg_bytes).decode("ascii")
                    self.get_logger().info(f"[send_round] attach image, bytes={len(jpeg_bytes)}")
            except Exception as e:
                self.get_logger().warn(f"影像封裝失敗：{e}")

        wav_bytes, audio_sig = self._read_latest_wav_and_sig()
        if not wav_bytes:
            self.get_logger().warn("找不到最新語音，取消本回合")
            self._busy = False
            self.rec_enable_pub.publish(Bool(data=True))
            self.speaking_pub.publish(Bool(data=False))
            return

        if audio_sig and audio_sig == self._last_audio_sig:
            self.get_logger().info("音檔未變更，忽略此次觸發")
            self._busy = False
            self.rec_enable_pub.publish(Bool(data=True))
            self.speaking_pub.publish(Bool(data=False))
            return
        self._last_audio_sig = audio_sig

        # 在背景執行緒中，同步等待 asyncio coroutine 完成
        future = asyncio.run_coroutine_threadsafe(
            self._client.send_round(image_b64, wav_bytes), self._loop
        )
        
        try:
            # 等待結果
            ok = future.result(timeout=15)  # 設置15秒超時
            if not ok:
                self.get_logger().warn("send_round 回傳 False，恢復錄音/解鎖")
                self._reset_state_after_failure()
        except Exception as e:
            self.get_logger().error(f"send_round 執行失敗: {e}，恢復錄音/解鎖")
            self._reset_state_after_failure()

    def _read_latest_wav_and_sig(self) -> Tuple[Optional[bytes], Optional[str]]:
        meta = self.latest_audio_meta or {}
        path = meta.get("path") or ""
        ts = meta.get("ts")
        sha = meta.get("sha256") or ""
        if not path or not os.path.isfile(path):
            return None, None
        try:
            with open(path, "rb") as f:
                data = f.read()
        except Exception as e:
            self.get_logger().warn(f"讀取音檔失敗：{e}")
            return None, None
        sig = f"{ts}:{sha}:{path}"
        return data, sig

    def _reset_state_after_failure(self):
        """通訊失敗後重置狀態"""
        self._busy = False
        self.rec_enable_pub.publish(Bool(data=True))
        self.speaking_pub.publish(Bool(data=False))

    # ---------- lifecycle ----------
    def destroy_node(self):
        try:
            if self._loop.is_running():
                self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = WSConnectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
