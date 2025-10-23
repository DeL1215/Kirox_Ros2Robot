#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import base64
import json
import os
import threading
import time
from typing import Optional, Dict, Any, Tuple
import concurrent.futures as cf  # 用於辨識 TimeoutError 等

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String, Bool as RosBool
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
        self.declare_parameter("round_timeout_sec", 60)  # 單回合上傳 + 等待伺服器回應的本地等待超時

        # ---- 讀取參數 ----
        p = self.get_parameter
        self.botid = p("botid").value

        # 初始抓取設定
        cfg = get_robot_config(self.botid)
        if not cfg:
            raise RuntimeError("get_robot_config(botid) 失敗，無法啟動 WSConnectionNode")

        # 這些欄位會在 reload 時被覆寫
        self.prompt_style = cfg.promptstyle
        self.tts_voice = cfg.voicename
        self.tts_lang = getattr(cfg, "language", "zh")
        self.tts_fpb = int(p("tts_frames_per_buffer").value)

        # 伺服器端 API/WS
        self.rest_create_url = "https://agent.xbotworks.com/create_instance"
        self.ws_url = "wss://agent.xbotworks.com/ws"

        # 其他控制
        self.audio_only = bool(p("audio_only").value)
        self.keepalive_sec = int(p("keepalive_interval_sec").value)
        self.min_round_interval = float(p("min_round_interval_sec").value)
        self.auto_enable_rec = bool(p("auto_enable_rec_after_done").value)
        self.round_timeout_sec = int(p("round_timeout_sec").value)
        self.rgb_latest_sub = str("vision/rgb_latest")

        # 保存目前設定，reload 時會被替換
        self.config_for_reload = cfg

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

        # 設定模式與重載
        self.create_subscription(RosBool, "system/setting_mode", self._on_setting_mode, 10)
        self.create_subscription(RosBool, "system/reload_config", self._on_reload_config, 10)

        # 錄音開關（latched）
        self.rec_enable_pub = self.create_publisher(Bool, "ears/record_enable", latched_qos)
        self.rec_enable_pub.publish(Bool(data=True))  # 啟動即開啟錄音

        # 嘴巴動畫布林狀態（即時，VOLATILE）
        speaking_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.speaking_pub = self.create_publisher(Bool, "face/speaking", speaking_qos)
        self.speaking_pub.publish(Bool(data=False))  # 啟動時關閉嘴巴

        # **新增：face/animation（latched）僅供連線錯誤/無網路時使用**
        self.face_pub = self.create_publisher(String, "face/animation", latched_qos)

        # **新增：ws/status (latched) 讓其他節點知道連線狀態**
        self.ws_status_pub = self.create_publisher(String, "ws/status", latched_qos)
        self.ws_status_pub.publish(String(data="disconnected")) # 初始為斷線

        # 狀態
        self._last_round_t = 0.0
        self._last_audio_sig: Optional[str] = None
        self._busy = False
        self._ws_ready = False  # 只有 True 才允許送回合
        self.is_setting_mode = False

        # ---- 啟動 WS 客戶端（獨立事件圈）----
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
                "force": True,  # 允許伺服器端搶占舊連線（避免 BOTID_IN_USE）
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

        # 先把目前設定同步到伺服器（只在開機與重載時呼叫）
        self._call_create_instance()

        self.get_logger().info(
            f"WSConnectionNode ready | audio_only={self.audio_only} | "
            f"min_round_interval={self.min_round_interval}s | "
            f"round_timeout={self.round_timeout_sec}s | rgb_latest_sub={self.rgb_latest_sub}"
        )

    # ---------- 設定套用 ----------
    def _apply_config(self, cfg) -> None:
        """將取得的設定套用到本節點狀態（本地欄位 + 後續送 REST 用）"""
        self.config_for_reload = cfg
        self.prompt_style = getattr(cfg, "promptstyle", self.prompt_style)
        self.tts_voice = getattr(cfg, "voicename", self.tts_voice)
        self.tts_lang = getattr(cfg, "language", self.tts_lang)
        self.tts_fpb = int(getattr(cfg, "frames_per_buffer", self.tts_fpb))

    # ---------- REST ----------
    def _call_create_instance(self):
        """呼叫 REST API 建立或更新 Agent 實例（單一真相來源：以當前 self.* 為準）"""
        payload_info = (
            f"style={self.prompt_style}, voice={self.tts_voice}, lang={self.tts_lang}, fpb={self.tts_fpb}"
        )
        self.get_logger().info(f"Calling create_instance... ({payload_info})")
        res = create_instance(
            botid=self.botid,
            prompt_style=self.prompt_style,
            voice_name=self.tts_voice,
            language=self.tts_lang,
            frames_per_buffer=self.tts_fpb,
            server_url=self.rest_create_url,
            timeout=10,
        )
        # 僅網路/連線錯誤（返回 None）才發佈 No_connection；其他狀態不改臉
        if res is None:
            self.get_logger().warn("create_instance 失敗: timeout or network error")
            self.ws_status_pub.publish(String(data="disconnected"))
            self.face_pub.publish(String(data="No_connection"))
            # 觸發 WebSocket 客戶端重新連線，以便在恢復時更新狀態
            if self._client and self._client._ws:
                asyncio.run_coroutine_threadsafe(self._client._ws.close(), self._loop)
        elif not (res.get("ok") or res.get("status") == "ok"):
            self.get_logger().warn(f"create_instance 失敗: {res.get('reason') or res.get('message') or 'unknown server error'}")
        else:
            self.get_logger().info("create_instance 成功")

    # ---------- WS 狀態回呼 ----------
    def _on_ws_connected(self):
        self._ws_ready = True
        self.get_logger().info("[ws] connected")
        self.ws_status_pub.publish(String(data="connected"))
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
        self.ws_status_pub.publish(String(data="disconnected"))
        self.face_pub.publish(String(data="No_connection"))

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
    def _on_setting_mode(self, msg: RosBool):
        self.is_setting_mode = msg.data
        self.get_logger().info(f"Setting mode changed to: {self.is_setting_mode}")

    def _on_reload_config(self, msg: RosBool):
        """收到 system/reload_config=True → 重新抓設定 → 套用本地 → 透過 create_instance 同步到伺服器"""
        if not msg.data:
            return
        self.get_logger().info("Received config reload request → 重新抓取設定...")

        try:
            # 1) 重新抓設定（遠端/本地皆可，由 get_robot_config 決定）
            new_cfg = get_robot_config(self.botid)
            if not new_cfg:
                self.get_logger().warn("reload 失敗：get_robot_config 回傳空結果，保留舊設定")
                return

            # 2) 套用到本地欄位
            self._apply_config(new_cfg)
            self.get_logger().info(
                f"Reload OK → style={self.prompt_style}, voice={self.tts_voice}, lang={self.tts_lang}, fpb={self.tts_fpb}"
            )

            # 3) 同步到伺服器（覆寫/更新 Agent 實例）
            self._call_create_instance()

            # 4) 不送任何 WS 控制訊息；新設定將在下一輪對話自動生效
        except Exception as e:
            self.get_logger().error(f"reload 發生例外：{e}")

    def _on_rgb_latest(self, msg: CompressedImage):
        self.latest_img = msg

    def _on_latest_audio_meta(self, msg: String):
        try:
            self.latest_audio_meta = json.loads(msg.data) if msg.data else {}
        except Exception as e:
            self.get_logger().warn(f"解析 latest_audio_meta 失敗：{e}")

    def _on_triggered(self, msg: Bool):
        if not msg.data:
            return
        if self.is_setting_mode:
            self.get_logger().info("In setting mode, ignoring trigger.")
            return
        # **關鍵修改：在觸發時若未連線，則不送出請求，避免卡住**
        if not self._ws_ready:
            self.get_logger().warn("WS 未連線，忽略此次觸發。")
            # 主動發布 No_connection 動畫給使用者即時回饋
            self.face_pub.publish(String(data="No_connection"))
            # 不進入 busy 狀態，直接返回
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

        # 在背景執行緒中，將 coroutine 提交到 asyncio 事件迴圈
        future = asyncio.run_coroutine_threadsafe(
            self._client.send_round(image_b64, wav_bytes, timeout=self.round_timeout_sec),
            self._loop
        )

        # 為 future 附加一個完成時的回呼函式，該回呼將在 asyncio 執行緒中執行
        future.add_done_callback(self._on_round_done)

        # _send_round_bg 函式到此結束，不會阻塞，主執行緒可以繼續處理其他事務

    def _on_round_done(self, future: cf.Future):
        """
        當 send_round 的 Future 完成時被呼叫（在 asyncio 執行緒中）。
        """
        try:
            # 獲取結果，如果 coroutine 中發生例外，此處會重新拋出
            ok = future.result()
            if not ok:
                # send_round 內部邏輯判斷失敗（例如 WS 未連線）
                self.get_logger().warn("send_round 任務回傳 False，重置狀態。")
                self._reset_state_after_failure()

        except asyncio.TimeoutError:
            # 這是由 _client.send_round 內部 asyncio.wait_for 拋出的
            self.get_logger().error(f"send_round 任務逾時({self.round_timeout_sec}s)，重置狀態。")
            self.ws_status_pub.publish(String(data="disconnected"))
            self._reset_state_after_failure()

        except Exception as e:
            # 其他所有在 coroutine 中發生的例外
            self.get_logger().error(f"send_round 任務執行失敗: {e}，重置狀態。")
            self.ws_status_pub.publish(String(data="disconnected"))
            self._reset_state_after_failure()

    def _read_latest_wav_and_sig(self) -> Tuple[Optional[bytes], Optional[str]]:
        meta = self.latest_audio_meta or {}
        path = meta.get("path") or ""
        if not path or not os.path.isfile(path):
            return None, None
        
        ts = meta.get("ts")
        sha = meta.get("sha256") or ""
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
        # **關鍵修改：將發布 No_connection 的邏輯統一到這裡**
        self._busy = False
        self.rec_enable_pub.publish(Bool(data=True))
        self.speaking_pub.publish(Bool(data=False))

        # 發布 No_connection 動畫
        try:
            self.face_pub.publish(String(data="No_connection"))
        except Exception as e:
            self.get_logger().warn(f"發布 No_connection 動畫失敗: {e}")
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
