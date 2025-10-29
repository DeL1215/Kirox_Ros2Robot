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
from sensor_msgs.msg import CompressedImage  # 影像 (JPEG bytes)

from kirox_robot.robotclient.client_api import create_instance, AsyncWSClient
from kirox_robot.robotclient.get_config import get_robot_config


class WSConnectionNode(Node):
    """
    負責：
    - 跟雲端 agent 溝通 (REST create_instance + WebSocket)
    - 控制錄音 on/off、嘴巴動畫 on/off
    - 在互動回合中送音檔 / 視覺截圖
    - 廣播系統連線狀態，讓 UI/其他 node 用

    介面協議 (ROS topics):
    ------------------------------------------------
    輸入：
      - brain/triggered (Bool) :
            True 表示 "使用者講完/要送一輪"
      - ears/latest_audio_meta (String) :
            {"path": "/tmp/...wav", "ts":..., "sha256":...}
      - vision/rgb_latest (sensor_msgs/CompressedImage)
      - system/setting_mode (Bool) :
            True = 使用者正在設定(顯示 Wi-Fi 面板等)，暫停互動
      - system/reload_config (Bool) :
            True = 重新抓取參數+重建 instance+強制重連 WS

    輸出 (Publishers)：
      - ears/record_enable (Bool, latched):
            True  = 允許錄音
            False = 關閉錄音(避免自錄 TTS 播放)
      - face/speaking (Bool, volatile):
            True  = 嘴巴動畫顯示(播 TTS)
            False = 嘴巴關
      - ws/status (String, latched):
            "connected" / "disconnected"
            *純狀態指示，不等於顯示 error*
      - face/animation (String, latched):
            "No_connection" 只在「真的處理不了互動 / create_instance 網路錯誤」時送出
            face 會自己顯示 5 秒然後回 baseline
      - agent/ci_status (String, latched):
            "pending" = create_instance 處理中  → face 用來顯示 loading3
            "ok"      = create_instance 完成
            "error"   = create_instance 失敗(網斷/逾時等)
    """

    def __init__(self):
        super().__init__("ws_connection_async")

        # ---- 參數 ----
        self.declare_parameter("botid", "31517165-19e5-44d5-8562-350cb071d1ae")
        self.declare_parameter("tts_frames_per_buffer", 2048)
        self.declare_parameter("audio_only", False)
        self.declare_parameter("keepalive_interval_sec", 20)
        self.declare_parameter("min_round_interval_sec", 1.0)
        self.declare_parameter("auto_enable_rec_after_done", True)
        self.declare_parameter("round_timeout_sec", 60)  # 單輪 request + 等回應的本地 timeout 秒數

        # ---- 把參數讀出來 ----
        p = self.get_parameter
        self.botid = p("botid").value

        cfg = get_robot_config(self.botid)
        if not cfg:
            raise RuntimeError("get_robot_config(botid) 失敗，無法啟動 WSConnectionNode")

        # 這些欄位後續 reload 時會被覆寫
        self.prompt_style = cfg.promptstyle
        self.tts_voice = cfg.voicename
        self.tts_lang = getattr(cfg, "language", "zh")
        self.tts_fpb = int(p("tts_frames_per_buffer").value)

        self.rest_create_url = "https://agent.xbotworks.com/create_instance"
        self.ws_url = "wss://agent.xbotworks.com/ws"

        self.audio_only = bool(p("audio_only").value)
        self.keepalive_sec = int(p("keepalive_interval_sec").value)
        self.min_round_interval = float(p("min_round_interval_sec").value)
        self.auto_enable_rec = bool(p("auto_enable_rec_after_done").value)
        self.round_timeout_sec = int(p("round_timeout_sec").value)
        self.rgb_latest_sub = str("vision/rgb_latest")

        self.config_for_reload = cfg  # 記當前設定

        # ---- 狀態 ----
        self.latest_img: Optional[CompressedImage] = None
        self.latest_audio_meta: Dict[str, Any] = {}
        self._last_round_t = 0.0
        self._last_audio_sig: Optional[str] = None
        self._busy = False
        self._ws_ready = False          # 只有 True 才能送回合
        self.is_setting_mode = False    # 設定面板打開時就不處理 trigger

        # ------------------------------------------------
        # QoS profiles
        # ------------------------------------------------
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        speaking_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ------------------------------------------------
        # Publishers
        # ------------------------------------------------
        # 錄音 on/off
        self.rec_enable_pub = self.create_publisher(Bool, "ears/record_enable", latched_qos)
        self.rec_enable_pub.publish(Bool(data=True))  # 啟動時允許錄音

        # 嘴巴動畫開關
        self.speaking_pub = self.create_publisher(Bool, "face/speaking", speaking_qos)
        self.speaking_pub.publish(Bool(data=False))   # 預設嘴巴關

        # 臉的臨時動畫 (No_connection 用)
        self.face_anim_pub = self.create_publisher(String, "face/animation", latched_qos)

        # 對 UI / face 用來判斷 create_instance 進度
        self.ci_status_pub = self.create_publisher(String, "agent/ci_status", latched_qos)
        # 先預設 "pending"，因為一啟動我們等下就會 call create_instance
        self.ci_status_pub.publish(String(data="pending"))

        # 對 UI 用來判斷 ws 當前是否連上，但【不等於】要顯示 error
        self.ws_status_pub = self.create_publisher(String, "ws/status", latched_qos)
        self.ws_status_pub.publish(String(data="disconnected"))

        # ------------------------------------------------
        # Subscriptions
        # ------------------------------------------------
        # 圖片（取最新一張）
        self.create_subscription(
            CompressedImage, self.rgb_latest_sub,
            self._on_rgb_latest, latched_qos
        )

        # 音訊中繼資料（wav 路徑、hash 等）
        self.create_subscription(
            String, "ears/latest_audio_meta",
            self._on_latest_audio_meta, latched_qos
        )

        # 來自 brain 的觸發信號（True = 試著發一輪到雲端）
        self.create_subscription(
            Bool, "brain/triggered",
            self._on_triggered, 10
        )

        # 設定模式 (True 時 UI在設定/使用者操作wifi，不要打擾)
        self.create_subscription(
            RosBool, "system/setting_mode",
            self._on_setting_mode, 10
        )

        # 要求重載 config（換 voice/style/...）
        self.create_subscription(
            RosBool, "system/reload_config",
            self._on_reload_config, 10
        )

        # ------------------------------------------------
        # 啟動 WS 客戶端（獨立事件圈）
        # ------------------------------------------------
        # 這個 event loop 由 thread 維持，負責 AsyncWSClient 永續連線 & send_round
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
                "force": True,  # server可搶占舊連線，避免 BOTID_IN_USE
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

        # ------------------------------------------------
        # 啟動時就同步 create_instance (建/更新 Agent)
        # ------------------------------------------------
        # Face 會看 agent/ci_status:
        #   pending -> show loading3
        #   ok      -> 可以回到 idle (或後續真 idle)
        #   error   -> face 也會短暫顯示 No_connection (我們也會直接 publish)
        self._call_create_instance()

        self.get_logger().info(
            f"WSConnectionNode ready | "
            f"audio_only={self.audio_only} | "
            f"min_round_interval={self.min_round_interval}s | "
            f"round_timeout={self.round_timeout_sec}s | "
            f"rgb_latest_sub={self.rgb_latest_sub}"
        )

    # ==========================================================
    # Config lifecycle
    # ==========================================================

    def _apply_config(self, cfg) -> None:
        """將取得的設定套用到本節點狀態（本地欄位 + 後續送 REST 用）"""
        self.config_for_reload = cfg

        # prompt_style / voice / lang 直接覆蓋（如果沒給就維持舊值）
        new_style = getattr(cfg, "promptstyle", None)
        if new_style is not None:
            self.prompt_style = new_style

        new_voice = getattr(cfg, "voicename", None)
        if new_voice is not None:
            self.tts_voice = new_voice

        new_lang = getattr(cfg, "language", None)
        if new_lang is not None:
            self.tts_lang = new_lang

        # frames_per_buffer 可能不存在或是 None，要特別防呆
        new_fpb = getattr(cfg, "frames_per_buffer", None)
        if new_fpb is not None:
            try:
                self.tts_fpb = int(new_fpb)
            except (TypeError, ValueError):
                # 如果伺服器亂給（例如字串 "NaN" 或 None），就忽略，不要炸
                self.node.get_logger().warn(
                    f"[reload] invalid frames_per_buffer={new_fpb}, keep old {self.tts_fpb}"
                )


    def _call_create_instance(self):
        """
        呼叫 REST API 建立/更新 Agent 實例（告訴伺服器使用哪個 voice/style 等）
        這是系統層的 init / reload 行為 → face 要顯示 loading3
        """
        # 標記 "pending"
        self.ci_status_pub.publish(String(data="pending"))

        payload_info = (
            f"style={self.prompt_style}, "
            f"voice={self.tts_voice}, "
            f"lang={self.tts_lang}, "
            f"fpb={self.tts_fpb}"
        )
        self.get_logger().info(f"[create_instance] start ... ({payload_info})")

        res = create_instance(
            botid=self.botid,
            prompt_style=self.prompt_style,
            voice_name=self.tts_voice,
            language=self.tts_lang,
            frames_per_buffer=self.tts_fpb,
            server_url=self.rest_create_url,
            timeout=30,
        )

        if res is None:
            # None = timeout / network 層面錯誤
            self.get_logger().warn("[create_instance] timeout or network error")
            self.ci_status_pub.publish(String(data="error"))

            # 這種是「系統無法連到 server」，使用者需要看到 No_connection
            self._publish_no_connection_once()

            # 把 ws.status 設成 disconnected (狀態提示)
            self.ws_status_pub.publish(String(data="disconnected"))

            # 主動要求 WS client 斷線，等它下次自動重連
            if self._client and getattr(self._client, "_ws", None):
                try:
                    asyncio.run_coroutine_threadsafe(
                        self._client._ws.close(),
                        self._loop
                    )
                except Exception as e:
                    self.get_logger().warn(f"[create_instance] force-close WS failed: {e}")
            return

        if not (res.get("ok") or res.get("status") == "ok"):
            # 伺服器回應了，但語意上是失敗（ex: botid無效）
            reason = (
                res.get("reason")
                or res.get("message")
                or "unknown server error"
            )
            self.get_logger().warn(f"[create_instance] server error: {reason}")

            # 這是「後端說NO」，不一定是網路問題
            # -> 我們結束pending，讓 face 回 idle (ci_status="ok")，
            #    但不一定要 No_connection，因為網路其實是通的，只是配置錯
            self.ci_status_pub.publish(String(data="ok"))
            return

        # 成功
        self.get_logger().info("[create_instance] success")
        self.ci_status_pub.publish(String(data="ok"))

    # ==========================================================
    # WebSocket 回呼 (from AsyncWSClient)
    # ==========================================================

    def _on_ws_connected(self):
        self._ws_ready = True
        self.get_logger().info("[ws] connected")
        self.ws_status_pub.publish(String(data="connected"))

        # 連上 → 預設允許錄音、關嘴巴動畫、也清忙碌旗標
        self.rec_enable_pub.publish(Bool(data=True))
        self.speaking_pub.publish(Bool(data=False))
        self._busy = False

    def _on_ws_disconnected(self):
        self._ws_ready = False
        self.get_logger().warn("[ws] disconnected")

        # 斷線後：我們仍允許錄音（等下一輪可用），先把嘴巴動畫關掉
        self.rec_enable_pub.publish(Bool(data=True))
        self.speaking_pub.publish(Bool(data=False))
        self._busy = False

        self.ws_status_pub.publish(String(data="disconnected"))

        # 🚫 不主動送 face/animation="No_connection" 在這裡
        # 理由：WS 在 create_instance / reload 流程中常短暫中斷，這不是 user-facing error
        # 真正 user-facing error 在 _on_triggered / _on_round_done 內才會送

    def _on_play_start(self):
        # 伺服器宣告要開始播放 → 關錄音避免自錄回音，嘴巴動畫打開
        self.get_logger().info("[playback] start → disable recording, speaking=True")
        self.rec_enable_pub.publish(Bool(data=False))
        self.speaking_pub.publish(Bool(data=True))

    def _on_play_end(self):
        self.get_logger().info("[playback] end → speaking=False")
        self.speaking_pub.publish(Bool(data=False))

        # 播放完畢之後，如果自動恢復錄音
        if self.auto_enable_rec:
            self.rec_enable_pub.publish(Bool(data=True))

        self._busy = False

    # ==========================================================
    # ROS callbacks (來自其他 node)
    # ==========================================================

    def _on_setting_mode(self, msg: RosBool):
        self.is_setting_mode = bool(msg.data)
        self.get_logger().info(f"[setting_mode] {self.is_setting_mode}")

    def _on_reload_config(self, msg: RosBool):
        if not msg.data:
            return
        self.get_logger().info("[reload] request received → re-fetch config...")

        try:
            # 抓最新雲端設定
            new_cfg = get_robot_config(self.botid)
            if not new_cfg:
                self.get_logger().warn("[reload] get_robot_config failed, keeping old config")
                return

            # 更新本地參數
            self._apply_config(new_cfg)
            self.get_logger().info(
                f"[reload] applied new config: "
                f"style={self.prompt_style}, "
                f"voice={self.tts_voice}, "
                f"lang={self.tts_lang}, "
                f"fpb={self.tts_fpb}"
            )

            # 同步到伺服器（會重新 set ci_status）
            self._call_create_instance()

            # 更新 WS hello，下次握手用新 voice/style/fpb
            if self._client:
                self._client.hello.update({
                    "botid": self.botid,
                    "PROMPT_STYLE": self.prompt_style,
                    "TTS_VOICE": self.tts_voice,
                    "TTS_LANG": self.tts_lang,
                    "TTS_FRAMES_PER_BUFFER": self.tts_fpb,
                    "force": True,
                })
                self.get_logger().info(f"[reload] updated WS hello: {self._client.hello}")

            # 強制斷線 (讓 connect_forever() 用新 hello 重連)
            if self._client and getattr(self._client, "_ws", None):
                try:
                    asyncio.run_coroutine_threadsafe(
                        self._client._ws.close(),
                        self._loop
                    )
                except Exception as close_err:
                    self.get_logger().warn(f"[reload] WS close() failed: {close_err}")

        except Exception as e:
            self.get_logger().error(f"[reload] exception: {e}")

    def _on_rgb_latest(self, msg: CompressedImage):
        self.latest_img = msg

    def _on_latest_audio_meta(self, msg: String):
        try:
            self.latest_audio_meta = json.loads(msg.data) if msg.data else {}
        except Exception as e:
            self.get_logger().warn(f"[audio_meta] parse fail: {e}")

    def _on_triggered(self, msg: Bool):
        """
        brain/triggered=True → 使用者講完一句，我們嘗試上傳 (音+圖) 做一輪推理/回應
        """
        if not msg.data:
            return

        if self.is_setting_mode:
            self.get_logger().info("[triggered] ignored (setting mode)")
            return

        # 如果 WS 沒連上，這是真的 user-facing fail
        if not self._ws_ready:
            self.get_logger().warn("[triggered] WS not ready, cannot send round")
            self._publish_no_connection_once()
            # 不進 busy，允許下一次 trigger 再試
            return

        # 忙碌就丟掉這次
        if self._busy:
            self.get_logger().info("[triggered] busy, skip")
            return

        # 整體限流：避免瘋狂洗伺服器
        now = time.monotonic()
        if (now - self._last_round_t) < self.min_round_interval:
            self.get_logger().info("[triggered] rate-limited, skip this round")
            return
        self._last_round_t = now

        # 進入忙碌，實際送資料在另一個 thread
        self._busy = True
        threading.Thread(target=self._send_round_bg, daemon=True).start()

    # ==========================================================
    # 送出一輪 (背景thread → 丟 coroutine 給 asyncio loop)
    # ==========================================================

    def _send_round_bg(self):
        """
        - 把 latest_img 轉成 data URL (base64)
        - 把 latest_audio_meta 指到最新 wav
        - 丟給 AsyncWSClient.send_round(...) (async)
        - Future 完成後交給 _on_round_done 處理
        """
        img_b64 = None

        if not self.audio_only and self.latest_img is not None:
            try:
                jpeg_bytes = bytes(self.latest_img.data)
                if jpeg_bytes:
                    img_b64 = (
                        "data:image/jpeg;base64," +
                        base64.b64encode(jpeg_bytes).decode("ascii")
                    )
                    self.get_logger().info(
                        f"[send_round] attach image ({len(jpeg_bytes)} bytes)"
                    )
            except Exception as e:
                self.get_logger().warn(f"[send_round] image pack fail: {e}")

        wav_bytes, audio_sig = self._read_latest_wav_and_sig()
        if not wav_bytes:
            self.get_logger().warn("[send_round] no audio, cancel")
            self._reset_state_after_failure(user_error=False)
            return

        # 如果最新音檔signature跟上次一樣，就視為重複觸發：丟掉
        if audio_sig and audio_sig == self._last_audio_sig:
            self.get_logger().info("[send_round] duplicate audio, skip")
            self._reset_state_after_failure(user_error=False)
            return
        self._last_audio_sig = audio_sig

        # 在背景thread裡，把 coroutine 丟給 asyncio loop
        fut = asyncio.run_coroutine_threadsafe(
            self._client.send_round(
                img_b64,
                wav_bytes,
                timeout=self.round_timeout_sec
            ),
            self._loop
        )
        fut.add_done_callback(self._on_round_done)

    def _on_round_done(self, fut: cf.Future):
        """
        send_round(...) 結束後呼叫
        注意：這個 callback 會在 asyncio 執行緒中跑
        """
        try:
            ok = fut.result()  # 若 coroutine raise，這裡會丟出例外
            if not ok:
                # send_round 自己回 False => 代表沒成功
                self.get_logger().warn("[send_round] returned False")
                self._reset_state_after_failure(user_error=True)

        except asyncio.TimeoutError:
            self.get_logger().error(
                f"[send_round] timeout ({self.round_timeout_sec}s)"
            )
            # timeout 代表「這次互動真的沒辦法送出或拿回應」
            self.ws_status_pub.publish(String(data="disconnected"))
            self._reset_state_after_failure(user_error=True)

        except Exception as e:
            self.get_logger().error(f"[send_round] exception: {e}")
            self.ws_status_pub.publish(String(data="disconnected"))
            self._reset_state_after_failure(user_error=True)

    # ==========================================================
    # Helpers
    # ==========================================================

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
            self.get_logger().warn(f"[read_wav] open fail: {e}")
            return None, None

        sig = f"{ts}:{sha}:{path}"
        return data, sig

    def _reset_state_after_failure(self, user_error: bool):
        """
        每次一輪互動沒成功或提前中止時呼叫
        - 解鎖 busy
        - 關嘴巴動畫
        - 允許錄音
        - 若這是 user-facing 的失敗(=user_error=True)，就丟 No_connection
        """
        self._busy = False
        self.rec_enable_pub.publish(Bool(data=True))
        self.speaking_pub.publish(Bool(data=False))

        if user_error:
            self._publish_no_connection_once()

    def _publish_no_connection_once(self):
        """
        對 face 發出一次 "No_connection"。
        face 那邊會：
          - 顯示 No_connection.gif
          - 啟動 5 秒計時
          - 計時結束後自動 fallback 回 baseline
            (baseline 可能是 init / loading3 / idle，看當下旗標)
        我們不在這邊做計時，這是 face 的責任。
        """
        try:
            self.face_anim_pub.publish(String(data="No_connection"))
        except Exception as e:
            self.get_logger().warn(f"[face_anim] publish No_connection failed: {e}")

    # ==========================================================
    # Lifecycle
    # ==========================================================

    def destroy_node(self):
        try:
            if self._loop.is_running():
                # 結束 asyncio loop
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
