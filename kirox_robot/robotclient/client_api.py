#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
client_api.py — ROS2 WebSocket 客戶端工具：
- create_instance(): 呼叫 REST 建立/更新 agent instance
- AsyncWSClient: 維持 WS 長連線、keepalive、接收串流 TTS（本地播放）
  * send_round(image_b64?, wav_bytes): 傳送一回合（可選圖 + 音訊 + end:true），回 True/False
  * 事件回呼：on_connected/on_disconnected/on_play_start/on_play_end/on_text/on_action
  * 額外：自動發佈 brain/text、brain/action（std_msgs/String）供其他節點訂閱
"""

import requests
import asyncio
import json
import threading
import time
from typing import Optional, Dict, Any, Callable

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString
import websockets

try:
    import sounddevice as sd  # type: ignore
    _HAS_SD = True
except Exception:
    sd = None
    _HAS_SD = False


# -------------------- REST: create_instance --------------------

def _compact(d: Dict[str, Any]) -> Dict[str, Any]:
    """移除 None，避免覆蓋伺服器端既有設定。"""
    return {k: v for k, v in d.items() if v is not None}


def create_instance(
    botid: str,
    prompt_style: Optional[str] = None,
    voice_name: Optional[str] = None,
    language: Optional[str] = "zh",
    frames_per_buffer: Optional[int] = None,
    server_url: str = "https://agent.xbotworks.com/create_instance",
    timeout: int = 10,
) -> Optional[Dict[str, Any]]:
    """
    呼叫 FastAPI /create_instance：
      - 不存在 → 建立
      - 已存在 → 僅更新傳入的欄位
    回傳伺服器 JSON（dict），失敗回 None。
    """
    payload = _compact({
        "botid": botid,
        "PROMPT_STYLE": prompt_style,
        "TTS_VOICE": voice_name,
        "TTS_LANG": language,
        "TTS_FRAMES_PER_BUFFER": frames_per_buffer,
    })
    try:
        resp = requests.post(server_url, json=payload, timeout=timeout)
        resp.raise_for_status()
    except requests.RequestException as e:
        print(f"[create_instance] 連線錯誤: {e}")
        return None

    try:
        result = resp.json()
    except Exception:
        print(f"[create_instance] 回應非 JSON | 狀態碼={resp.status_code} | 內容={resp.text[:200]}")
        return None

    status = result.get("status")
    msg = result.get("message", "")
    if status == "ok":
        print(f"[create_instance] 成功: {msg}")
    else:
        print(f"[create_instance] 回應: status={status} message={msg}")
    return result


# -------------------- 播放器：串流 PCM -> 本地喇叭 --------------------

class PCMPlayer:
    """極簡 PCM 串流播放器：預設使用系統預設輸出裝置。"""
    def __init__(self):
        self.stream: Optional[Any] = None
        self.sr: Optional[int] = None
        self.ch: Optional[int] = None
        self._lock = threading.Lock()

    def configure(self, sample_rate: int, channels: int = 1):
        if not _HAS_SD:
            return
        with self._lock:
            if self.stream and self.sr == sample_rate and self.ch == channels:
                return
            self._close_locked()
            self.sr, self.ch = int(sample_rate), int(channels)
            self.stream = sd.OutputStream(
                samplerate=self.sr,
                channels=self.ch,
                dtype="int16",
                blocksize=0
            )
            self.stream.start()

    def write(self, pcm_bytes: bytes):
        if not _HAS_SD or not pcm_bytes:
            return
        with self._lock:
            if not self.stream:
                # server 預設 24k/mono
                self.configure(24000, 1)
            arr = np.frombuffer(pcm_bytes, dtype=np.int16)
            frames = arr if self.ch == 1 else arr.reshape(-1, self.ch)
            self.stream.write(frames)

    def _close_locked(self):
        try:
            if self.stream:
                self.stream.stop()
                self.stream.close()
        finally:
            self.stream = None

    def close(self):
        if not _HAS_SD:
            return
        with self._lock:
            self._close_locked()


# -------------------- WebSocket 非同步客戶端 --------------------

class AsyncWSClient:
    """
    持續連線 + keepalive + 接收串流 TTS（本地播放）。
    - send_round(image_b64?, wav_bytes) -> bool：送一回合（可選 image + audio + end:true）
    - 回呼：
        on_connected() / on_disconnected()
        on_play_start() / on_play_end()
        on_text(str) / on_action(Any)
    """
    def __init__(
        self,
        node: Node,
        ws_url: str,
        hello: Dict[str, Any],
        keepalive_sec: int = 20,
        on_connected: Optional[Callable[[], None]] = None,
        on_disconnected: Optional[Callable[[], None]] = None,
        on_play_start: Optional[Callable[[], None]] = None,
        on_play_end: Optional[Callable[[], None]] = None,
        on_text: Optional[Callable[[str], None]] = None,
        on_action: Optional[Callable[[Any], None]] = None,
    ):
        self.node = node
        self.ws_url = ws_url
        self.hello = hello
        self.keepalive_sec = keepalive_sec

        self.on_connected = on_connected
        self.on_disconnected = on_disconnected
        self.on_play_start = on_play_start
        self.on_play_end = on_play_end
        self.on_text = on_text
        self.on_action = on_action

        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        self._recv_task: Optional[asyncio.Task] = None
        self._ping_task: Optional[asyncio.Task] = None
        self._send_lock = asyncio.Lock()
        self._player = PCMPlayer()
        self._in_audio_fmt: Optional[tuple[str, int, int]] = None  # (fmt, sr, ch)

        # ROS 發佈器
        self.pub_text = self.node.create_publisher(RosString, "brain/text", 10)
        self.pub_action = self.node.create_publisher(RosString, "body/action", 10)
        self.pub_face = self.node.create_publisher(RosString, "face/animation", 10)

    # ---- 關鍵修復：相容性安全的連線狀態檢查 ----
    def _ws_is_open(self) -> bool:
        ws = self._ws
        if ws is None:
            return False
        # 優先嘗試常見屬性
        val = getattr(ws, "closed", None)
        if callable(val):
            try:
                val = val()
            except Exception:
                val = None
        if isinstance(val, bool):
            return not val
        cc = getattr(ws, "close_code", None)
        if cc is not None:
            return cc is None
        state = getattr(ws, "state", None)
        name = getattr(state, "name", None)
        if name:
            return name.upper() in ("OPEN",)
        # 不確定就假定開著，交給 send() 自己拋錯
        return True

    async def connect_forever(self):
        """
        永久保持連線，具備自動重連與指數回退（最大 15 秒）。
        """
        backoff = 1.0
        MAX_BACKOFF = 15.0
        while rclpy.ok():
            try:
                self.node.get_logger().info(f"嘗試連線 {self.ws_url} …")
                async with websockets.connect(self.ws_url, max_size=None) as ws:
                    self._ws = ws
                    # 握手
                    await ws.send(json.dumps(self.hello, ensure_ascii=False))
                    self.node.get_logger().info("[ws] handshake sent")

                    # 成功後立即重置回退時間
                    backoff = 1.0

                    # 觸發連線成功回呼
                    if callable(self.on_connected):
                        try:
                            self.on_connected()
                        except Exception as cb_err:
                            self.node.get_logger().warn(f"on_connected 回呼錯誤: {cb_err}")

                    # 啟動背景任務
                    self._recv_task = asyncio.create_task(self._recv_loop(ws))
                    self._ping_task = asyncio.create_task(self._keepalive_loop(ws))

                    # 等其中任一任務結束（例如 recv 因斷線而結束）
                    await asyncio.wait(
                        {self._recv_task, self._ping_task},
                        return_when=asyncio.FIRST_COMPLETED
                    )

            except Exception as e:
                self.node.get_logger().warn(f"WS 斷線或錯誤：{e}")
            finally:
                self._stop_bg_tasks()
                self._player.close()
                self._ws = None
                if callable(self.on_disconnected):
                    try:
                        self.on_disconnected()
                    except Exception as cb_err:
                        self.node.get_logger().warn(f"on_disconnected 回呼錯誤: {cb_err}")
                self.node.get_logger().warn("[ws] disconnected")

            self.node.get_logger().warn(f"{backoff:.1f}s 後重連…")
            await asyncio.sleep(backoff)
            # 指數回退但不超過 15 秒
            backoff = min(MAX_BACKOFF, max(1.0, backoff * 2))

    def _stop_bg_tasks(self):
        for t in (self._recv_task, self._ping_task):
            if t and not t.done():
                t.cancel()
        self._recv_task = None
        self._ping_task = None

    async def _keepalive_loop(self, ws: websockets.WebSocketClientProtocol):
        while True:
            await asyncio.sleep(self.keepalive_sec)
            try:
                await ws.send(json.dumps({"ping": int(time.time())}))
            except Exception:
                # 送失敗表示連線可能已斷，讓外層感知
                break

    async def _recv_loop(self, ws: websockets.WebSocketClientProtocol):
        while True:
            msg = await ws.recv()

            # 音訊串流（bytes）
            if isinstance(msg, (bytes, bytearray)):
                self._player.write(msg)
                continue

            # 文字 JSON
            try:
                d = json.loads(msg)
            except Exception:
                self.node.get_logger().info(str(msg)[:200])
                continue

            # TTS 格式宣告 -> 播放開始
            if "audio_format" in d:
                if callable(self.on_play_start):
                    try:
                        self.on_play_start()
                    except Exception as cb_err:
                        self.node.get_logger().warn(f"on_play_start 回呼錯誤: {cb_err}")
                self._in_audio_fmt = (
                    d.get("audio_format", "pcm_s16le"),
                    int(d.get("sample_rate", 24000)),
                    int(d.get("channels", 1)),
                )
                self._player.configure(self._in_audio_fmt[1], self._in_audio_fmt[2])
                continue

            # 助理「文字」回覆
            if "text" in d:
                text = d.get("text") or ""
                # 發佈 brain/text
                try:
                    self.pub_text.publish(RosString(data=text))
                except Exception as pub_err:
                    self.node.get_logger().warn(f"publish brain/text 失敗: {pub_err}")
                # 回呼
                if callable(self.on_text):
                    try:
                        self.on_text(text)
                    except Exception as cb_err:
                        self.node.get_logger().warn(f"on_text 回呼錯誤: {cb_err}")
                else:
                    self.node.get_logger().info(f"[assistant] {text}")
                continue

            # 助理「動作」回覆（可能是 dict / list / str）
            if "action" in d:
                action_payload = d.get("action")
                try:
                    if isinstance(action_payload, dict):
                        # 取出 Face 與 Action
                        face_value = action_payload.get("Face", "")
                        action_value = action_payload.get("Action", [])

                        # 發佈 Face（直接字串）
                        if face_value is not None:
                            self.pub_face.publish(RosString(data=str(face_value)))

                        # 發佈 Action（保持 JSON 格式，因為可能是 list/dict）
                        if action_value is not None:
                            action_str = json.dumps(action_value, ensure_ascii=False) \
                                        if isinstance(action_value, (dict, list)) else str(action_value)
                            self.pub_action.publish(RosString(data=action_str))
                    else:
                        self.node.get_logger().warn(f"action 欄位不是 dict，收到: {action_payload}")
                except Exception as e:
                    self.node.get_logger().error(f"解析/發佈 action_payload 失敗: {e}")

                if callable(self.on_action):
                    try:
                        self.on_action(action_payload)
                    except Exception as cb_err:
                        self.node.get_logger().warn(f"on_action 回呼錯誤: {cb_err}")
                else:
                    self.node.get_logger().info(f"[assistant action] {action_payload}")
                continue

            # 一回合結束
            if d.get("done") is True:
                if callable(self.on_play_end):
                    try:
                        self.on_play_end()
                    except Exception as cb_err:
                        self.node.get_logger().warn(f"on_play_end 回呼錯誤: {cb_err}")
                continue

            # 錯誤
            if "error" in d:
                self.node.get_logger().warn(f"[server_error] {d.get('code','')} {d['error']}")
                if callable(self.on_play_end):
                    try:
                        self.on_play_end()
                    except Exception as cb_err:
                        self.node.get_logger().warn(f"on_play_end 回呼錯誤: {cb_err}")
                continue

    async def send_round(self, image_b64: Optional[str], wav_bytes: bytes, timeout: float) -> bool:
        """
        傳送一回合：
          1) 可選圖：{"image": "<data-url or base64>"}
          2) 音訊 bytes （wav 容器）
          3) {"end": true}
        成功則回 True，否則 False。此函式包含自己的超時控制。
        """
        try:
            # 使用 asyncio.wait_for 來包裹整個傳送操作
            return await asyncio.wait_for(self._do_send_round(image_b64, wav_bytes), timeout=timeout)
        except asyncio.TimeoutError:
            self.node.get_logger().error(f"傳送操作在 {timeout} 秒後超時。")
            # 重新拋出 TimeoutError，讓上層（例如 WSConnectionNode._on_round_done）能捕獲
            raise
        except Exception as e:
            self.node.get_logger().warn(f"送回合失敗：{e}")
            if callable(self.on_play_end):
                try:
                    self.on_play_end()
                except Exception:
                    pass
            return False

    async def _do_send_round(self, image_b64: Optional[str], wav_bytes: bytes) -> bool:
        async with self._send_lock:
            if self._ws_is_open():
                if image_b64:
                    await self._ws.send(json.dumps({"image": image_b64}, ensure_ascii=False))
                await self._ws.send(wav_bytes)
                await self._ws.send(json.dumps({"end": True}))
                return True
            else:
                self.node.get_logger().warn("WS 尚未連線，略過本回合")
                return False
