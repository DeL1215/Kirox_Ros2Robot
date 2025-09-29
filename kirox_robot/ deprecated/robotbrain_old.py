#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, json, asyncio, threading, logging
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

# ===== 常數 =====
WS_URI       = "wss://agent.xbotworks.com/ws"
BOTID        = "d478758a-da9c-42b8-adfc-4ccc794fc046"
PROMPT_STYLE = "惡毒，言語尖銳。"
VOICE_NAME   = "Jay"
TTS_LANG     = "zh"

SAVE_OUTPUTS = False

FRAME_MS             = 32
START_TRIGGER_FRAMES = 3
END_TRIGGER_FRAMES   = 10
TARGET_PROC_RATE     = 16000
RMS_THRESHOLD        = 1200
MAX_SEGMENT_SEC      = 30.0
START_PROB           = 0.60
END_PROB             = 0.35

INPUT_DEVICE_INDEX   = None
INPUT_DEVICE_NAME    = "GENERAL WEBCAM"


OUTPUT_DEVICE_INDEX  = None
OUTPUT_DEVICE_NAME   = "VG27AQ3A"

# 仍保留（可給 create_instance 或其他用途）
FRAMES_PER_BUFFER    = 2048

TOPIC_MSG            = "brain_msg"
TOPIC_ACTION         = "brain_action"

logging.basicConfig(
    level=os.environ.get("LOGLEVEL", "INFO"),
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("robotbrain")

# ===== 嘗試在 import sounddevice 前解出可用裝置（僅為 log；實際輸出裝置由 agent_request 控制） =====
def _resolve_device(sd, index, name, want_input):
    if index is not None:
        return index
    if name:
        try:
            for i, d in enumerate(sd.query_devices()):
                nm = d.get("name", "")
                if want_input and d.get("max_input_channels", 0) > 0 and name in nm:
                    return i
                if not want_input and d.get("max_output_channels", 0) > 0 and name in nm:
                    return i
        except Exception:
            pass
    # 回退：挑第一個可用
    try:
        for i, d in enumerate(sd.query_devices()):
            if want_input and d.get("max_input_channels", 0) > 0:
                return i
            if not want_input and d.get("max_output_channels", 0) > 0:
                return i
    except Exception:
        pass
    return None

try:
    import sounddevice as sd
    in_dev  = _resolve_device(sd, INPUT_DEVICE_INDEX,  INPUT_DEVICE_NAME,  True)
    din = sd.query_devices(in_dev) if in_dev is not None else {"name": "N/A"}
    log.info(f"[AudioPreset] py={sys.executable}")
    log.info(f"[AudioPreset] input={in_dev} [{din['name']}]")
except Exception as e:
    print("[FATAL] sounddevice 未就緒：", e)
    sys.exit(1)

# ===== 自家模組（在 robotclient/） =====
from kirox_robot.robotclient.task_manager import TaskManager
from kirox_robot.robotclient.agent_request import send_to_llm  # TaskManager 用
from kirox_robot.robotclient.vad_trigger import VADMicStreamer
try:
    from kirox_robot.robotclient.client_api import create_instance
except Exception:
    def create_instance(*args, **kwargs):
        return {"ok": False, "reason": "create_instance not available"}


class RobotBrain(Node):
    def __init__(self):
        super().__init__("robotbrain")
        self.pub_msg    = self.create_publisher(String, TOPIC_MSG, 10)
        self.pub_action = self.create_publisher(String, TOPIC_ACTION, 10)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run_asyncio, daemon=True)
        self._thread.start()
        self.get_logger().info(f"[robotbrain] started, publish -> '{TOPIC_MSG}', '{TOPIC_ACTION}'")

    def _run_asyncio(self):
        asyncio.run(self._main_async())

    async def _main_async(self):
        # 可選：預建 TTS/Agent
        try:
            resp = create_instance(
                botid=BOTID,
                prompt_style=PROMPT_STYLE,
                voice_name=VOICE_NAME,
                language=TTS_LANG,
                output_device_index=OUTPUT_DEVICE_INDEX,
                frames_per_buffer=FRAMES_PER_BUFFER,
                server_url="https://agent.xbotworks.com/create_instance",
            )
            log.info(f"[Agent] create_instance resp={resp}")
        except Exception as e:
            log.warning(f"[Agent] create_instance failed: {e}")

        tm = TaskManager(
            send_coroutine=send_to_llm,
            timeout_seconds=180.0,
            on_done=self._on_done,   # 最終完成（可選）
            on_error=self._on_error,
        )
        loop = asyncio.get_running_loop()

        self.vad = VADMicStreamer(
            task_manager=tm,
            loop=loop,
            outputs_dir="outputs",
            uri=WS_URI,
            botid=BOTID,
            voice_name=VOICE_NAME,
            language=TTS_LANG,
            prompt_style=PROMPT_STYLE,
            frame_ms=FRAME_MS,
            start_trigger_frames=START_TRIGGER_FRAMES,
            end_trigger_frames=END_TRIGGER_FRAMES,
            max_segment_sec=MAX_SEGMENT_SEC,
            target_proc_rate=TARGET_PROC_RATE,
            rms_threshold=RMS_THRESHOLD,
            start_prob=START_PROB,
            end_prob=END_PROB,
            input_device_name=INPUT_DEVICE_NAME,
            input_device_index=INPUT_DEVICE_INDEX,
            save_outputs=SAVE_OUTPUTS,

            # ★ 將輸出裝置指定往下傳，最終由 agent_request 的播放器採用
            out_device_index=OUTPUT_DEVICE_INDEX,
            out_device_name=OUTPUT_DEVICE_NAME,

            # 即時回呼：收到就 publish
            on_text=self._on_text,
            on_action=self._on_action,
        )
        self.vad.start()
        log.info("[VAD] Started. Speak...")

        try:
            while not self._stop.is_set():
                await asyncio.sleep(0.5)
        finally:
            self.vad.stop()
            await tm.wait_idle()
            log.info("[VAD] Stopped.]")

    # ===== 即時回呼：一收到就丟 topic =====
    async def _on_text(self, raw_text: str):
        msg = String()
        msg.data = raw_text if isinstance(raw_text, str) else str(raw_text)
        self.pub_msg.publish(msg)
        self.get_logger().info(f"[PUB-early text] {msg.data[:160].replace('\n',' ')}")

    async def _on_action(self, raw_action, parsed_action):
        try:
            if isinstance(raw_action, (dict, list)):
                out = json.dumps(raw_action, ensure_ascii=False)
            else:
                out = str(raw_action)
        except Exception:
            out = str(raw_action)
        msg = String(); msg.data = out
        self.pub_action.publish(msg)
        self.get_logger().info(f"[PUB-early action] {out[:160].replace('\n',' ')}")

    # =====（可選）最終完成時，也把解析後訊息丟一次到 brain_msg =====
    async def _on_done(self, req, res: dict) -> None:
        text = (res.get("parsed") or {}).get("Message") or ""
        if text:
            msg = String(); msg.data = text
            self.pub_msg.publish(msg)
            self.get_logger().info(f"[PUB-done] {text}")

    async def _on_error(self, req, err: Exception) -> None:
        self.get_logger().error(f"[ERROR] kind={getattr(req,'kind','?')} {type(err).__name__}: {err}")

    def destroy_node(self):
        self._stop.set()
        try:
            self._thread.join(timeout=3.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = RobotBrain()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
