#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robotmouth_node.py — 中文發音導向嘴型輸出

輸出 7 種嘴型狀態字串，同時也是前端 PNG 名稱：

    OPEN_BIG       # 超大張嘴: 「啊──」「ㄚ/ㄞ/ㄟ」強音、喊
    OPEN_MED       # 中等張嘴: 「ㄜ、ㄝ、ㄤ、ㄥ」主體口型
    OPEN_SOFT      # 小聲/語尾: 輕「啊..嗯..」收下來、弱音
    TIGHT_CONS     # 緊縫子音: ㄋ ㄉ ㄗ ㄘ ㄙ ㄓ ㄔ ㄕ 等
    CLOSED_REST    # 完全閉嘴: ㄇ ㄅ ㄆ / 休息臉 / 不說話
    PUCKER_SMALL   # 小噘嘴: ㄨ ㄩ ㄡ 短促 (去、住、就)
    PUCKER_BIG     # 大噘嘴: ㄛ ㄡ/喔── 嗚── 拉長強調

上游輸入 (std_msgs/String, topic "system/voice_stream"):
    {
      "pcm_b64": "...",  # int16 PCM little-endian mono (or multi,會downmix)
      "sr": 24000,
      "ch": 1
    }

輸出 (std_msgs/String, topic "face/mouth_shape"):
    以上其中一個字串，例如 "OPEN_MED"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString
import json
import base64
import numpy as np
import time


class RobotMouth(Node):
    def __init__(self):
        super().__init__("robotmouth")

        # ------- ROS I/O -------
        self.sub_voice = self.create_subscription(
            RosString, "system/voice_stream", self.on_voice, 10
        )

        # 選配：上游可丟 face/speaking = "true"/"false"
        # 用來延後收嘴，不是必須
        self.sub_speaking = self.create_subscription(
            RosString, "face/speaking", self.on_speaking, 10
        )

        self.pub_mouth = self.create_publisher(
            RosString, "face/mouth_shape", 10
        )

        # ------- 狀態 -------
        self.speaking_hint = False      # "我還在講話" 的提示
        self.prev_shape = "CLOSED_REST" # 上一次真的送出去的嘴型
        self.last_sent_time = 0.0       # 上次 publish 嘴型時間
        self.last_audio_time = 0.0      # 最近有音訊的時間
        self.last_non_rest_time = 0.0   # 最近送出非 CLOSED_REST 的時間
        self.pending_shape = "CLOSED_REST"

        # ------- 參數調整區 -------
        # 音量(RMS)門檻
        self.energy_silence = 0.010   # < silence => 幾乎沒聲音
        self.energy_low     = 0.030   # low ~ med
        self.energy_high    = 0.070   # med ~ big

        # 頻譜分類門檻
        # 低頻高 -> 噘嘴 (ㄨ/ㄩ/ㄡ/ㄛ 類)
        self.lowfreq_gate   = 0.40

        # 高頻高 -> 子音/擦音 (ㄋ ㄉ ㄗ ㄕ...)
        self.highfreq_gate  = 0.50

        # 節奏/平滑
        self.min_switch_interval = 0.08  # 嘴型至少 80ms 才能切一次
        self.hold_after_voice     = 0.40 # 說完話後，再撐 0.40s 才收嘴
        self.full_silence_to_rest = 0.60 # 完全沒音訊達 0.6s -> 一定收成 CLOSED_REST

        # 定時器：管理收尾/補送
        self.create_timer(0.05, self.timer_tick)

        self.get_logger().info("✅ RobotMouth ready (CN viseme set).")

    # ------------------------------------------------------------------
    # ROS Callbacks
    # ------------------------------------------------------------------
    def on_speaking(self, msg: RosString):
        """
        face/speaking: "true"/"false"
        幫助我們判斷：剛停聲但其實還在講 -> 嘴型先不要瞬間關
        """
        val = msg.data.strip().lower()
        self.speaking_hint = val in ("true", "1", "yes", "on")

    def on_voice(self, msg: RosString):
        """
        每來一塊音訊：解碼 -> 分析 -> 決定候選嘴型 -> 嘗試送出
        """
        now = time.time()
        self.last_audio_time = now

        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"[robotmouth] JSON parse error: {e}")
            return

        pcm_b64 = payload.get("pcm_b64")
        if not pcm_b64:
            return

        sr = int(payload.get("sr", 24000))
        ch = int(payload.get("ch", 1))

        # 轉成 float32 [-1,1]
        try:
            raw = base64.b64decode(pcm_b64)
            audio = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
        except Exception as e:
            self.get_logger().warn(f"[robotmouth] PCM decode fail: {e}")
            return

        if audio.size == 0:
            return

        # 多聲道轉單聲道
        if ch > 1:
            try:
                audio = audio.reshape(-1, ch).mean(axis=1)
            except Exception:
                pass  # reshape 失敗就算了

        # 產生候選嘴型
        cand = self.infer_shape(audio, sr)
        self.pending_shape = cand

        # 嘗試發佈（含平滑）
        self.maybe_publish(now)

    # ------------------------------------------------------------------
    # DSP / 分類
    # ------------------------------------------------------------------
    def infer_shape(self, audio: np.ndarray, sr: int) -> str:
        """
        回傳七選一：
            OPEN_BIG / OPEN_MED / OPEN_SOFT /
            TIGHT_CONS / CLOSED_REST /
            PUCKER_SMALL / PUCKER_BIG
        """

        # 1) 能量 (RMS)
        rms = float(np.sqrt(np.mean(audio * audio)) + 1e-12)

        # 2) 頻譜分析：抓低頻比 / 高頻比
        spec = np.abs(np.fft.rfft(audio))
        if spec.size == 0:
            return "CLOSED_REST"

        freqs = np.fft.rfftfreq(audio.size, 1.0 / sr)

        low_band  = spec[(freqs >=   0) & (freqs <  500)].sum()
        mid_band  = spec[(freqs >= 500) & (freqs < 2000)].sum()
        high_band = spec[(freqs >=2000) & (freqs < 6000)].sum()
        total = low_band + mid_band + high_band + 1e-12

        low_ratio  = float(low_band  / total)   # 低頻占比高 -> 噘嘴音 (ㄨ/ㄛ/ㄡ/ㄩ)
        high_ratio = float(high_band / total)   # 高頻占比高 -> 緊子音 (ㄋ ㄉ ㄗ ㄕ)

        # ===== 決策開始 =====

        # 0. 幾乎沒聲音 => 閉嘴休息
        if rms < self.energy_silence:
            return "CLOSED_REST"

        # 1. 噘嘴 (ㄨ ㄩ ㄡ ㄛ)
        #    用低頻厚度 + 音量分成大小噘
        if low_ratio > self.lowfreq_gate:
            if rms < self.energy_high:
                return "PUCKER_SMALL"  # 短促「不/就/去」這種ㄨ/ㄩ
            else:
                return "PUCKER_BIG"    # 拉長「喔──」「嗚──」

        # 2. 緊縫子音 (ㄋ ㄉ ㄗ ㄕ...)：高頻尖+有力
        if high_ratio > self.highfreq_gate:
            if rms >= self.energy_low:
                return "TIGHT_CONS"    # 明顯咬字子音
            else:
                # 小聲高頻但不夠力，就當作柔軟小口
                return "OPEN_SOFT"

        # 3. 不是噘嘴也不是緊縫子音 -> 走開口等級
        if rms >= self.energy_high:
            return "OPEN_BIG"   # 最大開口「啊」「ㄚ/ㄞ/ㄟ」喊/強音

        if rms >= self.energy_low:
            return "OPEN_MED"   # 中等開度「ㄜ ㄝ ㄤ ㄥ」一般說話

        # 再小一點音量 -> 語尾/小聲說
        return "OPEN_SOFT"

    # ------------------------------------------------------------------
    # 發佈 & 平滑
    # ------------------------------------------------------------------
    def maybe_publish(self, now: float):
        """
        規則：
        - 嘴型沒變就不送
        - 切換間隔 < min_switch_interval 就先忍住，避免抖
        """
        cand = self.pending_shape

        if cand == self.prev_shape:
            return

        if (now - self.last_sent_time) < self.min_switch_interval:
            return

        self.send_shape(cand, now)

    def send_shape(self, shape: str, now: float):
        """
        真正 publish 嘴型給 UI
        """
        self.pub_mouth.publish(RosString(data=shape))
        self.prev_shape = shape
        self.last_sent_time = now
        if shape != "CLOSED_REST":
            self.last_non_rest_time = now

    # ------------------------------------------------------------------
    # 定時器：處理「收尾」和「沒聲音時回 CLOSED_REST」
    # ------------------------------------------------------------------
    def timer_tick(self):
        """
        每 50ms 執行：
        1. 長時間無音訊 -> 強制 CLOSED_REST
        2. 剛停聲 -> 先撐 hold_after_voice 秒才慢慢收
        3. 如果 pending_shape 因為間隔限制沒送，這裡補送
        """
        now = time.time()

        time_since_audio = now - self.last_audio_time

        # (1) 長時間完全沒音訊 -> 直接收嘴
        if time_since_audio > self.full_silence_to_rest:
            if self.prev_shape != "CLOSED_REST":
                self.send_shape("CLOSED_REST", now)
            return

        # (2) 有一段小空白但還在「講話尾巴」
        if time_since_audio > 0.05:
            since_non_rest = now - self.last_non_rest_time

            if since_non_rest > self.hold_after_voice:
                # hold 時間到了，可以收嘴
                if self.prev_shape != "CLOSED_REST":
                    if not self.speaking_hint:
                        self.send_shape("CLOSED_REST", now)
                        return
                # speaking_hint=True 代表上游還覺得在講，暫時別收

        # (3) 若 pending_shape 還沒送，現在可以補送
        self.maybe_publish(now)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMouth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
