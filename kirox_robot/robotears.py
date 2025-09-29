#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
robotears.py — 極簡版（無 services / 無覆寫指令）
- VAD（Silero）+ openWakeWord
- 一律把本段音訊存到 RAM：/dev/shm/robotears/seg_latest.wav（原子覆寫）
- 若 save_outputs=True，另複製一份到 outputs 目錄（meta.archive_path 提供參考）
- 發佈（latched）：
    * /ears/latest_audio_meta : std_msgs/String(JSON)
      {path, ts, sr, duration, reason, sha256, archive_path?}
    * /ears/vad_score         : std_msgs/String(JSON)
    * /ears/oww_score         : std_msgs/Float32  ← 段內最大值; 句尾再補一次峰值
- 事件（非 latched）：
    * /ears/vad_start : std_msgs/String(JSON)
    * /ears/vad_end   : std_msgs/String(JSON)
- 新增：
    * 訂閱 /ears/record_enable (std_msgs/Bool, latched)
      False → 完全忽略/丟棄音訊（避免自我回錄與請求併發）
"""

import os
import time
import json
import wave
import queue
import hashlib
import shutil
import threading
from typing import Optional, Tuple, List, Dict, Any

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import sounddevice as sd
import torch  # Silero VAD
from openwakeword.model import Model as OWWModel  # 熱詞

# ================= 工具 =================

def _resample_linear(x: np.ndarray, src_sr: int, dst_sr: int) -> np.ndarray:
    if x.size == 0 or src_sr == dst_sr:
        return x.astype(np.int16, copy=False)
    t_src = np.linspace(0.0, 1.0, num=len(x), endpoint=False, dtype=np.float64)
    n_dst = int(round(len(x) * dst_sr / src_sr))
    t_dst = np.linspace(0.0, 1.0, num=n_dst, endpoint=False, dtype=np.float64)
    y = np.interp(t_dst, t_src, x.astype(np.float64))
    return np.clip(np.rint(y), -32768, 32767).astype(np.int16)

def _find_sd_device_index_by_name_substr(name_substr: Optional[str]) -> Optional[int]:
    if not name_substr:
        return None
    key = (name_substr or "").lower()
    try:
        for i, d in enumerate(sd.query_devices()):
            if d.get("max_input_channels", 0) > 0 and key in (d.get("name") or "").lower():
                return i
    except Exception:
        pass
    return None

def _atomic_write_wav(path: str, pcm16: np.ndarray, sample_rate: int) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f"{path}.tmp"
    with wave.open(tmp, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(pcm16.tobytes())
    os.replace(tmp, path)  # 原子替換

def _new_archive_path(outputs_dir: str, prefix: str) -> str:
    ts = time.strftime("%Y%m%d_%H%M%S")
    os.makedirs(outputs_dir, exist_ok=True)
    return os.path.join(outputs_dir, f"{prefix}_{ts}_{int(time.time()*1000)%100000}.wav")

def _sha256_file(path: str) -> Optional[str]:
    try:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
        return h.hexdigest()
    except Exception:
        return None

# ================= 節點 =================

class RobotEarsNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_ears_node")

        # ---- 參數 ----
        self.declare_parameter("outputs_dir", "outputs")
        self.declare_parameter("save_outputs", False)
        self.declare_parameter("tmp_wav_path", "/dev/shm/robotears/seg_latest.wav")  # RAM 暫存
        self.declare_parameter("input_device_name", "pulse")
        self.declare_parameter("input_device_index", -1)

        # VAD/Seg
        self.declare_parameter("frame_ms", 32)
        self.declare_parameter("start_trigger_frames", 3)
        self.declare_parameter("end_trigger_frames", 20)
        self.declare_parameter("target_proc_rate", 16000)
        self.declare_parameter("rms_threshold", 150)
        self.declare_parameter("start_prob", 0.60)
        self.declare_parameter("end_prob", 0.35)
        self.declare_parameter("max_segment_sec", 30.0)

        # Score params
        self.declare_parameter("cont_cap_s", 3.0)
        self.declare_parameter("snr_max_db", 30.0)
        self.declare_parameter("min_utt_s", 0.12)
        self.declare_parameter("min_snr_db", 4.0)
        self.declare_parameter("p_min", 0.30)
        self.declare_parameter("w_a", 0.50)
        self.declare_parameter("w_s", 0.30)
        self.declare_parameter("w_c", 0.20)
        self.declare_parameter("w_o", 0.10)
        self.declare_parameter("noise_ema_alpha", 0.1)

        # OWW（熱詞）
        self.declare_parameter("oww_model_path", "/home/del1215/ros2_ws/src/kirox_robot/kirox_robot/models/kirox.onnx")
        self.declare_parameter("oww_frame_ms", 80)
        self.declare_parameter("oww_enable_speex_ns", False)
        self.declare_parameter("oww_threshold", 0.80)

        # 讀參數
        self.outputs_dir: str = self.get_parameter("outputs_dir").value
        self.save_outputs: bool = self.get_parameter("save_outputs").value
        self.tmp_wav_path: str = self.get_parameter("tmp_wav_path").value
        self.input_device_name: str = self.get_parameter("input_device_name").value
        _idx = self.get_parameter("input_device_index").value
        self.input_device_index: Optional[int] = None if int(_idx) < 0 else int(_idx)

        self.frame_ms: int = int(self.get_parameter("frame_ms").value)
        self.start_trigger_frames: int = int(self.get_parameter("start_trigger_frames").value)
        self.end_trigger_frames: int = int(self.get_parameter("end_trigger_frames").value)
        self.target_proc_rate: int = int(self.get_parameter("target_proc_rate").value)
        self.rms_threshold: int = int(self.get_parameter("rms_threshold").value)
        self.start_prob: float = float(self.get_parameter("start_prob").value)
        self.end_prob: float = float(self.get_parameter("end_prob").value)
        self.max_segment_sec: float = float(self.get_parameter("max_segment_sec").value)

        self.cont_cap_s: float = float(self.get_parameter("cont_cap_s").value)
        self.snr_max_db: float = float(self.get_parameter("snr_max_db").value)
        self.min_utt_s: float = float(self.get_parameter("min_utt_s").value)
        self.min_snr_db: float = float(self.get_parameter("min_snr_db").value)
        self.p_min: float = float(self.get_parameter("p_min").value)
        self.w_a: float = float(self.get_parameter("w_a").value)
        self.w_s: float = float(self.get_parameter("w_s").value)
        self.w_c: float = float(self.get_parameter("w_c").value)
        self.w_o: float = float(self.get_parameter("w_o").value)
        self.noise_ema_alpha: float = float(self.get_parameter("noise_ema_alpha").value)

        self.oww_model_path: str = self.get_parameter("oww_model_path").value
        self.oww_frame_ms: int = int(self.get_parameter("oww_frame_ms").value)
        self.oww_enable_speex_ns: bool = bool(self.get_parameter("oww_enable_speex_ns").value)
        self.oww_threshold: float = float(self.get_parameter("oww_threshold").value)

        # QoS：latched（TRANSIENT_LOCAL）
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Topic：事件與「最新錄音 / 最新評分 / 熱詞分數」
        self.pub_start  = self.create_publisher(String, "ears/vad_start", 10)
        self.pub_end    = self.create_publisher(String, "ears/vad_end", 10)
        self.pub_latest = self.create_publisher(String, "ears/latest_audio_meta", latched_qos)
        self.pub_score  = self.create_publisher(String, "ears/vad_score", latched_qos)
        self.pub_oww    = self.create_publisher(Float32, "ears/oww_score", latched_qos)

        # **錄音開關**（預設 True）
        self._record_enable = True
        self.create_subscription(
            Bool, "ears/record_enable", self._on_record_enable, latched_qos
        )

        # 背景事件 queue（音訊執行緒 -> ROS 執行緒）
        self._evq: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=200)
        self._timer = self.create_timer(0.05, self._pump_events)  # 20Hz

        # VAD 參數
        self.proc_rate = int(self.target_proc_rate)
        self.dtype = "int16"
        self.proc_frame_samples = max(
            int(self.proc_rate * self.frame_ms / 1000),
            int(np.ceil(self.proc_rate / 31.25))
        )  # >=512 @16k

        # 載入 Silero
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"[VAD] torch device={device}")
        self.silero_model, _ = torch.hub.load(
            repo_or_dir="snakers4/silero-vad",
            model="silero_vad",
            trust_repo=True
        )
        self.silero_model.eval().to(device)
        self._torch_device = device

        # 載入 OWW
        if not os.path.exists(self.oww_model_path):
            raise FileNotFoundError(f"OWW 模型不存在: {self.oww_model_path}")
        self.oww_model = OWWModel(
            wakeword_models=[self.oww_model_path],
            enable_speex_noise_suppression=self.oww_enable_speex_ns
        )
        self._oww_block = max(1, int(self.proc_rate * (self.oww_frame_ms / 1000.0)))
        _probe = self.oww_model.predict(np.zeros(self._oww_block, dtype=np.int16))
        if not _probe:
            raise RuntimeError("openWakeWord 載入失敗：predict 回傳空字典")
        self._oww_key = next(iter(_probe.keys()))
        self._oww_buf = np.zeros((0,), dtype=np.int16)

        # OWW：段內最大值與對外已發佈最大值（本段）
        self._oww_seg_max = 0.0
        self._oww_pub_max = 0.0
        self._latest_oww_score: float = 0.0

        # 選擇輸入裝置
        self.input_rate, self.input_device = self._pick_input_device()
        self._proc_buf = np.zeros((0,), dtype=np.int16)

        # 工作執行緒
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None

        # 記憶體狀態
        self._latest_meta: Dict[str, Any] = {}
        self._latest_score: Dict[str, Any] = {}

        # 底噪（dB）EMA 估計
        self._noise_db_ema: Optional[float] = None
        self._eps = 1e-6

        # 啟動
        self._start_stream()
        self.get_logger().info("[robotears] started")

    # ---------- 錄音開關 ----------
    def _on_record_enable(self, msg: Bool):
        prev = self._record_enable
        self._record_enable = bool(msg.data)
        if self._record_enable != prev:
            self.get_logger().info(f"[record_enable] -> {self._record_enable}")
        if not self._record_enable:
            # 立即重置 VAD 狀態（避免殘留）
            self._proc_buf = np.zeros((0,), dtype=np.int16)

    # ---------- 裝置 ----------
    def _pick_input_device(self) -> Tuple[int, Optional[int]]:
        if self.input_device_index is not None:
            try:
                sd.check_input_settings(device=self.input_device_index, samplerate=self.proc_rate, channels=1, dtype=self.dtype)
                return self.proc_rate, self.input_device_index
            except Exception:
                pass
        idx = _find_sd_device_index_by_name_substr(self.input_device_name)
        if idx is not None:
            try:
                sd.check_input_settings(device=idx, samplerate=self.proc_rate, channels=1, dtype=self.dtype)
                return self.proc_rate, idx
            except Exception:
                pass
        for sr in [self.proc_rate, 48000, 44100, 32000, 16000]:
            try:
                sd.check_input_settings(device=None, samplerate=sr, channels=1, dtype=self.dtype)
                return sr, None
            except Exception:
                continue
        return 44100, None

    # ---------- Stream ----------
    def _open_stream_with_fallback(self):
        trial_rates = [self.input_rate, 48000, 44100, 32000, 16000]
        for sr in trial_rates:
            try:
                sd.check_input_settings(device=self.input_device, samplerate=sr, channels=1, dtype=self.dtype)
                blocksize = int(sr * self.proc_frame_samples / self.proc_rate)  # 保持相近 frame 時長
                stream = sd.InputStream(
                    samplerate=sr,
                    channels=1,
                    dtype=self.dtype,
                    blocksize=blocksize,
                    callback=self._audio_callback,
                    device=self.input_device
                )
                stream.__enter__()
                self.input_rate = sr
                return stream
            except Exception:
                continue
        raise RuntimeError("No valid input sample rate for selected device")

    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            pass
        if indata.ndim == 1:
            mono = indata.astype(np.int16, copy=False)
        else:
            mono = np.clip(np.rint(indata.astype(np.float32).mean(axis=1)), -32768, 32767).astype(np.int16)
        try:
            self._evq.put_nowait({"kind": "audio_chunk", "data": mono})
        except queue.Full:
            pass

    # ---------- 背景主迴圈 ----------
    def _start_stream(self) -> None:
        if self._worker and self._worker.is_alive():
            return
        self._stop.clear()
        self._stream = self._open_stream_with_fallback()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self) -> None:
        in_speech = False
        voiced_count = 0
        unvoiced_count = 0
        seg_samples: List[np.ndarray] = []
        seg_start_t = 0.0

        # 句內統計
        p_sum = 0.0
        snr_n_sum = 0.0
        frame_cnt = 0
        onset_flag = 0

        while not self._stop.is_set():
            try:
                ev = self._evq.get(timeout=0.1)
            except queue.Empty:
                continue

            if ev["kind"] != "audio_chunk":
                self._evq.put(ev)
                continue

            # **關閉錄音時，直接丟棄音訊 frame**
            if not self._record_enable:
                in_speech = False
                voiced_count = unvoiced_count = 0
                seg_samples = []
                p_sum = snr_n_sum = 0.0
                frame_cnt = 0
                onset_flag = 0
                continue

            chunk_in: np.ndarray = ev["data"]
            # 重採樣到處理率
            chunk_proc = _resample_linear(chunk_in, self.input_rate, self.proc_rate)

            # 前次殘留
            if self._proc_buf.size > 0:
                chunk_proc = np.concatenate([self._proc_buf, chunk_proc], axis=0)
                self._proc_buf = np.zeros((0,), dtype=np.int16)

            offset = 0
            while offset + self.proc_frame_samples <= len(chunk_proc):
                frame_i16 = chunk_proc[offset:offset + self.proc_frame_samples]
                offset += self.proc_frame_samples

                # ====== VAD：RMS + Silero ======
                rms = float(np.sqrt(np.mean(frame_i16.astype(np.float32) ** 2)) + self._eps)
                L_db = 20.0 * np.log10(rms + self._eps)

                if rms < self.rms_threshold:
                    speech_prob = 0.0
                else:
                    f32 = (frame_i16.astype(np.float32) / 32768.0).clip(-1.0, 1.0)
                    with torch.no_grad():
                        tens = torch.from_numpy(f32).to(self._torch_device)
                        speech_prob = float(self.silero_model(tens, self.proc_rate).item())

                if self._noise_db_ema is None:
                    self._noise_db_ema = L_db
                else:
                    if speech_prob < self.start_prob * 0.6:
                        a = self.noise_ema_alpha
                        self._noise_db_ema = (1 - a) * self._noise_db_ema + a * L_db

                snr_db = max(L_db - (self._noise_db_ema or L_db), 0.0)
                snr_n = min(snr_db, self.snr_max_db) / self.snr_max_db  # 0~1

                is_speech = speech_prob >= (self.start_prob if not in_speech else self.end_prob)

                if not in_speech:
                    if is_speech:
                        voiced_count += 1
                        if voiced_count >= self.start_trigger_frames:
                            in_speech = True
                            seg_samples = [frame_i16.copy()]
                            seg_start_t = time.time()
                            unvoiced_count = 0
                            voiced_count = 0
                            onset_flag = 1
                            p_sum = 0.0
                            snr_n_sum = 0.0
                            frame_cnt = 0
                            # 句子開始：重置段內峰值
                            self._oww_seg_max = 0.0
                            self._oww_pub_max = 0.0
                            # 通知句子開始
                            self._evq.put({"kind": "vad_start", "ts": seg_start_t})
                    else:
                        voiced_count = 0
                else:
                    # 句內累計
                    p_sum += speech_prob
                    snr_n_sum += snr_n
                    frame_cnt += 1
                    seg_samples.append(frame_i16.copy())

                    if is_speech:
                        unvoiced_count = 0
                    else:
                        unvoiced_count += 1

                    end_by_silence = (unvoiced_count >= self.end_trigger_frames)
                    end_by_length = ((time.time() - seg_start_t) >= self.max_segment_sec)

                    if end_by_silence or end_by_length:
                        dur = time.time() - seg_start_t
                        pcm = np.concatenate(seg_samples, axis=0) if seg_samples else np.zeros((0,), dtype=np.int16)
                        why = "silence" if end_by_silence else "max_len"

                        # ========== 保存音檔 ==========
                        try:
                            _atomic_write_wav(self.tmp_wav_path, pcm, self.proc_rate)
                        except Exception as e:
                            self.get_logger().error(f"[save] 寫入 RAM 音檔失敗: {e}")

                        archive_path = ""
                        if self.save_outputs:
                            try:
                                archive_path = _new_archive_path(self.outputs_dir, "seg")
                                shutil.copyfile(self.tmp_wav_path, archive_path)
                            except Exception as e:
                                self.get_logger().warn(f"[save] 複製到 outputs 失敗: {e}")
                                archive_path = ""

                        sha = _sha256_file(self.tmp_wav_path) or ""
                        end_ts = time.time()
                        meta = {
                            "path": self.tmp_wav_path,
                            "ts": end_ts,
                            "sr": self.proc_rate,
                            "duration": float(dur),
                            "reason": why,
                            "sha256": sha,
                            "archive_path": archive_path,
                        }
                        self._evq.put({"kind": "vad_end", **meta})

                        # === 句尾評分 ===
                        mean_p = (p_sum / max(frame_cnt, 1)) if frame_cnt > 0 else 0.0
                        mean_snr_n = (snr_n_sum / max(frame_cnt, 1)) if frame_cnt > 0 else 0.0
                        cont = min(dur / self.cont_cap_s, 1.0)
                        onset = 1 if onset_flag else 0

                        invalid = False
                        reason_invalid = ""
                        if dur < self.min_utt_s:
                            invalid, reason_invalid = True, "too_short"
                        approx_snr_db = mean_snr_n * self.snr_max_db
                        if not invalid and approx_snr_db < self.min_snr_db:
                            invalid, reason_invalid = True, "low_snr"
                        if not invalid and mean_p < self.p_min:
                            invalid, reason_invalid = True, "low_p"

                        if invalid:
                            score = 0.0
                        else:
                            score = (self.w_a * mean_p +
                                     self.w_s * mean_snr_n +
                                     self.w_c * cont +
                                     self.w_o * onset)
                            score = float(max(0.0, min(1.0, score)))

                        tau_s = self.end_trigger_frames * (self.frame_ms / 1000.0)
                        score_msg = {
                            "ts": end_ts,
                            "score": score,
                            "duration": float(dur),
                            "tau_s": float(tau_s),
                            "components": {
                                "mean_p": float(mean_p),
                                "mean_snr_n": float(mean_snr_n),
                                "cont": float(cont),
                                "onset": int(onset),
                            },
                            "weights": {
                                "w_a": float(self.w_a),
                                "w_s": float(self.w_s),
                                "w_c": float(self.w_c),
                                "w_o": float(self.w_o),
                            },
                            "thresholds": {
                                "min_utt_s": float(self.min_utt_s),
                                "min_snr_db": float(self.min_snr_db),
                                "p_min": float(self.p_min),
                                "snr_max_db": float(self.snr_max_db),
                            },
                            "reason": why if not invalid else f"invalid:{reason_invalid}",
                        }

                        self.get_logger().info(
                            f"[score] {score:.3f} | dur={dur:.2f}s, mean_p={mean_p:.2f}, "
                            f"mean_snr_n={mean_snr_n:.2f} (~{mean_snr_n*self.snr_max_db:.1f}dB), "
                            f"cont={cont:.2f}, onset={onset}, end='{why}'" +
                            (f", invalid={reason_invalid}" if invalid else "")
                        )

                        self._set_latest_score(score_msg)

                        # Reset
                        in_speech = False
                        voiced_count = 0
                        unvoiced_count = 0
                        seg_samples = []
                        p_sum = snr_n_sum = 0.0
                        frame_cnt = 0
                        onset_flag = 0

                # ====== OWW：按 oww_frame_ms 推論（段內取最大；新高才發佈） ======
                if self._oww_block > 0:
                    if self._oww_buf.size == 0:
                        self._oww_buf = frame_i16.copy()
                    else:
                        self._oww_buf = np.concatenate([self._oww_buf, frame_i16], axis=0)

                    while self._oww_buf.size >= self._oww_block:
                        blk = self._oww_buf[:self._oww_block]
                        self._oww_buf = self._oww_buf[self._oww_block:]
                        scores = self.oww_model.predict(blk.astype(np.int16, copy=False))
                        s = float(scores.get(self._oww_key, 0.0))
                        self._latest_oww_score = s

                        if s > self._oww_seg_max:
                            self._oww_seg_max = s
                        if s > self._oww_pub_max:
                            self._oww_pub_max = s
                            if s >= self.oww_threshold:
                                self.get_logger().info(f"[OWW] wake word detected! score={s:.3f}")
                            self.pub_oww.publish(Float32(data=float(self._oww_pub_max)))

            if offset < len(chunk_proc):
                self._proc_buf = chunk_proc[offset:].copy()

        # 結束
        try:
            self._stream.__exit__(None, None, None)
        except Exception:
            pass

    # ---------- 事件泵（ROS 執行緒） ----------
    def _pump_events(self) -> None:
        processed = 0
        while processed < 50:
            try:
                ev = self._evq.get_nowait()
            except queue.Empty:
                break

            kind = ev.get("kind")
            if kind == "vad_start":
                self._on_vad_start(ev)
            elif kind == "vad_end":
                self._on_vad_end(ev)
            processed += 1

    # ---------- VAD 事件 ----------
    def _on_vad_start(self, ev: Dict[str, Any]) -> None:
        self._oww_seg_max = 0.0
        self._oww_pub_max = 0.0
        msg = {"event": "start", "ts": ev.get("ts", time.time())}
        self.pub_start.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    def _on_vad_end(self, ev: Dict[str, Any]) -> None:
        meta = {
            "path": ev.get("path", self.tmp_wav_path),
            "ts": ev.get("ts", time.time()),
            "sr": ev.get("sr", self.proc_rate),
            "duration": float(ev.get("duration", 0.0)),
            "reason": ev.get("reason", "unknown"),
            "sha256": ev.get("sha256", ""),
            "archive_path": ev.get("archive_path", ""),
        }
        # 事件
        self.pub_end.publish(String(data=json.dumps({"event": "end", **meta}, ensure_ascii=False)))
        # latched 最新錄音
        self._publish_latest(meta)
        self.get_logger().info(f"[latest] {meta['path']} ({meta['duration']:.2f}s)")

        # 句尾補發段內最大 OWW 分數（確保 latched 反映峰值）
        self.pub_oww.publish(Float32(data=float(self._oww_seg_max)))

    # ---------- 最新錄音 / 評分（記憶體 + latched） ----------
    def _publish_latest(self, meta: Dict[str, Any]) -> None:
        self._latest_meta = dict(meta)
        self.pub_latest.publish(String(data=json.dumps(self._latest_meta, ensure_ascii=False)))

    def _set_latest_score(self, score_msg: Dict[str, Any]) -> None:
        self._latest_score = dict(score_msg)
        self.pub_score.publish(String(data=json.dumps(self._latest_score, ensure_ascii=False)))

    # ---------- 關閉 ----------
    def destroy_node(self):
        self._stop.set()
        if hasattr(self, "_worker") and self._worker and self._worker.is_alive():
            self._worker.join(timeout=2.0)
        super().destroy_node()

def main():
    rclpy.init()
    node = RobotEarsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
