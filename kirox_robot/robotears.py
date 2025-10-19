#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
robotears.py — 精簡優化版（含 SpeexDSP-NS 降噪 + 穩定度打分）
- Silero VAD + openWakeWord
- SpeexDSP-NS：逐幀降噪（speexdsp-ns==0.1.2）
- 打分：用 RMS 穩定度（stability）取代原本 SNR 權重（w_s 不改名）
- 句尾存檔：/dev/shm/robotears/seg_latest.wav
- Topics：ears/vad_start, ears/vad_end, ears/latest_audio_meta, ears/vad_score, ears/oww_score, ears/ready
- 錄音開關（latched）：ears/record_enable
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
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

import sounddevice as sd
import torch
from openwakeword.model import Model as OWWModel
from speexdsp_ns import NoiseSuppression  # ★ 新增：SpeexDSP-NS

# ---------------- 工具 ----------------

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
    os.replace(tmp, path)

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

# ---------------- 節點 ----------------

class RobotEarsNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_ears_node")

        # ---- 裝置/檔案 ----
        self.declare_parameter("outputs_dir", "outputs")
        self.declare_parameter("save_outputs", False)
        self.declare_parameter("tmp_wav_path", "/dev/shm/robotears/seg_latest.wav")
        self.declare_parameter("input_device_name", "pulse")
        self.declare_parameter("input_device_index", -1)

        # ---- 偵測/計分（必要少量）----
        self.declare_parameter("frame_ms", 32)
        self.declare_parameter("start_prob", 0.48)   # 比 Conservative 再柔和一點
        self.declare_parameter("end_prob", 0.42)
        self.declare_parameter("k_start", 3)
        self.declare_parameter("k_end", 18)
        self.declare_parameter("t_cap_s", 3.0)
        self.declare_parameter("p_min", 0.52)        # 平均語音機率中等門檻
        self.declare_parameter("snr_min_db", 5.0)
        self.declare_parameter("rms_gate", 9)        # 小聲音不計，但比 12 寬鬆
        self.declare_parameter("target_proc_rate", 16000)

        # 權重（w_s 為穩定度）
        self.declare_parameter("w_p", 0.58)          # 主導仍是語音機率
        self.declare_parameter("w_s", 0.05)          # 穩定度稍微給分
        self.declare_parameter("w_c", 0.12)          # 持續時間
        self.declare_parameter("w_o", 0.05)
        # ---- OWW ----
        package_share_dir = get_package_share_directory('kirox_robot')
        default_oww_model_path = os.path.join(package_share_dir, 'models', 'kirox.onnx')
        self.declare_parameter("oww_model_path", default_oww_model_path)
        self.declare_parameter("oww_frame_ms", 80)
        self.declare_parameter("oww_threshold", 0.80)
        self.declare_parameter("oww_enable_speex_ns", False)

        # 取參數
        gp = self.get_parameter
        self.outputs_dir = gp("outputs_dir").value
        self.save_outputs = gp("save_outputs").value
        self.tmp_wav_path = gp("tmp_wav_path").value
        self.input_device_name = gp("input_device_name").value
        _idx = int(gp("input_device_index").value)
        self.input_device_index: Optional[int] = None if _idx < 0 else _idx

        self.frame_ms = int(gp("frame_ms").value)
        self.start_prob = float(gp("start_prob").value)
        self.end_prob   = float(gp("end_prob").value)
        self.k_start    = int(gp("k_start").value)
        self.k_end      = int(gp("k_end").value)
        self.t_cap_s    = float(gp("t_cap_s").value)
        self.p_min      = float(gp("p_min").value)
        self.snr_min_db = float(gp("snr_min_db").value)
        self.rms_gate   = int(gp("rms_gate").value)
        self.target_proc_rate = int(gp("target_proc_rate").value)

        self.w_p = float(gp("w_p").value)
        self.w_s = float(gp("w_s").value)
        self.w_c = float(gp("w_c").value)
        self.w_o = float(gp("w_o").value)

        self.oww_model_path = gp("oww_model_path").value
        self.oww_frame_ms   = int(gp("oww_frame_ms").value)
        self.oww_threshold  = float(gp("oww_threshold").value)
        self.oww_enable_speex_ns = bool(gp("oww_enable_speex_ns").value)

        # 固定常數（不外露）
        self.SNR_MAX_DB = 25.0          # 保留，不再進分數
        self.NOISE_ALPHA_SIL = 0.12      # 只在非語音時更新
        self.MIN_UTT_S = 0.25

        # QoS（latched）
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Topics
        self.pub_start  = self.create_publisher(String, "ears/vad_start", 10)
        self.pub_end    = self.create_publisher(String, "ears/vad_end", 10)
        self.pub_latest = self.create_publisher(String, "ears/latest_audio_meta", latched_qos)
        self.pub_score  = self.create_publisher(String, "ears/vad_score", latched_qos)
        self.pub_oww    = self.create_publisher(Float32, "ears/oww_score", latched_qos)
        self.pub_ready  = self.create_publisher(Bool, "ears/ready", latched_qos)  # ✅ 就緒訊號（latched）

        self._record_enable = True
        self.is_setting_mode = False
        # 訂閱設定模式
        self.create_subscription(Bool, "system/setting_mode", self._on_setting_mode, 10)
        self.create_subscription(Bool, "ears/record_enable", self._on_record_enable, latched_qos)

        # 背景事件 queue
        self._evq: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=200)
        self._timer = self.create_timer(0.05, self._pump_events)

        # VAD 參數
        self.proc_rate = int(self.target_proc_rate)
        self.dtype = "int16"
        self.proc_frame_samples = max(
            int(self.proc_rate * self.frame_ms / 1000),
            int(np.ceil(self.proc_rate / 31.25))
        )  # >=512 @16k

        # Silero
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"[VAD] torch device={device}")
        self.silero_model, _ = torch.hub.load("snakers4/silero-vad", "silero_vad", trust_repo=True)
        self.silero_model.eval().to(device)
        self._torch_device = device

        # OWW
        if not os.path.exists(self.oww_model_path):
            raise FileNotFoundError(f"OWW 模型不存在: {self.oww_model_path}")
        self.oww_model = OWWModel(
            wakeword_models=[self.oww_model_path],
            enable_speex_noise_suppression=self.oww_enable_speex_ns,
            inference_framework="onnx"
        )
        self._oww_block = max(1, int(self.proc_rate * (self.oww_frame_ms / 1000.0)))
        _probe = self.oww_model.predict(np.zeros(self._oww_block, dtype=np.int16))
        if not _probe:
            raise RuntimeError("openWakeWord predict 回傳空字典")
        self._oww_key = next(iter(_probe.keys()))
        self._oww_buf = np.zeros((0,), dtype=np.int16)
        self._oww_seg_max = 0.0
        self._oww_pub_max = 0.0
        self._latest_oww_score = 0.0

        # ---- SpeexDSP 降噪初始化（新增）----
        self.ns = NoiseSuppression.create(frame_size=self.proc_frame_samples)
        self.get_logger().info(f"[SpeexDSP] Noise Suppression init (frame={self.proc_frame_samples})")

        # 音源/執行緒
        self.input_rate, self.input_device = self._pick_input_device()
        self._proc_buf = np.zeros((0,), dtype=np.int16)
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None

        # 狀態
        self._latest_meta: Dict[str, Any] = {}
        self._latest_score: Dict[str, Any] = {}
        self._noise_db_ema: Optional[float] = None
        self._eps = 1e-6

        # 啟動
        self._start_stream()
        self.get_logger().info("[robotears] started")

        # ✅ 啟動完成即發布就緒訊號（latched，晚訂閱者也能收到最後一筆）
        self.pub_ready.publish(Bool(data=True))
        self.get_logger().info("[ready] robotears 啟動完成 (ears/ready=True)")

    # ---------- 錄音開關 ----------
    def _on_setting_mode(self, msg: Bool):
        self.is_setting_mode = msg.data
        self.get_logger().info(f"Setting mode changed to: {self.is_setting_mode}")

    def _on_record_enable(self, msg: Bool):
        prev = self._record_enable
        self._record_enable = bool(msg.data)
        if self._record_enable != prev:
            self.get_logger().info(f"[record_enable] -> {self._record_enable}")
        if not self._record_enable:
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
                blocksize = int(sr * self.proc_frame_samples / self.proc_rate)
                stream = sd.InputStream(
                    samplerate=sr, channels=1, dtype=self.dtype, blocksize=blocksize,
                    callback=self._audio_callback, device=self.input_device
                )
                stream.__enter__()
                self.input_rate = sr
                return stream
            except Exception:
                continue
        raise RuntimeError("No valid input sample rate for selected device")

    def _audio_callback(self, indata, frames, time_info, status):
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

        # 前置緩衝區（觸發前片段）
        pre_buffer = deque(maxlen=self.k_start + 2)

        # 句內統計
        p_sum = 0.0
        # snr_n_sum = 0.0            # ★ 不再用於打分（保留原變數名稱注釋）
        frame_cnt = 0
        onset_flag = 0

        # 新增：RMS 歷史（穩定度用）
        rms_hist = deque(maxlen=8)

        # 效能優化：預先分配 Tensor
        frame_f32 = np.zeros(self.proc_frame_samples, dtype=np.float32)
        frame_tensor = torch.from_numpy(frame_f32).to(self._torch_device)

        while not self._stop.is_set():
            try:
                ev = self._evq.get(timeout=0.1)
            except queue.Empty:
                continue

            if ev["kind"] != "audio_chunk":
                self._evq.put(ev)
                continue

            # 若不在錄音或設定模式，重置並跳過
            if not self._record_enable or self.is_setting_mode:
                pre_buffer.clear()
                in_speech = False
                voiced_count = unvoiced_count = 0
                seg_samples = []
                p_sum = 0.0
                # snr_n_sum = 0.0
                frame_cnt = 0
                onset_flag = 0
                rms_hist.clear()
                continue

            chunk_in: np.ndarray = ev["data"]
            chunk_proc = _resample_linear(chunk_in, self.input_rate, self.proc_rate)

            if self._proc_buf.size > 0:
                chunk_proc = np.concatenate([self._proc_buf, chunk_proc], axis=0)
                self._proc_buf = np.zeros((0,), dtype=np.int16)

            offset = 0
            while offset + self.proc_frame_samples <= len(chunk_proc):
                frame_i16 = chunk_proc[offset:offset + self.proc_frame_samples]
                offset += self.proc_frame_samples

                # ---- ★ 先做 SpeexDSP 降噪（新增）----
                frame_i16 = self.ns.process(frame_i16.tobytes())
                frame_i16 = np.frombuffer(frame_i16, dtype=np.int16)

                # ---- VAD：RMS + Silero ----
                # 效能優化：np.dot 代替 mean(x**2)
                frame_as_float = frame_i16.astype(np.float32)
                rms = float(np.sqrt(np.dot(frame_as_float, frame_as_float) / len(frame_as_float)) + self._eps)
                L_db = 20.0 * np.log10(rms + self._eps)

                # 新增：累積 RMS(dB) 用於句尾穩定度
                rms_hist.append(L_db)

                if rms < self.rms_gate:
                    speech_prob = 0.0
                else:
                    # 使用預分配 Tensor
                    np.divide(frame_as_float, 32768.0, out=frame_f32)
                    frame_tensor.copy_(torch.from_numpy(frame_f32))
                    with torch.no_grad():
                        speech_prob = float(self.silero_model(frame_tensor, self.proc_rate).item())

                # ---- 噪聲 EMA（非語音時更新）----
                if self._noise_db_ema is None:
                    self._noise_db_ema = L_db

                snr_db_tmp = max(L_db - (self._noise_db_ema or L_db), 0.0)
                snr_n = min(snr_db_tmp, self.SNR_MAX_DB) / self.SNR_MAX_DB  # 0~1（保留計算但不進打分）
                is_speech = speech_prob >= (self.start_prob if not in_speech else self.end_prob)

                if not is_speech:
                    a = self.NOISE_ALPHA_SIL
                    self._noise_db_ema = (1 - a) * (self._noise_db_ema if self._noise_db_ema is not None else L_db) + a * L_db
                    snr_db_tmp = max(L_db - (self._noise_db_ema or L_db), 0.0)
                    snr_n = min(snr_db_tmp, self.SNR_MAX_DB) / self.SNR_MAX_DB

                # 無論是否在說話，都加入前置緩衝
                pre_buffer.append(frame_i16)

                if not in_speech:
                    if is_speech:
                        voiced_count += 1
                        if voiced_count >= self.k_start:
                            in_speech = True
                            seg_samples = list(pre_buffer)
                            seg_start_t = time.time()
                            unvoiced_count = 0
                            voiced_count = 0
                            onset_flag = 1
                            p_sum = 0.0
                            # snr_n_sum = 0.0
                            frame_cnt = 0
                            # OWW 段內峰值重置
                            self._oww_seg_max = 0.0
                            self._oww_pub_max = 0.0
                            pre_buffer.clear()
                            self._evq.put({"kind": "vad_start", "ts": seg_start_t})
                    else:
                        voiced_count = 0
                else:
                    # 句內累計
                    p_sum += speech_prob
                    # snr_n_sum += snr_n  # 不再用於打分
                    frame_cnt += 1
                    seg_samples.append(frame_i16.copy())

                    if is_speech:
                        unvoiced_count = 0
                    else:
                        unvoiced_count += 1

                    end_by_silence = (unvoiced_count >= self.k_end)
                    end_by_length = ((time.time() - seg_start_t) >= 30.0)

                    if end_by_silence or end_by_length:
                        dur = time.time() - seg_start_t
                        pcm = np.concatenate(seg_samples, axis=0) if seg_samples else np.zeros((0,), dtype=np.int16)
                        why = "silence" if end_by_silence else "max_len"

                        # 保存音檔
                        try:
                            _atomic_write_wav(self.tmp_wav_path, pcm, self.proc_rate)
                        except Exception as e:
                            self.get_logger().error(f"[save] 寫入 RAM 失敗: {e}")

                        archive_path = ""
                        if self.save_outputs:
                            try:
                                archive_path = _new_archive_path(self.outputs_dir, "seg")
                                shutil.copyfile(self.tmp_wav_path, archive_path)
                            except Exception as e:
                                self.get_logger().warn(f"[save] 複製 outputs 失敗: {e}")
                                archive_path = ""

                        sha = _sha256_file(self.tmp_wav_path) or ""
                        end_ts = time.time()
                        meta = {
                            "path": self.tmp_wav_path, "ts": end_ts, "sr": self.proc_rate,
                            "duration": float(dur), "reason": why, "sha256": sha, "archive_path": archive_path,
                        }
                        self._evq.put({"kind": "vad_end", **meta})

                        # ===== 句尾打分 =====
                        mean_p = (p_sum / max(frame_cnt, 1)) if frame_cnt > 0 else 0.0
                        # mean_snr_n = (snr_n_sum / max(frame_cnt, 1)) if frame_cnt > 0 else 0.0
                        # mean_snr_db = mean_snr_n * self.SNR_MAX_DB

                        cont = min(dur / self.t_cap_s, 1.0)
                        onset = 1 if onset_flag else 0

                        if mean_p <= self.p_min:
                            p_eff = 0.0
                        else:
                            p_eff = float(np.sqrt(min((mean_p - self.p_min) / (1.0 - self.p_min), 1.0)))

                        # ★ 新增：以 RMS 穩定度替代 SNR 權重
                        rms_var = float(np.std(rms_hist)) if len(rms_hist) > 2 else 0.0  # dB 標準差
                        stability = float(np.clip(1.0 - (rms_var / 20.0), 0.0, 1.0))     # 20 dB 縮放

                        score_raw = (self.w_p * p_eff +
                                     self.w_s * stability +   # w_s 現在代表穩定度
                                     self.w_c * cont +
                                     self.w_o * onset)

                        # 移除舊 SNR 懲罰，直接裁剪
                        score = 0.0 if dur < self.MIN_UTT_S else float(np.clip(score_raw, 0.0, 1.0))

                        tau_s = self.k_end * (self.frame_ms / 1000.0)
                        score_msg = {
                            "ts": end_ts,
                            "score": score,
                            "duration": float(dur),
                            "tau_s": float(tau_s),
                            "components": {
                                "mean_p": float(mean_p),
                                "p_eff": float(p_eff),
                                "stability": float(stability),
                                "cont": float(cont),
                                "onset": int(onset),
                                # 兼容保留（不再使用）：可視需要填 None 或不給
                                # "mean_snr_db": None,
                                # "snr_eff": None,
                            },
                            "weights": {
                                "w_p": float(self.w_p), "w_s": float(self.w_s),
                                "w_c": float(self.w_c), "w_o": float(self.w_o),
                            },
                            "thresholds": {
                                "p_min": float(self.p_min),
                                "snr_min_db": float(self.snr_min_db),   # 保留顯示
                                "snr_max_db": float(self.SNR_MAX_DB),   # 保留顯示
                                "min_utt_s": float(self.MIN_UTT_S),
                            },
                            "reason": why,
                        }

                        self.get_logger().info(
                            f"[score] {score:.3f} | dur={dur:.2f}s, "
                            f"mean_p={mean_p:.2f}, stability={stability:.2f}, "
                            f"cont={cont:.2f}, onset={onset}, end='{why}'"
                        )

                        self._set_latest_score(score_msg)

                        # Reset
                        in_speech = False
                        voiced_count = 0
                        unvoiced_count = 0
                        seg_samples = []
                        p_sum = 0.0
                        # snr_n_sum = 0.0
                        frame_cnt = 0
                        onset_flag = 0
                        pre_buffer.clear()
                        rms_hist.clear()

                # ---- OWW：段內峰值 ----
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
            if ev.get("kind") == "vad_start":
                self._on_vad_start(ev)
            elif ev.get("kind") == "vad_end":
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
        self.pub_end.publish(String(data=json.dumps({"event": "end", **meta}, ensure_ascii=False)))
        self._publish_latest(meta)
        self.get_logger().info(f"[latest] {meta['path']} ({meta['duration']:.2f}s)")
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
