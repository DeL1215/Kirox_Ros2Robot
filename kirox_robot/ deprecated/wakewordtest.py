#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
openWakeWord 麥克風串流偵測（修正 vad_threshold=None 的 TypeError）
- 80ms 幀、16kHz、int16
- 使用 default/pulse，避免裝置不支援 16kHz 的錯誤
- 自動探測 scores 的 key 名稱
"""

import os
import time
import numpy as np
import sounddevice as sd
from ament_index_python.packages import get_package_share_directory
from openwakeword.model import Model

# ===== 參數 =====
package_share_dir = get_package_share_directory('kirox_robot')
MODEL_PATH = os.path.join(package_share_dir, 'models', 'kirox.onnx')
DEVICE = "default"      # 或改成 13 -> pulse
RATE = 16000
FRAME_MS = 80
BLOCK = int(RATE * (FRAME_MS / 1000.0))
THRESH = 0.5
DEBOUNCE_N = 2

# ===== 檢查模型 =====
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"模型不存在: {MODEL_PATH}")
print("[info] 使用模型:", MODEL_PATH)

# ===== 初始化模型（不要傳 vad_threshold=None）=====
model = Model(
    wakeword_models=[MODEL_PATH],
    enable_speex_noise_suppression=False  # 需要可改 True（前提：有 speexdsp_ns）
    # 不要傳 vad_threshold 參數，舊版會拿 None 去比較而爆掉
)

# 用一段靜音探測實際的 score key
probe = model.predict(np.zeros(BLOCK, dtype=np.int16))
if not probe:
    raise RuntimeError("模型載入失敗：predict 回傳空字典")
KEY = next(iter(probe.keys()))
print("[info] 使用 score key:", KEY)

# ===== 觸發狀態 =====
above_cnt = 0
max_score = 0.0
last_print = 0.0

def callback(indata, frames, t, status):
    global above_cnt, max_score, last_print
    if status:
        print("SD status:", status)

    # RawInputStream(dtype='int16') 會給 bytes；轉成 int16 向量
    data = np.frombuffer(indata, dtype=np.int16)

    scores = model.predict(data)
    s = float(scores.get(KEY, 0.0))
    if s > max_score:
        max_score = s

    now = time.time()
    if now - last_print > 0.5:
        last_print = now
        print(f"score={s:.3f}, max={max_score:.3f}")

    if s > THRESH:
        above_cnt += 1
        if above_cnt >= DEBOUNCE_N:
            print(f"[TRIGGER] {KEY} score={s:.3f}")
            above_cnt = 0
    else:
        if above_cnt > 0:
            above_cnt -= 1

print(f"[audio] 開始偵測（請說：hey jarvis） device={DEVICE} rate={RATE} block={BLOCK}")

try:
    with sd.RawInputStream(
        device=DEVICE,
        channels=1,
        samplerate=RATE,
        blocksize=BLOCK,
        dtype="int16",
        callback=callback
    ):
        while True:
            sd.sleep(1000)
except KeyboardInterrupt:
    print("結束偵測。")
