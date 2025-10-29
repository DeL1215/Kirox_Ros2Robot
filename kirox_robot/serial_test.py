#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson ↔ Arduino USB 串列測試
- 自動偵測 Arduino 或 CH340/CH341 的序列埠
- 支援 /dev/ttyACM*、/dev/ttyUSB*、/dev/ttyCH341USB*
- 波特率：9600
"""

import serial
import time
import os

# 嘗試自動偵測
PORT = None
CANDIDATES = [
    "/dev/ttyACM0", "/dev/ttyACM1",
    "/dev/ttyUSB0", "/dev/ttyUSB1",
    "/dev/ttyCH341USB0", "/dev/ttyCH341USB1"
]

for dev in CANDIDATES:
    if os.path.exists(dev):
        PORT = dev
        break

if PORT is None:
    print("❌ 找不到 Arduino 裝置 (/dev/ttyACM*、/dev/ttyUSB*、或 /dev/ttyCH341USB*)")
    exit(1)

BAUD = 9600

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"✅ 已開啟 {PORT}（baud={BAUD}）")
except Exception as e:
    print(f"❌ 開啟失敗：{e}")
    exit(1)

try:
    while True:
        msg = "Hello Arduino!"
        ser.write((msg + "\n").encode())
        print(f"→ 傳送: {msg}")

        # 嘗試接收 Arduino 回覆
        data = ser.readline().decode(errors="ignore").strip()
        if data:
            print(f"← 收到回覆: {data}")

        time.sleep(3)

except KeyboardInterrupt:
    print("\n🛑 結束測試")
finally:
    ser.close()
    print("🔒 串列埠已關閉")
