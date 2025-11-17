#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
distance_detect.py — ROS2 距離偵測節點
- 透過 serial 讀取距離感測器資料（Modbus CRC）
- 背景執行緒負責不斷從 serial 收資料、解析距離
- 每 0.2 秒將最新距離值 publish 到 body/action (std_msgs/String)
- 資料格式：{"type": "distance", "content": "1234"}

依賴：
    pip install pyserial
"""

import json
import threading
import time
from typing import Optional

import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ---------- CRC / 封包相關 ----------

def crc16_modbus(data: bytes) -> bytes:
    """
    計算 Modbus CRC16
    回傳：2 bytes (高位在前，低位在後)
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([(crc >> 8) & 0xFF, crc & 0xFF])


def build_packet(cmd: int) -> bytes:
    """
    建立啟動 / 停止封包
    cmd: 0x01 = start, 0x02 = stop
    body 格式：A5 03 20 <cmd> 00 00 00 + CRC16
    """
    body = bytes([0xA5, 0x03, 0x20, cmd, 0x00, 0x00, 0x00])
    return body + crc16_modbus(body)


PKT_START = build_packet(0x01)
PKT_STOP = build_packet(0x02)


def find_port() -> Optional[str]:
    """
    嘗試自動尋找 /dev/ttyUSBx 或 /dev/ttyACMx
    找不到就回傳 None
    """
    for p in serial.tools.list_ports.comports():
        if "USB" in p.device or "ACM" in p.device:
            return p.device
    return None


def parse_distance(frame: bytes) -> Optional[int]:
    """
    從一個完整 frame 解析距離值
    - 延續原本的邏輯：
        * frame 長度至少 23
        * frame[3] == 0x01 才視為距離資料
        * data 區從 frame[7:-2]
        * 距離 = data[6] + (data[7] << 8)，單位 mm
    """
    if len(frame) < 23 or frame[3] != 0x01:
        return None

    data = frame[7:-2]
    if len(data) < 8:
        return None

    dist = data[6] | (data[7] << 8)
    return dist


# ---------- ROS2 節點實作 ----------

class DistanceDetectNode(Node):
    """
    ROS2 節點：
    - 背景執行緒：處理 serial 連線與資料解析
    - Timer(0.2s)：將最新距離值以 JSON 字串形式 publish 到 body/action
    """

    def __init__(self):
        super().__init__('distance_detect')

        
        # 參數：可在 launch / CLI 覆寫
        self.declare_parameter('port', '/dev/ttyCH341USB0')
        self.declare_parameter('baud', 921600)

        self.port_param = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value

        # Publisher
        self.pub = self.create_publisher(String, 'body/action', 10)

        # 儲存最新距離值（mm），None 代表尚未有資料
        self.latest_distance: Optional[int] = None

        # 控制 serial 執行緒
        self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._running = True

        # 找不到 port 的 warning 節流：上一次印警告的時間
        self._last_no_port_warn_time = 0.0

        # 每 0.2 秒 publish 一次
        self.timer = self.create_timer(0.2, self._timer_publish)

        self.get_logger().info('distance_detect 節點啟動中...')
        self._serial_thread.start()
        self.isdebug = True

    # ----- ROS2 Timer：定期發佈 -----

    def _timer_publish(self):
        """
        每 0.2 秒呼叫一次，將最新距離值以 JSON 格式發佈：
        {"type": "distance", "content": "<距離mm>"}
        """
        if self.latest_distance is None:
            return

        payload = {
            "type": "distance",
            "content": str(self.latest_distance),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        # self.get_logger().info(f'{msg}')
        self.pub.publish(msg)

    # ----- Serial 開啟 -----

    def _open_serial(self) -> Optional[serial.Serial]:
        """
        嘗試打開 serial 連線：
        - 如果有指定 port 參數就用那個
        - 否則自動 find_port()
        找不到時會節流 warning，只偶爾印一次
        """
        port = self.port_param or find_port()
        if not port:
            now = time.time()
            # 每 10 秒才印一次 warning，其餘時間安靜重試
            if now - self._last_no_port_warn_time > 10.0:
                if self.isdebug:
                    self.get_logger().warn('找不到可用的 /dev/ttyUSB* 或 /dev/ttyACM*，持續重試中...')
                self._last_no_port_warn_time = now
            return None

        try:
            ser = serial.Serial(port, baudrate=self.baud, timeout=0.05)
            self.get_logger().info(f'已連線到序列埠：{port} @ {self.baud} baud')
            time.sleep(0.2)
            # 送啟動封包
            try:
                ser.write(PKT_START)
                self.get_logger().info('已送出 PKT_START')
            except Exception as e:
                self.get_logger().error(f'送出 PKT_START 失敗: {e}')
            return ser
        except Exception as e:
            self.get_logger().error(f'打開序列埠失敗: {e}')
            return None

    # ----- Serial 執行緒 -----

    def _serial_loop(self):
        """
        持續執行的 serial 讀取迴圈：
        - 自動重連
        - 解析封包、更新 latest_distance
        """
        ser: Optional[serial.Serial] = None
        buf = b''

        while self._running:
            # 確保 serial 已連線
            if ser is None or not ser.is_open:
                ser = self._open_serial()
                buf = b''
                time.sleep(1.0)
                continue

            try:
                # 一次讀一些 bytes
                chunk = ser.read(256)
                if not chunk:
                    # timeout，沒有資料也沒關係，繼續 loop
                    continue

                buf += chunk

                # 嘗試從 buffer 中取出完整 frame
                while True:
                    # 至少要有 header 7 bytes (A5 ... lenH lenL) + 一點空間
                    if len(buf) < 9:
                        break

                    # 找 sync 頭 (0xA5)
                    if buf[0] != 0xA5:
                        idx = buf.find(b'\xA5')
                        if idx >= 0:
                            buf = buf[idx:]
                        else:
                            buf = b''
                        break

                    # 長度資訊需要至少前 7 bytes
                    if len(buf) < 7:
                        break

                    data_len = (buf[5] << 8) | buf[6]
                    total_len = 7 + data_len + 2  # header + data + CRC

                    if len(buf) < total_len:
                        # frame 尚未完整
                        break

                    frame = buf[:total_len]
                    buf = buf[total_len:]

                    # 檢查 CRC
                    if crc16_modbus(frame[:-2]) != frame[-2:]:
                        self.get_logger().warn('CRC 錯誤，丟棄一帧資料')
                        continue

                    # 解析距離
                    dist = parse_distance(frame)
                    if dist is not None:
                        self.latest_distance = dist

            except Exception as e:
                self.get_logger().error(f'Serial 讀取錯誤: {e}')
                # 嘗試關閉並重連
                try:
                    if ser is not None and ser.is_open:
                        try:
                            ser.write(PKT_STOP)
                        except Exception:
                            pass
                        ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(1.0)

        # 結束前嘗試送停止封包
        if ser is not None and ser.is_open:
            try:
                ser.write(PKT_STOP)
            except Exception:
                pass
            ser.close()
            self.get_logger().info('Serial 已關閉')

    # ----- 清理 -----

    def destroy_node(self):
        """
        覆寫 destroy_node，在節點被關閉時結束 serial 執行緒
        """
        self._running = False
        if self._serial_thread.is_alive():
            self._serial_thread.join(timeout=2.0)
        super().destroy_node()


# ---------- main ----------

def main(args=None):
    rclpy.init(args=args)
    node = DistanceDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
