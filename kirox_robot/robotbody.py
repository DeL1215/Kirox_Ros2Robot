#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading


class RobotBodyNode(Node):
    """
    RobotBodyNode — ROS2 ↔ Arduino 雙向通訊節點
    - 訂閱主題 'body/action'：將指令轉發給 Arduino
    - 監聽 Arduino 回傳資料並列印於 ROS2 log
    """

    def __init__(self):
        super().__init__('robot_body_node')

        # 宣告參數
        self.declare_parameter('port', '/dev/ttyCH341USB0')
        self.declare_parameter('baud_rate', 115200)

        # 取得參數值
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.serial_port = None
        self._connection_warning_logged = False
        self.connect_serial()

        # 訂閱 body/action
        self.subscription = self.create_subscription(
            String,
            'body/action',
            self.action_callback,
            10
        )
        self.get_logger().info("🟢 Robot Body Node 已啟動，監聽 'body/action' 主題中...")

        # 每 5 秒嘗試重新連線
        self.reconnect_timer = self.create_timer(5.0, self.connect_serial)

        # 背景執行緒：讀取 Arduino 回傳資料
        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.read_thread.start()

    # -------------------------------------------------
    # 序列埠連線
    # -------------------------------------------------
    def connect_serial(self):
        if self.serial_port and self.serial_port.is_open:
            if self._connection_warning_logged:
                self.get_logger().info(f"✅ 已重新連上序列埠 {self.port}")
                self._connection_warning_logged = False
            return

        try:
            if not self._connection_warning_logged:
                self.get_logger().info(f"嘗試連線序列埠 {self.port} (baud={self.baud_rate})...")

            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=0.2)
            time.sleep(2)
            self.get_logger().info(f"✅ 成功連線序列埠 {self.port}")
            self._connection_warning_logged = False
        except serial.SerialException as e:
            self.serial_port = None
            if not self._connection_warning_logged:
                self.get_logger().warn(f"⚠️ 無法連線到 {self.port}: {e}，將持續重試。")
                self._connection_warning_logged = True

    # -------------------------------------------------
    # 處理 ROS2 → Arduino 的訊息
    # -------------------------------------------------
    def action_callback(self, msg):
        """
        當從 ROS 主題 /body/action 收到訊息時：
        - 印出 action 內容
        - 轉送給 Arduino
        """
        command = msg.data.strip()
        if not command:
            return

        # 🔸 新增明確 log 顯示收到的 ROS action
        self.get_logger().info(f"🎯 收到 ROS 指令: '{command}'")

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write((command + '\n').encode('utf-8'))
                self.get_logger().info(f"➡️ 已發送到 Arduino: '{command}'")
            except serial.SerialException as e:
                self.get_logger().error(f"❌ 寫入錯誤: {e}")
                self.serial_port.close()
                self.serial_port = None
        else:
            self.get_logger().warn("⚠️ 尚未連線 Arduino，指令未送出。")

    # -------------------------------------------------
    # 讀取 Arduino → ROS2 資料
    # -------------------------------------------------
    def read_from_serial(self):
        """
        持續讀取 Arduino 回傳資料，並在 ROS2 console 中印出。
        """
        while True:
            if self.serial_port and self.serial_port.is_open:
                try:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # 🔸 清楚顯示來自 Arduino 的訊息
                        self.get_logger().info(f"📥 來自 Arduino: {line}")
                except serial.SerialException as e:
                    self.get_logger().error(f"❌ 讀取錯誤: {e}")
                    try:
                        self.serial_port.close()
                    except Exception:
                        pass
                    self.serial_port = None
            time.sleep(0.05)

    # -------------------------------------------------
    # 關閉節點
    # -------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("🛑 關閉 Robot Body Node...")
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("🔌 序列埠已關閉。")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBodyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
