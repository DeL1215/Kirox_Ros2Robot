import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class RobotBodyNode(Node):
    """
    負責接收 'body/action' 主題的指令，並透過序列埠轉發給 Arduino。
    具備自動重連和安全保護機制。
    """
    def __init__(self):
        """
        節點初始化
        """
        super().__init__('robot_body_node')

        # 宣告參數，允許從外部配置序列埠和鮑率
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)

        # 獲取參數值
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.serial_port = None
        self._connection_warning_logged = False  # 用於追蹤是否已顯示過連線警告
        self.connect_serial()

        # 建立訂閱者，監聽 'body/action' 主題
        self.subscription = self.create_subscription(
            String,
            'body/action',
            self.action_callback,
            10)
        self.get_logger().info(f"Robot Body Node 已啟動，正在監聽 'body/action'...")

        # 建立一個計時器，用於定期檢查和重新連線序列埠
        self.reconnect_timer = self.create_timer(5.0, self.connect_serial)

    def connect_serial(self):
        """
        嘗試連接到序列埠。如果已連接則不做任何事。
        """
        if self.serial_port and self.serial_port.is_open:
            # 如果之前有顯示過警告，現在連線成功了，就印出成功訊息並重置旗標
            if self._connection_warning_logged:
                self.get_logger().info(f"重新成功連接到序列埠 {self.port}")
                self._connection_warning_logged = False
            return # 已經連線，無需操作

        try:
            # 只有在尚未顯示警告時，才印出「正在嘗試」的訊息
            if not self._connection_warning_logged:
                self.get_logger().info(f"正在嘗試連接序列埠 {self.port} (鮑率: {self.baud_rate})...")

            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            # 等待 Arduino 重啟 (DTR/RTS 會觸發重啟)
            time.sleep(2) 
            self.get_logger().info(f"成功連接到序列埠 {self.port}")
            self._connection_warning_logged = False # 連線成功，重置旗標
        except serial.SerialException as e:
            self.serial_port = None
            if not self._connection_warning_logged:
                self.get_logger().warn(f"無法連接到序列埠 {self.port}: {e}。將在背景持續重試。")
                self._connection_warning_logged = True # 標記已顯示過警告

    def action_callback(self, msg):
        """
        當收到 'body/action' 的訊息時，此回呼函式會被觸發。
        """
        command = msg.data
        self.get_logger().info(f"收到指令: '{command}'")

        # 安全保護：檢查序列埠是否成功連接
        if self.serial_port and self.serial_port.is_open:
            try:
                # 加上換行符，方便 Arduino 端的 readStringUntil('\n')
                data_to_send = (command + '\n').encode('utf-8')
                self.serial_port.write(data_to_send)
                self.get_logger().info(f"已發送指令到 Arduino: '{command}'")
            except serial.SerialException as e:
                self.get_logger().error(f"寫入序列埠時發生錯誤: {e}")
                # 發生錯誤時，關閉連接，等待計時器重連
                if self.serial_port:
                    self.serial_port.close()
                self.serial_port = None
        else:
            self.get_logger().warn("序列埠未連接，無法發送指令。")

    def destroy_node(self):
        """
        節點關閉時的清理工作
        """
        self.get_logger().info("正在關閉 Robot Body Node...")
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("序列埠已關閉。")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    robot_body_node = RobotBodyNode()
    try:
        rclpy.spin(robot_body_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 節點銷毀與資源清理
        robot_body_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
