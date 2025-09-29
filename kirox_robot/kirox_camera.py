# kirox_camera_node.py
# -*- coding: utf-8 -*-
"""
Kirox 簡易相機節點
- 用 OpenCV 讀取攝影機（/dev/videoX 或整數索引）
- 發布彩色影像到 rgb_topic (sensor_msgs/msg/Image, bgr8)
- 可選視窗預覽
相依：opencv-python, cv_bridge, rclpy
"""

import time
from typing import Optional, Union

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class KiroxCameraNode(Node):
    def __init__(self):
        super().__init__("kirox_camera")

        # ---------- 參數 ----------
        self.declare_parameter(
            "device", "/dev/video0",
            ParameterDescriptor(description="攝影機來源：/dev/videoX 或整數索引（0,1,2...）。")
        )
        self.declare_parameter(
            "width", 1280, ParameterDescriptor(description="影像寬度（像素）。")
        )
        self.declare_parameter(
            "height", 720, ParameterDescriptor(description="影像高度（像素）。")
        )
        self.declare_parameter(
            "fps", 30.0, ParameterDescriptor(description="目標幀率。")
        )
        self.declare_parameter(
            "fourcc", "MJPG",
            ParameterDescriptor(description="相機輸出編碼（常見：MJPG / YUYV）。")
        )
        self.declare_parameter(
            "rgb_topic", "/camera/color/image_raw",
            ParameterDescriptor(description="輸出影像的 topic（sensor_msgs/Image, bgr8）。")
        )
        self.declare_parameter(
            "frame_id", "camera_frame",
            ParameterDescriptor(description="header.frame_id。")
        )
        self.declare_parameter(
            "show_window", False,
            ParameterDescriptor(description="是否顯示 OpenCV 視窗預覽。")
        )
        self.declare_parameter(
            "window_name", "Kirox Camera",
            ParameterDescriptor(description="預覽視窗名稱。")
        )
        self.declare_parameter(
            "auto_reopen", True,
            ParameterDescriptor(description="讀取失敗時是否自動重開相機。")
        )
        self.declare_parameter(
            "reopen_interval_sec", 3.0,
            ParameterDescriptor(description="自動重開最小間隔（秒）。")
        )

        # ---------- 讀參數 ----------
        self.device_param: str = self.get_parameter("device").get_parameter_value().string_value or "/dev/video0"
        self.width: int = int(self.get_parameter("width").get_parameter_value().integer_value or 1280)
        self.height: int = int(self.get_parameter("height").get_parameter_value().integer_value or 720)
        self.fps: float = float(self.get_parameter("fps").get_parameter_value().double_value or 30.0)
        self.fourcc: str = self.get_parameter("fourcc").get_parameter_value().string_value or "MJPG"
        self.rgb_topic: str = self.get_parameter("rgb_topic").get_parameter_value().string_value or "/camera/color/image_raw"
        self.frame_id: str = self.get_parameter("frame_id").get_parameter_value().string_value or "camera_frame"
        self.show_window: bool = bool(self.get_parameter("show_window").get_parameter_value().bool_value)
        self.window_name: str = self.get_parameter("window_name").get_parameter_value().string_value or "Kirox Camera"
        self.auto_reopen: bool = bool(self.get_parameter("auto_reopen").get_parameter_value().bool_value)
        self.reopen_interval_sec: float = float(self.get_parameter("reopen_interval_sec").get_parameter_value().double_value or 3.0)

        # ---------- Publisher / Bridge ----------
        self.pub_img = self.create_publisher(Image, self.rgb_topic, qos_profile_sensor_data)
        self.bridge = CvBridge()

        # ---------- 打開相機 ----------
        self.cap: Optional[cv2.VideoCapture] = None
        self._last_open_t: float = 0.0
        self._open_camera(initial=True)

        # ---------- 計時器（依 fps 發布）----------
        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Kirox camera started -> device={self.device_param} "
            f"-> topic={self.rgb_topic}  size={self.width}x{self.height}  fps={self.fps}"
        )

    # ========== 相機控制 ==========
    def _parse_device(self, dev: str) -> Union[int, str]:
        # 允許傳 0/1/2 或 '/dev/video0'
        try:
            return int(dev)
        except Exception:
            return dev

    def _open_camera(self, initial: bool = False):
        now = time.monotonic()
        if (not initial) and (now - self._last_open_t < self.reopen_interval_sec):
            return  # 避免過度重試
        self._last_open_t = now

        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

        dev = self._parse_device(self.device_param)

        # 在 Linux 用 CAP_V4L2 會較穩
        api_pref = cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else 0
        self.cap = cv2.VideoCapture(dev, api_pref)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera: {self.device_param}")
            return

        # 設定解析度/FPS/FOURCC（能否成功視相機而定）
        self._try_set(self.cap, cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self._try_set(self.cap, cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self._try_set(self.cap, cv2.CAP_PROP_FPS, float(self.fps))
        if self.fourcc:
            fourcc_val = cv2.VideoWriter_fourcc(*self.fourcc)
            self._try_set(self.cap, cv2.CAP_PROP_FOURCC, float(fourcc_val))

        # 讀回實際生效值
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS))
        self.get_logger().info(f"Camera opened: {self.device_param} -> {actual_w}x{actual_h} @ {actual_fps:.1f} ({self.fourcc})")

    def _try_set(self, cap: cv2.VideoCapture, prop: int, val: float):
        try:
            cap.set(prop, val)
        except Exception:
            pass

    # ========== 定時拍/發 ==========
    def _on_timer(self):
        if self.cap is None or not self.cap.isOpened():
            if self.auto_reopen:
                self._open_camera()
            self._show_waiting_window()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Camera read failed.")
            if self.auto_reopen:
                self._open_camera()
            self._show_waiting_window()
            return

        # 有些驅動可能回傳非 BGR8；通常 OpenCV 會轉好，但保險起見做型別檢查
        if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            try:
                frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
            except Exception:
                frame = np.ascontiguousarray(frame).astype(np.uint8)
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # 尺寸與參數不符時可選擇 resize（保證輸出大小）
        if (frame.shape[1] != self.width) or (frame.shape[0] != self.height):
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

        # 組 ROS Image 訊息
        msg: Image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # 發布
        self.pub_img.publish(msg)

        # 視窗
        if self.show_window:
            try:
                cv2.imshow(self.window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info("Window closed by 'q'")
                    rclpy.shutdown()
            except Exception as e:
                self.get_logger().warn(f"imshow failed: {e}")

    def _show_waiting_window(self):
        if not self.show_window:
            return
        h, w = int(self.height), int(self.width)
        frame = np.zeros((max(240, h // 2), max(320, w // 2), 3), dtype=np.uint8)
        txt = f"Opening camera: {self.device_param}"
        cv2.putText(frame, txt, (20, frame.shape[0] // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2, cv2.LINE_AA)
        try:
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"imshow failed: {e}")

    # ========== 結束 ==========
    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        try:
            if self.show_window:
                cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = KiroxCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
