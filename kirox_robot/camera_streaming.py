#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_streaming.py — Kirox 相機串流上傳節點

功能：
- 訂閱 KiroxCameraNode 發布的影像 topic（sensor_msgs/msg/Image, bgr8）
- 使用 cv_bridge 轉成 OpenCV 影像，再壓成 JPEG
- 透過 WebSocket 持續上傳 JPEG frame 到後端 API
  目標端點：wss://api.xbotworks.com/api/v1/camera/upload/ws?robot_id=...&camera_id=...

相依套件：
- rclpy
- sensor_msgs
- cv_bridge
- opencv-python
- websocket-client

使用方式（範例）：
    ros2 run kirox_robot camera_streaming \
        --ros-args \
        -p robot_id:=31517165-19e5-44d5-8562-350cb071d1ae \
        -p camera_id:=default \
        -p rgb_topic:=/camera/color/image_raw \
        -p api_ws_url:=wss://api.xbotworks.com/api/v1/camera/upload/ws \
        -p jpeg_quality:=60 \
        -p max_fps:=5.0
"""

import time
import threading
import queue
import socket
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import websocket
from websocket import WebSocketConnectionClosedException


class CameraStreamingNode(Node):
    def __init__(self):
        super().__init__("kirox_camera_streaming")

        # ================== 參數宣告 ==================
        self.declare_parameter(
            "robot_id",
            "31517165-19e5-44d5-8562-350cb071d1ae",
            ParameterDescriptor(description="機器人唯一 ID，用來對應後端與 WebView。"),
        )
        self.declare_parameter(
            "camera_id",
            "default",
            ParameterDescriptor(description="相機 ID，例如 front / top / head，預設 default。"),
        )
        self.declare_parameter(
            "rgb_topic",
            "/camera/color/image_raw",
            ParameterDescriptor(description="要訂閱的彩色影像 topic（sensor_msgs/Image, bgr8）。"),
        )
        self.declare_parameter(
            "api_ws_url",
            "wss://api.xbotworks.com/api/v1/camera/upload/ws",
            ParameterDescriptor(description="後端 WebSocket 上傳端點（不含 query string）。"),
        )
        self.declare_parameter(
            "jpeg_quality",
            60,
            ParameterDescriptor(description="JPEG 影像壓縮品質（1~100，越高越清晰但檔案越大）。"),
        )
        self.declare_parameter(
            "max_fps",
            5.0,
            ParameterDescriptor(description="上傳的最大幀率（避免佔滿頻寬與 CPU）。"),
        )
        self.declare_parameter(
            "queue_size",
            5,
            ParameterDescriptor(description="影像佇列大小；滿了時新影像會直接丟掉。"),
        )
        self.declare_parameter(
            "reconnect_interval_sec",
            5.0,
            ParameterDescriptor(description="WebSocket 斷線後多久再嘗試重連（秒）。"),
        )

        # ================== 讀取參數 ==================
        self.robot_id: str = (
            self.get_parameter("robot_id").get_parameter_value().string_value or "kirox-unknown"
        )
        self.camera_id: str = (
            self.get_parameter("camera_id").get_parameter_value().string_value or "default"
        )
        self.rgb_topic: str = (
            self.get_parameter("rgb_topic").get_parameter_value().string_value
            or "/camera/color/image_raw"
        )
        self.api_ws_url: str = (
            self.get_parameter("api_ws_url").get_parameter_value().string_value
            or "wss://api.xbotworks.com/api/v1/camera/upload/ws"
        )
        self.jpeg_quality: int = int(
            self.get_parameter("jpeg_quality").get_parameter_value().integer_value or 60
        )

        # max_fps 在 ROS 參數裡是 double，所以用 double_value 取
        self.max_fps: float = float(
            self.get_parameter("max_fps").get_parameter_value().double_value or 5.0
        )

        self.queue_size: int = int(
            self.get_parameter("queue_size").get_parameter_value().integer_value or 5
        )
        self.reconnect_interval_sec: float = float(
            self.get_parameter("reconnect_interval_sec").get_parameter_value().double_value or 5.0
        )

        if self.jpeg_quality < 1:
            self.jpeg_quality = 1
        if self.jpeg_quality > 100:
            self.jpeg_quality = 100
        if self.max_fps <= 0:
            self.max_fps = 1.0

        # ================== ROS 設定 ==================
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            Image, self.rgb_topic, self._on_image, qos_profile_sensor_data
        )

        # ================== 上傳佇列 / WebSocket Thread ==================
        self.frame_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=self.queue_size)
        self.stop_event = threading.Event()
        self.ws_thread: Optional[threading.Thread] = None
        self.last_sent_time: float = 0.0

        # 實際 WebSocket URL：加上 robot_id / camera_id
        connector = "&" if "?" in self.api_ws_url else "?"
        self.full_ws_url: str = (
            f"{self.api_ws_url}{connector}robot_id={self.robot_id}&camera_id={self.camera_id}"
        )

        self.get_logger().info(
            f"CameraStreamingNode started.\n"
            f"  robot_id   = {self.robot_id}\n"
            f"  camera_id  = {self.camera_id}\n"
            f"  rgb_topic  = {self.rgb_topic}\n"
            f"  api_ws_url = {self.full_ws_url}\n"
            f"  jpeg_q     = {self.jpeg_quality}\n"
            f"  max_fps    = {self.max_fps}\n"
            f"  queue_size = {self.queue_size}"
        )

        # 啟動 WebSocket 傳送執行緒
        self.ws_thread = threading.Thread(target=self._ws_worker, daemon=True)
        self.ws_thread.start()

    # ================================================================
    # ROS 訂閱回呼：收到影像 → 壓 JPEG → 丟進佇列
    # ================================================================
    def _on_image(self, msg: Image):
        """收到相機影像後，把它轉成 JPEG 壓進佇列。"""

        # FPS 限制：簡單用時間間隔控制，避免 Jetson 過度忙碌
        now = time.monotonic()
        min_interval = 1.0 / self.max_fps
        if now - self.last_sent_time < min_interval:
            return
        self.last_sent_time = now

        try:
            # ROS Image -> OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 轉換失敗: {e}")
            return

        if cv_image is None:
            return

        # 確保是 8-bit 3 channel
        if cv_image.dtype != np.uint8:
            cv_image = cv_image.astype(np.uint8)
        if cv_image.ndim == 2:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        # 壓成 JPEG
        try:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
            ok, buf = cv2.imencode(".jpg", cv_image, encode_param)
            if not ok:
                self.get_logger().warn("JPEG 壓縮失敗")
                return
            jpeg_bytes: bytes = buf.tobytes()
        except Exception as e:
            self.get_logger().warn(f"JPEG 壓縮異常: {e}")
            return

        # 丟進佇列（滿了就丟掉最新這張，不要卡住 callback）
        try:
            self.frame_queue.put_nowait(jpeg_bytes)
        except queue.Full:
            # 不印太多 log，避免洗版
            pass

    # ================================================================
    # WebSocket 傳送執行緒：負責連線 / 重連 / 送出 JPEG + heartbeat
    # ================================================================
    def _ws_worker(self):
        """
        在獨立執行緒中跑 WebSocket 連線與重連邏輯。

        - 有 frame 就送 frame
        - 沒有 frame 的時候，定期送 ping，避免 Cloudflare / proxy 把 idle 連線砍掉
        """
        ws: Optional[websocket.WebSocket] = None

        while not self.stop_event.is_set():
            try:
                self.get_logger().info(f"嘗試連線到 WebSocket：{self.full_ws_url}")
                ws = websocket.create_connection(
                    self.full_ws_url,
                    timeout=10,
                    # 關掉 Nagle，降低延遲
                    sockopt=((socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),),
                )
                self.get_logger().info("WebSocket 已連線，開始傳送影像。")

                # heartbeat 參數
                ping_interval = 20.0  # N 秒沒任何 I/O 就送一次 ping
                last_io_time = time.monotonic()  # 最後一次成功 send frame / ping 的時間

                while not self.stop_event.is_set():
                    now = time.monotonic()
                    idle_for = now - last_io_time

                    # 1) 先嘗試拿一張 frame，最多等 1 秒
                    try:
                        frame: bytes = self.frame_queue.get(timeout=1.0)
                        # 有拿到 frame：送出去
                        try:
                            ws.send_binary(frame)
                            last_io_time = time.monotonic()
                        except WebSocketConnectionClosedException as e:
                            self.get_logger().warn(
                                f"WebSocket 已被遠端關閉（send frame 時），準備重連："
                                f"{type(e).__name__}: {e}"
                            )
                            break
                        except Exception as e:
                            self.get_logger().warn(
                                f"WebSocket 傳送 frame 失敗，將中斷重連：{type(e).__name__}: {e}"
                            )
                            break

                    except queue.Empty:
                        # 2) 一秒內都沒有 frame → 看看需不需要送 ping
                        idle_for = time.monotonic() - last_io_time
                        if idle_for >= ping_interval:
                            try:
                                # 這裡使用 ping()，部分版本沒有的話就用 send() + PING opcode
                                if hasattr(ws, "ping"):
                                    ws.ping()
                                else:
                                    ws.send(b"", opcode=websocket.ABNF.OPCODE_PING)
                                last_io_time = time.monotonic()
                                # debug log 不一定要開，如果太吵可以改成 debug
                                # self.get_logger().debug("WebSocket ping 已送出。")
                            except WebSocketConnectionClosedException as e:
                                self.get_logger().warn(
                                    f"WebSocket 已被遠端關閉（send ping 時），準備重連："
                                    f"{type(e).__name__}: {e}"
                                )
                                break
                            except Exception as e:
                                self.get_logger().warn(
                                    f"WebSocket ping 失敗，將中斷重連：{type(e).__name__}: {e}"
                                )
                                break
                        # 沒超過 ping_interval 就繼續 loop（等下一輪）

                self.get_logger().info("WebSocket 離線，準備重連。")

            except Exception as e:
                self.get_logger().warn(
                    f"WebSocket 連線失敗：{type(e).__name__}: {e}"
                )

            # 關閉連線（若還開著）
            try:
                if ws is not None:
                    try:
                        close_status = getattr(ws, "close_status", None)
                        if close_status is not None:
                            self.get_logger().info(f"WebSocket close_status={close_status}")
                    except Exception:
                        pass

                    ws.close()
            except Exception:
                pass
            ws = None

            if self.stop_event.is_set():
                break

            time.sleep(self.reconnect_interval_sec)

        self.get_logger().info("WebSocket worker 結束。")

    # ================================================================
    # Node 結束清理
    # ================================================================
    def destroy_node(self):
        """覆寫 destroy_node，把執行緒也一起收掉。"""
        self.get_logger().info("CameraStreamingNode 正在關閉，請稍候...")
        # 通知執行緒停止
        self.stop_event.set()
        # 等候執行緒結束
        try:
            if self.ws_thread is not None and self.ws_thread.is_alive():
                self.ws_thread.join(timeout=5.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
