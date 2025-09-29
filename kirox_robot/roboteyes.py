#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kirox 影像分數節點（SCRFD + 6DRepNet）
只輸出影像分數：
- q：單幀品質分數 (0~1)
- q_bar：EMA 平滑分數 (0~1)
同時發佈：距離、p_front、yaw_deg、pitch_deg 以便驗證。
去除：6 秒/加扣分/觸發判斷/服務，確保精簡。

新增：
- 轉發「純原始影像」到 latched topic（TRANSIENT_LOCAL），支援等比縮放 + JPEG 品質參數
  topic 由參數 rgb_latest_topic 控制（預設 vision/rgb_latest）
"""

import math
from typing import Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge


# =========================
# 超參數（可調整的常數）
# =========================
THETA_FRONT   = 0.60
K_GAZE        = 8.0
D0_M          = 1.20
GAMMA_DIST    = 1.5
EMA_LAMBDA    = 0.80
FACE_REAL_W   = 0.16
FOCAL_PX      = 600.0
SHOW_WINDOW   = False
PUBLISH_DEBUG = False
WINDOW_NAME   = "Kirox Vision Score (SCRFD + 6DRepNet)"
EPS           = 1e-6


# =========================
# 工具函式
# =========================
def _sigmoid(x: float) -> float:
    if x >= 0:
        z = math.exp(-x)
        return 1.0 / (1.0 + z)
    else:
        z = math.exp(x)
        return z / (1.0 + z)

def _w_gaze(p_front: float) -> float:
    p = max(0.0, min(1.0, p_front))
    return _sigmoid(K_GAZE * (p - THETA_FRONT))

def _w_dist(distance_m: float) -> float:
    d = max(0.0, distance_m)
    return math.exp(- (d / max(D0_M, EPS)) ** max(GAMMA_DIST, 1.0))


# =========================
# 節點
# =========================
class RobotEyesNode(Node):
    def __init__(self):
        super().__init__("robot_eyes_node")

        # ---- 基本參數 ----
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("use_gpu", True)
        self.declare_parameter("det_size", 640)
        self.declare_parameter("ctx_id", 0)
        self.declare_parameter("pose_device", "cuda:0")

        # 轉發 topic 與輸出畫質控制（等比縮放 + JPEG 品質）
        self.declare_parameter("rgb_latest_topic", "vision/rgb_latest")
        self.declare_parameter("rgb_latest_scale", 0.3)          # 等比倍率（>0），例：0.5 縮小一半，2.0 放大兩倍
        self.declare_parameter("rgb_latest_jpeg_quality", 90)     # 1~100，越大越清晰

        self.rgb_topic        = self.get_parameter("rgb_topic").value
        self.depth_topic      = self.get_parameter("depth_topic").value
        self.use_gpu          = bool(self.get_parameter("use_gpu").value)
        self.det_size         = int(self.get_parameter("det_size").value)
        self.ctx_id           = int(self.get_parameter("ctx_id").value)
        self.pose_device      = str(self.get_parameter("pose_device").value or "auto")

        self.rgb_latest_topic = str(self.get_parameter("rgb_latest_topic").value or "vision/rgb_latest")
        self.scale            = float(self.get_parameter("rgb_latest_scale").value)
        self.jpeg_q           = int(self.get_parameter("rgb_latest_jpeg_quality").value)

        # 合理限制
        self.scale  = max(0.05, min(4.0, self.scale))
        self.jpeg_q = int(np.clip(self.jpeg_q, 1, 100))

        # ---- 載入臉偵測（InsightFace / SCRFD）----
        try:
            from insightface.app import FaceAnalysis
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if self.use_gpu else ['CPUExecutionProvider']
            self.face_app = FaceAnalysis(providers=providers)
            ci = self.ctx_id if self.use_gpu else -1
            self.face_app.prepare(ctx_id=ci, det_size=(self.det_size, self.det_size))
        except Exception as e:
            raise RuntimeError(f"載入 InsightFace 失敗：{e}")

        # ---- 載入頭姿（SixDRepNet）----
        from sixdrepnet import SixDRepNet
        import torch
        self.pose_model = SixDRepNet()
        if torch.cuda.is_available():
            try:
                (getattr(self.pose_model, "model", self.pose_model)).to("cuda")
            except Exception as e:
                self.get_logger().warn(f"SixDRepNet set device failed: {e}")

        # ---- ROS 介面 ----
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.pub_q        = self.create_publisher(Float32, "vision/q_score", latched)
        self.pub_qbar     = self.create_publisher(Float32, "vision/q_bar_score", latched)
        self.pub_distance = self.create_publisher(Float32, "vision/distance", latched)
        self.pub_pfront   = self.create_publisher(Float32, "vision/p_front", latched)
        self.pub_yaw      = self.create_publisher(Float32, "vision/yaw_deg", latched)
        self.pub_pitch    = self.create_publisher(Float32, "vision/pitch_deg", latched)

        # 轉發純影像（壓縮後）
        self.pub_rgb_latest = self.create_publisher(CompressedImage, self.rgb_latest_topic, latched)

        self.bridge = CvBridge()
        self.sub_rgb = self.create_subscription(Image, self.rgb_topic, self.on_rgb, qos_profile_sensor_data)
        self.sub_depth = None
        if self.depth_topic:
            self.sub_depth = self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)

        self.pub_debug = None
        if PUBLISH_DEBUG:
            self.pub_debug = self.create_publisher(Image, "vision/debug_image", qos_profile_sensor_data)

        # ---- 狀態 ----
        self.depth_latest: Optional[np.ndarray] = None
        self.q_bar: float = 0.0
        self._last_frame_for_show: Optional[np.ndarray] = None
        self._fps = 0.0
        self._fps_n = 0
        self._last_fps_t = rclpy.clock.Clock().now().nanoseconds / 1e9

        # 解析度變動時才列印，避免洗版
        self._last_print_wh: Optional[Tuple[int, int]] = None

        self.get_logger().info("RobotEyesNode ready (SCRFD + 6DRepNet).")

    # ----------------- 訂閱回呼 -----------------
    def on_depth(self, msg: Image):
        try:
            self.depth_latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"depth convert error: {e}")

    def on_rgb(self, msg: Image):
        # 取出影像 (BGR)
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"rgb convert error: {e}")
            return

        # 等比縮放（給 rgb_latest 用；偵測仍用原圖 bgr）
        h0, w0 = bgr.shape[:2]
        if abs(self.scale - 1.0) < 1e-6:
            resized = bgr
        else:
            interp = cv2.INTER_AREA if self.scale < 1.0 else cv2.INTER_LINEAR
            new_w = max(1, int(round(w0 * self.scale)))
            new_h = max(1, int(round(h0 * self.scale)))
            resized = cv2.resize(bgr, (new_w, new_h), interpolation=interp)

        # 列印目前輸出解析度（僅在變動時）
        wh = (resized.shape[1], resized.shape[0])
        if wh != self._last_print_wh:
            self._last_print_wh = wh
            self.get_logger().info(f"[rgb_latest] {w0}x{h0} -> {wh[0]}x{wh[1]} (scale={self.scale:.2f}, q={self.jpeg_q})")

        # 發佈 CompressedImage (JPEG)
        try:
            ok, buf = cv2.imencode(".jpg", resized, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_q)])
            if ok:
                msg_comp = CompressedImage()
                msg_comp.header = msg.header
                msg_comp.format = "jpeg"
                msg_comp.data = np.asarray(buf).tobytes()
                self.pub_rgb_latest.publish(msg_comp)
            else:
                self.get_logger().warn("cv2.imencode('.jpg', ...) failed")
        except Exception as e:
            self.get_logger().warn(f"publish rgb_latest failed: {e}")

        # 1) 臉偵測（取最大框）— 使用原圖 bgr（不影響既有行為）
        faces = []
        try:
            faces = self.face_app.get(bgr)
        except Exception as e:
            self.get_logger().warn(f"insightface get() 失敗：{e}")

        best_bbox: Optional[Tuple[int, int, int, int]] = None
        confidence = 0.0
        if faces:
            f = max(faces, key=lambda x: float(x.bbox[2]-x.bbox[0]) * float(x.bbox[3]-x.bbox[1]))
            x1, y1, x2, y2 = map(int, f.bbox)
            best_bbox = (x1, y1, max(1, x2 - x1), max(1, y2 - y1))
            confidence = float(getattr(f, "det_score", 1.0))
            confidence = max(0.0, min(1.0, confidence))
        else:
            self._update_and_publish_scores(
                q=0.0, dist_m=math.inf, p_front=0.0, yaw_deg=None, pitch_deg=None, bgr=bgr, bbox=None
            )
            return

        # 2) 距離估計
        dist_m = self._estimate_distance(best_bbox)

        # 3) 頭姿 → p_front
        p_front, yaw_deg, pitch_deg = self._estimate_front_prob(bgr, best_bbox)

        # 4) q、q_bar
        w_gaze = _w_gaze(p_front)
        w_dist = _w_dist(dist_m if np.isfinite(dist_m) else 1e6)
        q = w_gaze * w_dist * confidence
        self.q_bar = EMA_LAMBDA * self.q_bar + (1.0 - EMA_LAMBDA) * q

        # 5) 發佈/疊圖/視窗
        self._publish_raw(dist_m, p_front, yaw_deg, pitch_deg, q, self.q_bar)
        frame = self._draw_overlay(bgr.copy(), best_bbox, q, self.q_bar, dist_m, w_gaze, w_dist, p_front, yaw_deg, pitch_deg)
        self._last_frame_for_show = frame

        if PUBLISH_DEBUG and self.pub_debug is not None:
            try:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except Exception as e:
                self.get_logger().warn(f"publish debug image failed: {e}")

        if SHOW_WINDOW:
            self._update_fps_and_show(frame)

    # ----------------- 發佈/視覺化 -----------------
    def _update_and_publish_scores(self, q: float, dist_m: float, p_front: float,
                                   yaw_deg: Optional[float], pitch_deg: Optional[float],
                                   bgr: np.ndarray, bbox: Optional[Tuple[int, int, int, int]]):
        self.q_bar = EMA_LAMBDA * self.q_bar + (1.0 - EMA_LAMBDA) * q
        self._publish_raw(dist_m, p_front, yaw_deg, pitch_deg, q, self.q_bar)

        frame = self._draw_overlay(bgr.copy(), bbox, q, self.q_bar, dist_m, None, None, p_front, yaw_deg, pitch_deg)
        self._last_frame_for_show = frame

        if PUBLISH_DEBUG and self.pub_debug is not None:
            try:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except Exception as e:
                self.get_logger().warn(f"publish debug image failed: {e}")

        if SHOW_WINDOW:
            self._update_fps_and_show(frame)

    def _publish_raw(self, dist_m, p_front, yaw_deg, pitch_deg, q, q_bar):
        m_q = Float32();      m_q.data = float(q)
        m_qb = Float32();     m_qb.data = float(q_bar)
        m_d = Float32();      m_d.data = float(dist_m if np.isfinite(dist_m) else float("inf"))
        m_pf = Float32();     m_pf.data = float(p_front)
        m_yaw = Float32();    m_yaw.data = float(yaw_deg) if yaw_deg is not None else float("nan")
        m_pitch = Float32();  m_pitch.data = float(pitch_deg) if pitch_deg is not None else float("nan")
        self.pub_q.publish(m_q)
        self.pub_qbar.publish(m_qb)
        self.pub_distance.publish(m_d)
        self.pub_pfront.publish(m_pf)
        self.pub_yaw.publish(m_yaw)
        self.pub_pitch.publish(m_pitch)

    def _update_fps_and_show(self, frame: np.ndarray):
        now = rclpy.clock.Clock().now().nanoseconds / 1e9
        self._fps_n += 1
        if now - self._last_fps_t >= 1.0:
            self._fps = self._fps_n / (now - self._last_fps_t)
            self._fps_n = 0
            self._last_fps_t = now

        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("window closed by 'q'")
            self.destroy_node()
            rclpy.shutdown()

    # ----------------- 模型映射/工具 -----------------
    def _estimate_front_prob(self, bgr: np.ndarray, bbox: Tuple[int, int, int, int]) -> Tuple[float, Optional[float], Optional[float]]:
        x, y, w, h = bbox
        roi = bgr[max(0, y):y + h, max(0, x):x + w]
        if roi.size == 0:
            return 0.0, None, None
        try:
            roi_resized = cv2.resize(roi, (224, 224))
            pitch_deg, yaw_deg, _ = self.pose_model.predict(roi_resized)
            yaw = abs(float(yaw_deg))
            pitch = abs(float(pitch_deg))
            yaw_score = max(0.0, 1.0 - yaw / 40.0)
            pitch_score = max(0.0, 1.0 - pitch / 35.0)
            p_front = float(max(0.0, min(1.0, 0.5 * (yaw_score + pitch_score))))
            return p_front, float(yaw_deg), float(pitch_deg)
        except Exception:
            return 0.0, None, None

    def _estimate_distance(self, bbox: Tuple[int, int, int, int]) -> float:
        x, y, w, h = bbox
        if self.depth_latest is not None:
            rx = x + int(w * 0.25)
            ry = y + int(h * 0.25)
            rw = max(1, int(w * 0.50))
            rh = max(1, int(h * 0.50))
            patch = self.depth_latest[ry:ry + rh, rx:rx + rw]
            if patch.size > 0:
                d = np.asarray(patch).astype(np.float32)
                if d.dtype == np.uint16 or d.max() > 10.0:
                    d = d[d > 0]
                    if d.size:
                        return float(np.median(d)) / 1000.0
                else:
                    d = d[d > 0.0]
                    if d.size:
                        return float(np.median(d))
        if w <= 1:
            return math.inf
        Z = (FACE_REAL_W * FOCAL_PX) / float(w)
        return float(Z) if (Z > 0 and np.isfinite(Z)) else math.inf

    def _draw_overlay(self, img: np.ndarray,
                      bbox: Optional[Tuple[int, int, int, int]],
                      q: float, q_bar: float, dist_m: float,
                      w_gaze: Optional[float], w_dist: Optional[float],
                      p_front: Optional[float],
                      yaw_deg: Optional[float],
                      pitch_deg: Optional[float]) -> np.ndarray:
        if bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 200, 0), 2)
        y0 = 28
        cv2.putText(img, f"q={q:.3f}  q_bar={q_bar:.3f}  dist={'{:.2f}'.format(dist_m) if np.isfinite(dist_m) else 'inf'}m",
                    (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 200, 255), 2, cv2.LINE_AA)
        y0 += 28
        if p_front is not None:
            text2 = f"p_front={p_front:.2f}"
            if yaw_deg is not None and pitch_deg is not None:
                text2 += f"  yaw={yaw_deg:.1f}  pitch={pitch_deg:.1f}"
            if w_gaze is not None and w_dist is not None:
                text2 += f"  w_gaze={w_gaze:.3f}  w_dist={w_dist:.3f}"
            cv2.putText(img, text2, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 220, 120), 2, cv2.LINE_AA)
        return img


# =========================
# 入口
# =========================
def main():
    rclpy.init()
    node = RobotEyesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if SHOW_WINDOW:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
