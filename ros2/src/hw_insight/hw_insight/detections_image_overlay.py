#!/usr/bin/env python3
"""Subscribe to RGB + /uav/detections_2d JSON, draw bboxes, publish Image for RViz2.

可选订阅 /uav/semantic_targets_camera：在 bbox 标签旁叠加 grounding 后的 depth_m（米）。
"""

from __future__ import annotations

import json
import threading
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

# 与 yolo_world_detector 一致：AirSim Scene 为 RELIABLE
_QOS_RGB_AIRSIM = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)


class DetectionsImageOverlayNode(Node):
    """在最新 RGB 上绘制检测框，供 RViz Image 显示。"""

    def __init__(self) -> None:
        super().__init__('detections_image_overlay')

        self.declare_parameter('rgb_topic', '/airsim_node/PX4/CameraDepth1/Scene')
        self.declare_parameter('detections_topic', '/uav/detections_2d')
        self.declare_parameter('output_topic', '/uav/camera/detections_overlay')
        self.declare_parameter('line_thickness', 2)
        self.declare_parameter('font_scale', 0.5)
        self.declare_parameter('semantic_targets_topic', '/uav/semantic_targets_camera')
        self.declare_parameter('use_semantic_depth', True)
        self.declare_parameter('stamp_tolerance_sec', 0.35)

        rgb_topic = str(self.get_parameter('rgb_topic').value)
        det_topic = str(self.get_parameter('detections_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)
        sem_topic = str(self.get_parameter('semantic_targets_topic').value).strip()
        self._use_semantic_depth = bool(self.get_parameter('use_semantic_depth').value)
        self._stamp_tol = float(self.get_parameter('stamp_tolerance_sec').value)
        self._thickness = max(1, int(self.get_parameter('line_thickness').value))
        self._font_scale = float(self.get_parameter('font_scale').value)

        self._lock = threading.Lock()
        self._latest_bgr: Optional[np.ndarray] = None
        self._img_frame_id = ''
        self._last_det: Optional[Dict[str, Any]] = None
        self._last_semantic: Optional[Dict[str, Any]] = None

        try:
            import cv2  # noqa: PLC0415
            self._cv2 = cv2
        except ImportError as exc:
            self._cv2 = None
            self.get_logger().error(f'OpenCV (cv2) required: {exc}')

        self._pub = self.create_publisher(Image, out_topic, 10)
        self.create_subscription(Image, rgb_topic, self._on_image, _QOS_RGB_AIRSIM)
        self.create_subscription(String, det_topic, self._on_detections, 10)
        if sem_topic and self._use_semantic_depth:
            self.create_subscription(String, sem_topic, self._on_semantic, 10)

        self.get_logger().info(
            f'DetectionsImageOverlay | rgb={rgb_topic} det={det_topic} '
            f'sem={sem_topic or "(off)"} -> {out_topic}'
        )

    def _on_image(self, msg: Image) -> None:
        if self._cv2 is None:
            return
        try:
            bgr = self._ros_image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(
                f'Image decode failed: {exc}', throttle_duration_sec=5.0,
            )
            return
        with self._lock:
            self._latest_bgr = bgr
            self._img_frame_id = msg.header.frame_id

    def _on_detections(self, msg: String) -> None:
        if self._cv2 is None:
            return
        try:
            payload: Dict[str, Any] = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('detections JSON parse error')
            return
        with self._lock:
            self._last_det = payload
        self._render_and_publish()

    def _on_semantic(self, msg: String) -> None:
        if self._cv2 is None or not self._use_semantic_depth:
            return
        try:
            payload: Dict[str, Any] = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._last_semantic = payload
        # grounding 略晚于 detections，到达后再画一次以带上 depth_m
        self._render_and_publish()

    def _semantic_targets_aligned(
        self, det_image_stamp: float,
    ) -> List[Dict[str, Any]]:
        if not self._last_semantic:
            return []
        st = float(self._last_semantic.get('image_stamp', 0.0))
        if abs(st - float(det_image_stamp)) > self._stamp_tol:
            return []
        return list(self._last_semantic.get('targets', []) or [])

    @staticmethod
    def _pick_depth_m(
        uc: float,
        vc: float,
        label: str,
        targets: List[Dict[str, Any]],
    ) -> Optional[float]:
        """按 label + 与 bbox 中心 (u,v) 最近匹配 grounding 目标。"""
        if not targets:
            return None
        same_label = [t for t in targets if str(t.get('label', '')) == label]
        pool = same_label if same_label else targets
        best_d: Optional[float] = None
        best_dist = 1e18
        for t in pool:
            try:
                tu = float(t['u'])
                tv = float(t['v'])
                dm = float(t['depth_m'])
            except (KeyError, TypeError, ValueError):
                continue
            dist = (tu - uc) ** 2 + (tv - vc) ** 2
            if dist < best_dist:
                best_dist = dist
                best_d = dm
        return best_d

    def _render_and_publish(self) -> None:
        if self._cv2 is None:
            return
        with self._lock:
            if self._latest_bgr is None or self._last_det is None:
                return
            canvas = self._latest_bgr.copy()
            frame_id = self._img_frame_id
            payload = self._last_det

        dets: List[Dict[str, Any]] = payload.get('detections', []) or []
        prompt = str(payload.get('prompt', ''))
        score_thr = payload.get('score_thr', '')
        det_stamp = float(payload.get('image_stamp', 0.0))
        sem_targets = (
            self._semantic_targets_aligned(det_stamp)
            if self._use_semantic_depth
            else []
        )

        green = (0, 220, 0)
        red = (0, 0, 255)
        yellow = (0, 255, 255)

        for det in dets:
            bbox = det.get('bbox') or {}
            try:
                x1 = int(bbox['x_min'])
                y1 = int(bbox['y_min'])
                x2 = int(bbox['x_max'])
                y2 = int(bbox['y_max'])
            except (KeyError, TypeError, ValueError):
                continue
            label = str(det.get('label', ''))
            score = float(det.get('score', 0.0))
            uc = 0.5 * (x1 + x2)
            vc = 0.5 * (y1 + y2)
            depth_m = (
                self._pick_depth_m(uc, vc, label, sem_targets)
                if sem_targets
                else None
            )

            self._cv2.rectangle(canvas, (x1, y1), (x2, y2), green, self._thickness)
            txt = f'{label} {score:.2f}'
            if depth_m is not None:
                txt += f'  d={depth_m:.1f}m'
            fs = self._font_scale
            (tw, th), _ = self._cv2.getTextSize(
                txt, self._cv2.FONT_HERSHEY_SIMPLEX, fs, 1,
            )
            ty = max(th + 8, y1)
            self._cv2.rectangle(
                canvas, (x1, ty - th - 6), (x1 + tw + 4, ty), green, -1,
            )
            self._cv2.putText(
                canvas, txt, (x1 + 2, ty - 4),
                self._cv2.FONT_HERSHEY_SIMPLEX, fs, (0, 0, 0), 1, self._cv2.LINE_AA,
            )

        if not dets and prompt:
            hint = f'{prompt} | dets=0'
            if score_thr != '':
                hint += f' | thr={score_thr}'
            self._cv2.putText(
                canvas, hint, (8, 28),
                self._cv2.FONT_HERSHEY_SIMPLEX, 0.65, red, 2, self._cv2.LINE_AA,
            )

        if prompt and dets:
            self._cv2.putText(
                canvas, prompt, (8, canvas.shape[0] - 12),
                self._cv2.FONT_HERSHEY_SIMPLEX, 0.55, yellow, 2, self._cv2.LINE_AA,
            )

        out = self._bgr_to_ros_image(canvas)
        out.header.frame_id = frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(out)

    def _ros_image_to_bgr(self, msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        h, w = msg.height, msg.width

        if enc in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            channels = 4 if enc.endswith('a8') else 3
            frame = buf.reshape((h, w, channels))
            if enc.startswith('rgb'):
                frame = self._cv2.cvtColor(frame, self._cv2.COLOR_RGB2BGR)
            elif enc == 'rgba8':
                frame = self._cv2.cvtColor(frame, self._cv2.COLOR_RGBA2BGR)
            elif enc == 'bgra8':
                frame = self._cv2.cvtColor(frame, self._cv2.COLOR_BGRA2BGR)
            return frame

        if enc in ('mono8', '8uc1'):
            gray = buf.reshape((h, w))
            return self._cv2.cvtColor(gray, self._cv2.COLOR_GRAY2BGR)

        if enc in ('mono16', '16uc1'):
            gray16 = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
            gray8 = (gray16 / 256).astype(np.uint8)
            return self._cv2.cvtColor(gray8, self._cv2.COLOR_GRAY2BGR)

        raise ValueError(f'Unsupported image encoding: {msg.encoding}')

    @staticmethod
    def _bgr_to_ros_image(bgr: np.ndarray) -> Image:
        msg = Image()
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(msg.width * 3)
        msg.data = bgr.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionsImageOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
