#!/usr/bin/env python3
"""target_grounding_node – 2D detections + depth -> camera-frame 3D points.

Subscribes to YOLO-World detection results, a depth image, and a camera_info
message.  For each detection bbox it samples the depth region, filters invalid
values, takes the median, then back-projects the 2-D centroid to a 3-D point
expressed in the camera optical frame.

Topic contract
--------------
Subscribed:
  <detections_topic>  std_msgs/String     – JSON from yolo_world_detector
  <depth_topic>       sensor_msgs/Image   – DepthPlanar float32 (metres)
  <camera_info_topic> sensor_msgs/CameraInfo

Published:
  <camera_targets_topic>  std_msgs/String – JSON list of grounded targets:
      [{label, score, u, v, depth_m, x_cam, y_cam, z_cam, stamp, frame_id}]

Coordinate convention
---------------------
Output is in the camera *optical* frame (ROS convention):
  x_cam: right, y_cam: down, z_cam: into the scene (forward along optical axis)

This is deliberately NOT the NED/ENU frame – that conversion happens in
semantic_target_tf_node.py, keeping each stage single-responsibility.

Parameters
----------
  detections_topic      (str)   default /uav/detections_2d
  depth_topic           (str)   default /airsim_node/PX4/CameraDepth1/DepthPlanar
  camera_info_topic     (str)   default /airsim_node/PX4/CameraDepth1/camera_info
  camera_targets_topic  (str)   default /uav/semantic_targets_camera
  depth_min_m           (float) default 0.1   – discard closer depths
  depth_max_m           (float) default 50.0  – discard farther depths
  depth_sample_stride   (int)   default 4     – pixel stride inside bbox sampling
  max_depth_age_sec     (float) default 0.5   – reject stale depth images
"""

import json
import threading
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class TargetGroundingNode(Node):
    """Project 2D bboxes onto 3D camera-frame points using depth median."""

    def __init__(self) -> None:
        super().__init__('target_grounding_node')

        self.declare_parameter('detections_topic', '/uav/detections_2d')
        self.declare_parameter(
            'depth_topic', '/airsim_node/PX4/CameraDepth1/DepthPlanar'
        )
        self.declare_parameter(
            'camera_info_topic', '/airsim_node/PX4/CameraDepth1/camera_info'
        )
        self.declare_parameter('camera_targets_topic', '/uav/semantic_targets_camera')
        self.declare_parameter('depth_min_m', 0.1)
        self.declare_parameter('depth_max_m', 50.0)
        self.declare_parameter('depth_sample_stride', 4)
        self.declare_parameter('max_depth_age_sec', 0.5)

        p = self.get_parameter
        det_topic = str(p('detections_topic').value)
        depth_topic = str(p('depth_topic').value)
        ci_topic = str(p('camera_info_topic').value)
        self._cam_targets_topic = str(p('camera_targets_topic').value)
        self._depth_min = float(p('depth_min_m').value)
        self._depth_max = float(p('depth_max_m').value)
        self._stride = int(p('depth_sample_stride').value)
        self._max_depth_age = float(p('max_depth_age_sec').value)

        self._cam_targets_pub = self.create_publisher(
            String, self._cam_targets_topic, 10
        )

        self.create_subscription(String, det_topic, self._on_detections, 10)
        self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(CameraInfo, ci_topic, self._on_camera_info, 10)

        self._depth_lock = threading.Lock()
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_depth_stamp: Optional[float] = None

        self._fx: Optional[float] = None
        self._fy: Optional[float] = None
        self._cx: Optional[float] = None
        self._cy: Optional[float] = None
        self._cam_info_received = False

        self.get_logger().info(
            f'TargetGroundingNode ready | '
            f'det={det_topic} depth={depth_topic} '
            f'out={self._cam_targets_topic}'
        )

    # ──────────────────────────────── callbacks ───────────────────────────────

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._cam_info_received:
            return
        k = msg.k  # row-major 3x3
        self._fx = float(k[0])
        self._fy = float(k[4])
        self._cx = float(k[2])
        self._cy = float(k[5])
        self._cam_info_received = True
        self.get_logger().info(
            f'CameraInfo received: fx={self._fx:.2f} fy={self._fy:.2f} '
            f'cx={self._cx:.2f} cy={self._cy:.2f}'
        )

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self._decode_depth(msg)
        except Exception as exc:
            self.get_logger().warn(f'Depth decode failed: {exc}')
            return
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        with self._depth_lock:
            self._latest_depth = depth
            self._latest_depth_stamp = stamp

    def _on_detections(self, msg: String) -> None:
        if not self._cam_info_received:
            self.get_logger().warn('Waiting for CameraInfo …', throttle_duration_sec=5.0)
            return

        try:
            payload: Dict = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Detection JSON parse error: {exc}')
            return

        detections: List[Dict] = payload.get('detections', [])
        if not detections:
            return

        with self._depth_lock:
            depth_map = self._latest_depth
            depth_stamp = self._latest_depth_stamp

        if depth_map is None:
            self.get_logger().warn('No depth image yet', throttle_duration_sec=5.0)
            return

        now = time.monotonic()
        if depth_stamp is not None and (now - depth_stamp) > self._max_depth_age:
            self.get_logger().warn(
                f'Depth image stale ({now - depth_stamp:.2f}s)', throttle_duration_sec=3.0
            )
            return

        h, w = depth_map.shape
        results = []
        for det in detections:
            try:
                bbox = det['bbox']
                x1 = max(0, int(bbox['x_min']))
                y1 = max(0, int(bbox['y_min']))
                x2 = min(w - 1, int(bbox['x_max']))
                y2 = min(h - 1, int(bbox['y_max']))
                if x2 <= x1 or y2 <= y1:
                    continue

                depth_m = self._sample_median_depth(depth_map, x1, y1, x2, y2)
                if depth_m is None:
                    self.get_logger().warn(
                        f'No valid depth for bbox [{x1},{y1},{x2},{y2}]'
                    )
                    continue

                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0
                x_cam = (u - self._cx) * depth_m / self._fx
                y_cam = (v - self._cy) * depth_m / self._fy
                z_cam = depth_m

                results.append({
                    'label': det.get('label', ''),
                    'score': det.get('score', 0.0),
                    'u': round(u, 1),
                    'v': round(v, 1),
                    'depth_m': round(depth_m, 3),
                    'x_cam': round(x_cam, 3),
                    'y_cam': round(y_cam, 3),
                    'z_cam': round(z_cam, 3),
                    'stamp': det.get('stamp', 0.0),
                    'frame_id': det.get('frame_id', ''),
                })
                self.get_logger().info(
                    f'[GROUND] "{det.get("label","")}": '
                    f'depth={depth_m:.2f}m '
                    f'cam=({x_cam:.2f},{y_cam:.2f},{z_cam:.2f})'
                )
            except (KeyError, TypeError) as exc:
                self.get_logger().warn(f'Skipping malformed detection: {exc}')

        if results:
            out_msg = String()
            out_msg.data = json.dumps(
                {
                    'prompt': payload.get('prompt', ''),
                    'image_stamp': payload.get('image_stamp', 0.0),
                    'targets': results,
                },
                ensure_ascii=False,
            )
            self._cam_targets_pub.publish(out_msg)

    # ──────────────────────────────── helpers ─────────────────────────────────

    def _sample_median_depth(
        self, depth: np.ndarray, x1: int, y1: int, x2: int, y2: int
    ) -> Optional[float]:
        stride = max(1, self._stride)
        region = depth[y1:y2:stride, x1:x2:stride]
        mask = (
            np.isfinite(region)
            & (region > self._depth_min)
            & (region < self._depth_max)
        )
        valid = region[mask]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _decode_depth(self, msg: Image) -> np.ndarray:
        """Return depth in metres as float32 2-D array."""
        enc = msg.encoding.lower()
        h, w = msg.height, msg.width

        if enc in ('32fc1', '32fc'):
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
            return arr.copy()

        if enc in ('16uc1', '16uc'):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
            return arr.astype(np.float32) / 1000.0  # mm -> m

        if enc in ('mono16',):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
            return arr.astype(np.float32) / 1000.0

        raise ValueError(f'Unsupported depth encoding: {enc}')


def main(args=None) -> None:
    """Entrypoint registered in setup.py."""
    rclpy.init(args=args)
    node = TargetGroundingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
