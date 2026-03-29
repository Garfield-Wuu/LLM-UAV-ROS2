#!/usr/bin/env python3
"""YOLO-World open-vocabulary detector ROS 2 node.

Subscribes to an AirSim RGB image stream and a text prompt topic, runs
YOLO-World inference on GPU (falls back to CPU), and publishes detections
as a JSON string on /uav/detections_2d.

Topic contract
--------------
Subscribed:
  <rgb_topic>       sensor_msgs/Image   – RGB image from AirSim camera
  <prompt_topic>    std_msgs/String     – comma-separated text prompt, e.g. "red car"

Published:
  <detections_topic> std_msgs/String    – JSON with fields:
      prompt, image_stamp, frame_id, inference_ms, score_thr,
      detections: [{label, score, bbox:{x_min,y_min,x_max,y_max},
                    stamp, frame_id}]

Parameters
----------
  rgb_topic                 (str)   default /airsim_node/PX4/CameraDepth1/Scene
  prompt_topic              (str)   default /uav/target_query
  detections_topic          (str)   default /uav/detections_2d
  texts                     (str)   default "red car"   (static prompt fallback)
  score_thr                 (float) default 0.25
  max_dets                  (int)   default 20
  min_inference_interval_sec(float) default 0.5  – rate-limit on GPU
  yolo_world_root           (str)   default /home/hw/YOLO-World
  config_path               (str)   relative or absolute path to mmdet config
  weights_path              (str)   relative or absolute path to checkpoint
  work_dir                  (str)   mmengine work_dir (temp output)
"""

import json
import os
import os.path as osp
import sys
import tempfile
import threading
import time
import warnings
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


_DEFAULT_CONFIG = (
    'configs/pretrain/'
    'yolo_world_v2_s_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_'
    'goldg_train_lvis_minival.py'
)
_DEFAULT_WEIGHTS = 'weights/yolo_world_v2_s_stage1.pth'
_DEFAULT_PROMPT = 'red car'
_DEFAULT_YOLO_ROOT = '/home/hw/YOLO-World'


def _parse_labels(raw: str) -> List[str]:
    return [item.strip() for item in raw.split(',') if item.strip()]


def _stamp_sec(stamp_msg) -> float:
    return float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9


class YoloWorldDetector(Node):
    """ROS 2 wrapper around YOLO-World open-vocabulary detector."""

    def __init__(self) -> None:
        super().__init__('yolo_world_detector')

        self.declare_parameter('rgb_topic', '/airsim_node/PX4/CameraDepth1/Scene')
        self.declare_parameter('prompt_topic', '/uav/target_query')
        self.declare_parameter('detections_topic', '/uav/detections_2d')
        self.declare_parameter('texts', _DEFAULT_PROMPT)
        self.declare_parameter('score_thr', 0.25)
        self.declare_parameter('max_dets', 20)
        self.declare_parameter('min_inference_interval_sec', 0.5)
        self.declare_parameter('yolo_world_root', _DEFAULT_YOLO_ROOT)
        self.declare_parameter('config_path', _DEFAULT_CONFIG)
        self.declare_parameter('weights_path', _DEFAULT_WEIGHTS)
        self.declare_parameter('work_dir', '/tmp/hw_insight_yolo_world')

        p = self.get_parameter
        self._rgb_topic = str(p('rgb_topic').value)
        self._prompt_topic = str(p('prompt_topic').value)
        self._det_topic = str(p('detections_topic').value)
        self._prompt = str(p('texts').value).strip()
        self._score_thr = float(p('score_thr').value)
        self._max_dets = int(p('max_dets').value)
        self._min_interval = float(p('min_inference_interval_sec').value)
        self._yolo_root = str(p('yolo_world_root').value)
        self._config_path = str(p('config_path').value)
        self._weights_path = str(p('weights_path').value)
        self._work_dir = str(p('work_dir').value)

        self._det_pub = self.create_publisher(String, self._det_topic, 10)
        self.create_subscription(
            Image, self._rgb_topic, self._on_image, qos_profile_sensor_data,
        )
        self.create_subscription(
            String, self._prompt_topic, self._on_prompt, 10,
        )

        self._frame_lock = threading.Lock()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_stamp = None
        self._latest_frame_id = ''
        self._frame_counter = 0
        self._last_inferred_counter = -1
        self._last_publish_time = 0.0
        self._inference_active = False

        self._cv2 = None
        self._torch = None
        self._model = None
        self._pipeline = None

        self._load_model()
        self.create_timer(0.1, self._poll_inference)

        self.get_logger().info(
            f'YoloWorldDetector ready | rgb={self._rgb_topic} '
            f'prompt_topic={self._prompt_topic} '
            f'out={self._det_topic} '
            f'texts="{self._prompt}"'
        )

    # ──────────────────────────────── model init ─────────────────────────────

    def _load_model(self) -> None:
        warnings.filterwarnings('ignore', category=FutureWarning)
        try:
            root = osp.abspath(self._yolo_root)
            self._ensure_sys_path(root)

            import cv2  # noqa: PLC0415
            import torch  # noqa: PLC0415
            from mmengine.config import Config  # noqa: PLC0415
            from mmengine.dataset import Compose  # noqa: PLC0415
            from mmdet.apis import init_detector  # noqa: PLC0415
            from mmdet.utils import get_test_pipeline_cfg  # noqa: PLC0415

            config_abs = (
                self._config_path if osp.isabs(self._config_path)
                else osp.join(root, self._config_path)
            )
            weights_abs = (
                self._weights_path if osp.isabs(self._weights_path)
                else osp.join(root, self._weights_path)
            )

            for label, path in [('config', config_abs), ('weights', weights_abs)]:
                if not osp.exists(path):
                    raise FileNotFoundError(f'{label} not found: {path}')

            os.makedirs(self._work_dir, exist_ok=True)
            cfg = Config.fromfile(config_abs)
            cfg.work_dir = self._work_dir
            cfg.load_from = weights_abs

            local_clip = osp.join(root, 'clip_tokenizer')
            if osp.exists(local_clip):
                cfg.model.backbone.text_model.model_name = local_clip

            device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
            model = init_detector(cfg, checkpoint=weights_abs, device=device, palette='coco')
            pipeline = Compose(get_test_pipeline_cfg(cfg=cfg))

            self._cv2 = cv2
            self._torch = torch
            self._model = model
            self._pipeline = pipeline
            self.get_logger().info(f'YOLO-World model loaded on {device.upper()}')
        except Exception as exc:
            self.get_logger().error(f'Model init failed: {exc}')

    def _ensure_sys_path(self, root: str) -> None:
        for candidate in [root, osp.join(root, 'third_party', 'mmyolo')]:
            if osp.isdir(candidate) and candidate not in sys.path:
                sys.path.insert(0, candidate)

    # ───────────────────────────────── callbacks ──────────────────────────────

    def _on_prompt(self, msg: String) -> None:
        prompt = msg.data.strip()
        if prompt:
            self._prompt = prompt
            self.get_logger().info(f'Prompt updated: "{prompt}"')

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._ros_image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}')
            return
        with self._frame_lock:
            self._latest_bgr = bgr
            self._latest_stamp = msg.header.stamp
            self._latest_frame_id = msg.header.frame_id
            self._frame_counter += 1

    # ─────────────────────────────── polling loop ─────────────────────────────

    def _poll_inference(self) -> None:
        if self._model is None or self._pipeline is None:
            return
        if self._inference_active:
            return
        now = time.monotonic()
        if now - self._last_publish_time < self._min_interval:
            return
        with self._frame_lock:
            if (
                self._latest_bgr is None
                or self._frame_counter == self._last_inferred_counter
            ):
                return
            bgr = self._latest_bgr.copy()
            stamp = self._latest_stamp
            frame_id = self._latest_frame_id
            seq = self._frame_counter
            prompt = self._prompt

        self._inference_active = True
        t = threading.Thread(
            target=self._infer_and_publish,
            args=(bgr, stamp, frame_id, seq, prompt),
            daemon=True,
        )
        t.start()

    # ───────────────────────────── inference worker ───────────────────────────

    def _infer_and_publish(
        self,
        bgr: np.ndarray,
        stamp,
        frame_id: str,
        seq: int,
        prompt: str,
    ) -> None:
        tmp_path = None
        try:
            labels = _parse_labels(prompt) or _parse_labels(_DEFAULT_PROMPT)
            texts = [[lbl] for lbl in labels] + [[' ']]

            tmp_path = self._save_temp_image(bgr)
            boxes, pred_labels, scores, elapsed_ms = self._run_yolo(tmp_path, texts)

            detections = []
            for box, lbl_idx, score in zip(boxes, pred_labels, scores):
                idx = int(lbl_idx)
                if idx >= len(labels):
                    continue
                x1, y1, x2, y2 = [int(round(float(v))) for v in box]
                detections.append({
                    'label': labels[idx],
                    'score': round(float(score), 4),
                    'bbox': {'x_min': x1, 'y_min': y1, 'x_max': x2, 'y_max': y2},
                    'stamp': _stamp_sec(stamp),
                    'frame_id': frame_id,
                })

            payload = {
                'prompt': prompt,
                'image_stamp': _stamp_sec(stamp),
                'frame_id': frame_id,
                'inference_ms': round(elapsed_ms, 1),
                'score_thr': self._score_thr,
                'detections': detections,
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._det_pub.publish(msg)

            self._last_publish_time = time.monotonic()
            self._last_inferred_counter = seq
            self.get_logger().info(
                f'[DETECT] prompt="{prompt}" '
                f'dets={len(detections)} '
                f'({elapsed_ms:.1f} ms)'
            )
        except Exception as exc:
            self.get_logger().error(f'Inference error: {exc}')
        finally:
            if tmp_path and osp.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass
            self._inference_active = False

    def _run_yolo(
        self,
        image_path: str,
        texts: List[List[str]],
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        data_info = dict(img_id=0, img_path=image_path, texts=texts)
        data_info = self._pipeline(data_info)
        batch = dict(
            inputs=data_info['inputs'].unsqueeze(0),
            data_samples=[data_info['data_samples']],
        )
        t0 = time.perf_counter()
        with self._torch.no_grad():
            output = self._model.test_step(batch)[0]
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        pred = output.pred_instances
        pred = pred[pred.scores.float() > self._score_thr]
        if len(pred.scores) > self._max_dets:
            indices = pred.scores.float().topk(self._max_dets)[1]
            pred = pred[indices]
        pred = pred.cpu().numpy()
        return pred['bboxes'], pred['labels'], pred['scores'], elapsed_ms

    # ──────────────────────────────── image utils ─────────────────────────────

    def _save_temp_image(self, bgr: np.ndarray) -> str:
        fd, path = tempfile.mkstemp(suffix='.jpg', prefix='yw_', dir='/tmp')
        os.close(fd)
        self._cv2.imwrite(path, bgr)
        return path

    def _ros_image_to_bgr(self, msg: Image) -> np.ndarray:
        """Convert sensor_msgs/Image to OpenCV BGR ndarray without cv_bridge."""
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

        raise ValueError(f'Unsupported image encoding: {enc}')


def main(args=None) -> None:
    """Entrypoint registered in setup.py."""
    rclpy.init(args=args)
    node = YoloWorldDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
