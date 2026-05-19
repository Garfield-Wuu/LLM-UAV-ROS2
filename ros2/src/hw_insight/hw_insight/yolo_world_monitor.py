#!/usr/bin/env python3
"""YOLO-World 检测结果监控节点。

订阅 /uav/detections_2d，格式化打印每帧检测结果，
支持可选的终端彩色输出与推理延迟统计。

用法:
  ros2 run hw_insight yolo_world_monitor
  ros2 run hw_insight yolo_world_monitor --ros-args -p show_empty:=false
"""

import json
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RESET  = '\033[0m'
BOLD   = '\033[1m'
GREEN  = '\033[92m'
YELLOW = '\033[93m'
CYAN   = '\033[96m'
RED    = '\033[91m'
GRAY   = '\033[90m'


class YoloWorldMonitor(Node):
    def __init__(self):
        super().__init__('yolo_world_monitor')
        self.declare_parameter('detections_topic', '/uav/detections_2d')
        self.declare_parameter('show_empty', True)

        topic = str(self.get_parameter('detections_topic').value)
        self._show_empty = bool(self.get_parameter('show_empty').value)

        self._latency_buf: deque = deque(maxlen=20)
        self._frame_count = 0
        self._det_count = 0

        self.create_subscription(String, topic, self._on_detection, 10)
        self.get_logger().info(
            f'{BOLD}YoloWorldMonitor{RESET} 启动，订阅: {CYAN}{topic}{RESET}'
        )

    def _on_detection(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'JSON 解析失败: {e}')
            return

        dets = payload.get('detections', [])
        elapsed = payload.get('inference_ms', 0.0)
        prompt = payload.get('prompt', '')
        self._frame_count += 1
        self._det_count += len(dets)
        self._latency_buf.append(elapsed)

        if not dets and not self._show_empty:
            return

        avg_lat = statistics.mean(self._latency_buf) if self._latency_buf else 0.0

        sep = '─' * 60
        print(f'\n{BOLD}{sep}{RESET}')
        print(
            f'{BOLD}帧 #{self._frame_count:04d}{RESET}  '
            f'prompt={CYAN}"{prompt}"{RESET}  '
            f'推理={YELLOW}{elapsed:.1f}ms{RESET}  '
            f'均值={GRAY}{avg_lat:.1f}ms{RESET}'
        )
        print(f'{sep}')

        if not dets:
            print(f'  {GRAY}（本帧无检测结果）{RESET}')
        else:
            for i, det in enumerate(dets):
                label = det.get('label', '?')
                score = det.get('score', 0.0)
                bbox  = det.get('bbox', {})
                x1, y1 = bbox.get('x_min', 0), bbox.get('y_min', 0)
                x2, y2 = bbox.get('x_max', 0), bbox.get('y_max', 0)
                color = GREEN if score >= 0.5 else YELLOW if score >= 0.35 else GRAY
                print(
                    f'  [{i:02d}] {color}{BOLD}{label:<16}{RESET} '
                    f'score={color}{score:.3f}{RESET}  '
                    f'bbox=[{x1},{y1},{x2},{y2}]'
                )

        print(f'{GRAY}总计: {self._frame_count} 帧, {self._det_count} 次检测{RESET}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloWorldMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
