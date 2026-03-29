#!/usr/bin/env python3
"""semantic_target_tf_node – camera-frame targets -> world-frame targets.

Reads camera-frame 3-D target points from /uav/semantic_targets_camera,
applies a chain of static + dynamic transforms to express each point in
the planning world frame, then publishes the result on
/uav/semantic_targets_world.  It can optionally also publish the primary
(highest-score) target as a geometry_msgs/PoseStamped on /uav/target_goal
so that EGO-Planner can consume it directly.

Transform chain
---------------
  camera_optical_frame  (fixed extrinsics from parameter)
       ↓  R_cam_body, t_cam_body
  body_frame            (the drone body – tied to the incoming odom pose)
       ↓  pose from /odom topic (nav_msgs/Odometry)
  world_frame           (planning frame, ENU)

NED / ENU handling
------------------
AirSim publishes odom in NED (x=North, y=East, z=Down).
EGO-Planner expects world-frame in ENU / "world" convention where
z is up.  The node converts NED -> ENU when `odom_frame_is_ned` is True
(the default), matching the existing mapping_config.yaml convention.

The NED -> ENU rotation is:
  x_enu =  x_ned_pos_north   (no change for x, but axis meaning changes)
  Actually the standard NED→ENU rotation is:
    R_ned_to_enu = [[0,1,0],[1,0,0],[0,0,-1]]  i.e.
        x_enu = y_ned  (East)
        y_enu = x_ned  (North)
        z_enu = -z_ned (Up)

Camera extrinsics (camera_optical -> body)
------------------------------------------
The camera is mounted on the drone.  The default extrinsic rotation assumes
the camera faces forward, z-axis pointing forward (optical convention):
  camera_optical: x=right, y=down, z=forward
  body NED:       x=forward, y=right, z=down

So R_body_from_cam_optical (default, forward-facing no tilt):
  x_body =  z_cam   (forward)
  y_body =  x_cam   (right)
  z_body =  y_cam   (down)
This can be overridden via the `cam_to_body_rpy_deg` parameter.

Parameters
----------
  camera_targets_topic  (str)   default /uav/semantic_targets_camera
  world_targets_topic   (str)   default /uav/semantic_targets_world
  odom_topic            (str)   default /airsim_node/PX4/odom_local_ned
  target_goal_topic     (str)   default /uav/target_goal
  publish_target_goal   (bool)  default True
  odom_frame_is_ned     (bool)  default True  – apply NED->ENU conversion
  cam_to_body_rpy_deg   (str)   default "0,0,0"
    roll/pitch/yaw applied to the camera-optical->body rotation (additional
    correction in degrees; use "0,0,0" for a perfect forward-facing camera).
  cam_body_offset_m     (str)   default "0,0,0"
    x,y,z translation of camera in body frame (metres)
  world_targets_altitude_hold_m (float) default -1.0
    if >= 0 clamp published /uav/target_goal altitude to this value (metres ENU)
  max_odom_age_sec      (float) default 1.0
"""

import json
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from visualization_msgs.msg import Marker


def _rpy_to_rotation_matrix(r_deg: float, p_deg: float, y_deg: float) -> np.ndarray:
    """Extrinsic ZYX rotation (yaw->pitch->roll) -> 3x3 matrix."""
    r = math.radians(r_deg)
    p = math.radians(p_deg)
    y = math.radians(y_deg)
    Rz = np.array([
        [math.cos(y), -math.sin(y), 0],
        [math.sin(y),  math.cos(y), 0],
        [0,            0,           1],
    ])
    Ry = np.array([
        [ math.cos(p), 0, math.sin(p)],
        [ 0,           1, 0          ],
        [-math.sin(p), 0, math.cos(p)],
    ])
    Rx = np.array([
        [1, 0,           0          ],
        [0, math.cos(r), -math.sin(r)],
        [0, math.sin(r),  math.cos(r)],
    ])
    return Rz @ Ry @ Rx


def _parse_floats(s: str) -> List[float]:
    return [float(v.strip()) for v in s.split(',') if v.strip()]


_R_NED_TO_ENU = np.array([
    [0, 1,  0],
    [1, 0,  0],
    [0, 0, -1],
], dtype=np.float64)

_R_CAM_OPT_TO_BODY_NED_DEFAULT = np.array([
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0],
], dtype=np.float64)


class SemanticTargetTFNode(Node):
    """Transform camera-frame targets to world frame and publish."""

    def __init__(self) -> None:
        super().__init__('semantic_target_tf_node')

        self.declare_parameter('camera_targets_topic', '/uav/semantic_targets_camera')
        self.declare_parameter('world_targets_topic', '/uav/semantic_targets_world')
        self.declare_parameter('odom_topic', '/airsim_node/PX4/odom_local_ned')
        self.declare_parameter('target_goal_topic', '/uav/target_goal')
        self.declare_parameter('semantic_marker_topic', '/uav/semantic_target_marker')
        self.declare_parameter('publish_target_goal', True)
        self.declare_parameter('odom_frame_is_ned', True)
        self.declare_parameter('cam_to_body_rpy_deg', '0,0,0')
        self.declare_parameter('cam_body_offset_m', '0,0,0')
        self.declare_parameter('world_targets_altitude_hold_m', -1.0)
        self.declare_parameter('max_odom_age_sec', 1.0)

        p = self.get_parameter
        self._cam_targets_topic = str(p('camera_targets_topic').value)
        self._world_targets_topic = str(p('world_targets_topic').value)
        self._odom_topic = str(p('odom_topic').value)
        self._target_goal_topic = str(p('target_goal_topic').value)
        self._marker_topic = str(p('semantic_marker_topic').value)
        self._publish_goal = bool(p('publish_target_goal').value)
        self._odom_is_ned = bool(p('odom_frame_is_ned').value)
        self._alt_hold = float(p('world_targets_altitude_hold_m').value)
        self._max_odom_age = float(p('max_odom_age_sec').value)

        rpy = _parse_floats(str(p('cam_to_body_rpy_deg').value))
        rpy = (rpy + [0.0, 0.0, 0.0])[:3]
        self._R_cam_to_body = (
            _R_CAM_OPT_TO_BODY_NED_DEFAULT @ _rpy_to_rotation_matrix(*rpy)
        )

        offset = _parse_floats(str(p('cam_body_offset_m').value))
        self._t_cam_in_body = np.array(
            (offset + [0.0, 0.0, 0.0])[:3], dtype=np.float64
        )

        self._world_targets_pub = self.create_publisher(
            String, self._world_targets_topic, 10
        )
        self._marker_pub = self.create_publisher(Marker, self._marker_topic, 10)
        if self._publish_goal:
            self._goal_pub = self.create_publisher(
                PoseStamped, self._target_goal_topic, 10
            )

        self.create_subscription(
            String, self._cam_targets_topic, self._on_camera_targets, 10
        )
        self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, qos_profile_sensor_data
        )

        self._odom_pos: Optional[np.ndarray] = None  # [x, y, z] in odom frame
        self._odom_quat: Optional[np.ndarray] = None  # [w, x, y, z]
        self._odom_stamp: Optional[float] = None
        self._marker_id = 0

        self.get_logger().info(
            f'SemanticTargetTFNode ready | '
            f'cam_targets={self._cam_targets_topic} '
            f'odom={self._odom_topic} '
            f'world_out={self._world_targets_topic} '
            f'publish_goal={self._publish_goal} '
            f'ned_odom={self._odom_is_ned}'
        )

    # ──────────────────────────────── callbacks ───────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._odom_pos = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        self._odom_quat = np.array([q.w, q.x, q.y, q.z], dtype=np.float64)
        stamp = msg.header.stamp
        self._odom_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_camera_targets(self, msg: String) -> None:
        if self._odom_pos is None:
            self.get_logger().warn(
                f'No odometry yet from {self._odom_topic}', throttle_duration_sec=5.0
            )
            return

        age = time.monotonic() - (self._odom_stamp or 0.0)
        if age > self._max_odom_age:
            self.get_logger().warn(
                f'Odometry stale ({age:.2f}s)', throttle_duration_sec=3.0
            )
            return

        try:
            payload: Dict = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'JSON parse error: {exc}')
            return

        camera_targets: List[Dict] = payload.get('targets', [])
        if not camera_targets:
            return

        R_body_to_world, t_body_in_world = self._get_body_to_world_transform()

        world_targets = []
        for tgt in camera_targets:
            try:
                p_cam = np.array([tgt['x_cam'], tgt['y_cam'], tgt['z_cam']])
                p_world = self._transform_to_world(
                    p_cam, R_body_to_world, t_body_in_world
                )
                world_targets.append({
                    'label': tgt.get('label', ''),
                    'score': tgt.get('score', 0.0),
                    'u': tgt.get('u', 0.0),
                    'v': tgt.get('v', 0.0),
                    'depth_m': tgt.get('depth_m', 0.0),
                    'x_cam': tgt.get('x_cam', 0.0),
                    'y_cam': tgt.get('y_cam', 0.0),
                    'z_cam': tgt.get('z_cam', 0.0),
                    'x_world': round(float(p_world[0]), 3),
                    'y_world': round(float(p_world[1]), 3),
                    'z_world': round(float(p_world[2]), 3),
                    'stamp': tgt.get('stamp', 0.0),
                    'frame_id': tgt.get('frame_id', ''),
                })
                self.get_logger().info(
                    f'[TF] "{tgt.get("label","")}" '
                    f'cam=({tgt["x_cam"]:.2f},{tgt["y_cam"]:.2f},{tgt["z_cam"]:.2f}) '
                    f'-> world=({p_world[0]:.2f},{p_world[1]:.2f},{p_world[2]:.2f})'
                )
            except (KeyError, TypeError) as exc:
                self.get_logger().warn(f'Transform failed for target: {exc}')

        if not world_targets:
            return

        out = String()
        out.data = json.dumps(
            {
                'prompt': payload.get('prompt', ''),
                'image_stamp': payload.get('image_stamp', 0.0),
                'targets': world_targets,
            },
            ensure_ascii=False,
        )
        self._world_targets_pub.publish(out)

        best = max(world_targets, key=lambda t: t['score'])
        self._publish_marker(best)

        if self._publish_goal:
            self._publish_pose_goal(best)

    # ─────────────────────────────── transforms ───────────────────────────────

    def _get_body_to_world_transform(
        self,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return (R_body_to_world, t_body_in_world) both as float64 arrays."""
        pos = self._odom_pos  # drone position in odom frame
        q = self._odom_quat   # orientation quaternion [w,x,y,z]

        # Quaternion -> rotation matrix (body -> odom frame)
        R_body_odom = self._quat_to_matrix(q)

        if self._odom_is_ned:
            # Convert NED odom to ENU world
            R_body_to_world = _R_NED_TO_ENU @ R_body_odom
            t_body_in_world = _R_NED_TO_ENU @ pos
        else:
            R_body_to_world = R_body_odom
            t_body_in_world = pos

        return R_body_to_world, t_body_in_world

    def _transform_to_world(
        self,
        p_cam: np.ndarray,
        R_body_to_world: np.ndarray,
        t_body_in_world: np.ndarray,
    ) -> np.ndarray:
        # camera_optical -> body
        p_body = self._R_cam_to_body @ p_cam + self._t_cam_in_body
        # body -> world
        p_world = R_body_to_world @ p_body + t_body_in_world
        return p_world

    @staticmethod
    def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
        """q = [w,x,y,z] -> 3x3 rotation matrix."""
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ], dtype=np.float64)

    # ────────────────────────────── publishers ────────────────────────────────

    def _publish_pose_goal(self, tgt: Dict) -> None:
        x = float(tgt['x_world'])
        y = float(tgt['y_world'])
        z = float(tgt['z_world'])
        if self._alt_hold >= 0.0:
            z = self._alt_hold

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)
        self.get_logger().info(
            f'[GOAL] Published /uav/target_goal '
            f'({x:.2f}, {y:.2f}, {z:.2f}) for "{tgt.get("label","")}"'
        )

    def _publish_marker(self, tgt: Dict) -> None:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.ns = 'semantic_target'
        m.id = self._marker_id
        self._marker_id = (self._marker_id + 1) % 10000
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(tgt['x_world'])
        m.pose.position.y = float(tgt['y_world'])
        m.pose.position.z = float(tgt['z_world'])
        m.pose.orientation.w = 1.0
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 0.85
        m.lifetime.sec = 3
        m.lifetime.nanosec = 0
        self._marker_pub.publish(m)


def main(args=None) -> None:
    """Entrypoint registered in setup.py."""
    rclpy.init(args=args)
    node = SemanticTargetTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
