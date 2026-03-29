#!/usr/bin/env python3
"""
TextCommandBridge — natural language / JSON → PX4 velocity controller.

Architecture notes (learned from LLM-controlled-drone, EchoPilot, Lim et al. 2025):

* Planner–Executor separation: this node is the planner/translator; move_velocity
  is the executor.  Commands are velocity setpoints, never raw MAVLink bypasses,
  except for safety-critical commands (RTL, EMERGENCY_STOP) which publish
  VehicleCommand directly for the shortest possible path to PX4.

* Telemetry-verified execution: TELEMETRY messages carry rich context
  (position, velocity, heading, goto distance, orbit progress) so an
  upstream LLM can make informed decisions without re-querying PX4.

* 10 Hz control loop: all continuous controllers (altitude hold, GOTO_NED,
  ORBIT, YAW_TO) run inside publish_active_command at 10 Hz, matching the
  Offboard heartbeat rate.

* Control gains follow the proportional-velocity approach used in
  LLM-controlled-drone's command_translator: P-gain × error, clamped to
  max speed.  ORBIT uses tangential + corrective velocity to maintain radius.
"""

import math
from dataclasses import dataclass
import json
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

from hw_interface.msg import HWSimpleKeyboardInfo


# ── ARM phase ────────────────────────────────────────────────────────────────
ARM_TRIGGER_Z = -6.0     # m/s, NED upward – crosses move_velocity's z < -5 gate
ARM_TRIGGER_SEC = 2.5    # seconds: WSL2 time-sync jitter can delay ARM acceptance;
                         # 2.5s gives PX4 time to reconverge before giving up

# ── Speed hard limits (safety clamps) ────────────────────────────────────────
MAX_H_SPEED = 15.0       # m/s horizontal
MAX_V_SPEED = 5.0        # m/s vertical
MAX_YAW_RATE = 1.5       # rad/s

# ── GOTO_NED controller ───────────────────────────────────────────────────────
GOTO_KP = 0.8            # P-gain: speed = GOTO_KP × distance (clamped)
GOTO_TOLERANCE = 0.5     # metres – waypoint acceptance radius

# ── ORBIT controller ──────────────────────────────────────────────────────────
ORBIT_KP = 1.0           # radius correction P-gain
ORBIT_TIMER_DT = 0.1     # seconds – matches create_timer period

# ── YAW_TO controller ────────────────────────────────────────────────────────
YAW_KP = 1.5             # rad/s per rad error
YAW_TOLERANCE = 0.05     # radians (~3°)


@dataclass(frozen=True)
class VelocityCommand:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    duration_sec: Optional[float] = None


class TextCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__('text_command_bridge')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ──────────────────────────────────────────────────────
        self.command_publisher = self.create_publisher(
            HWSimpleKeyboardInfo, '/hw_insight/keyboard_velocity', 1,
        )
        self.status_publisher = self.create_publisher(
            String, '/uav/llm_task_status', 10,
        )
        self.declare_parameter('target_goal_topic', '/uav/target_goal')
        self.declare_parameter('publish_target_goal_on_goto', True)
        self.declare_parameter('planner_mode_for_goto', False)
        self.target_goal_topic = str(self.get_parameter('target_goal_topic').value)
        self.publish_target_goal_on_goto = bool(
            self.get_parameter('publish_target_goal_on_goto').value
        )
        self.planner_mode_for_goto = bool(
            self.get_parameter('planner_mode_for_goto').value
        )
        self.target_goal_pub = self.create_publisher(
            PoseStamped, self.target_goal_topic, 10,
        )
        # Direct VehicleCommand channel for safety-critical actions (RTL, E-STOP)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile,
        )
        # Visual-semantic search: publish query to detector, subscribe to world targets
        self._target_query_pub = self.create_publisher(
            String, '/uav/target_query', 10,
        )

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(String, '/uav/user_command', self.user_command_callback, 10)
        self.create_subscription(
            String, '/uav/semantic_targets_world',
            self._on_semantic_targets_world, 10,
        )
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile,
        )
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile,
        )

        # ── Speed / gain parameters ─────────────────────────────────────────
        self.default_speed_xy = 4.0
        self.default_speed_z = 2.5
        self.default_yaw_rate = 0.6
        self.default_duration = 1.0
        self.takeoff_altitude_default = 6.0
        self.altitude_tolerance = 0.35
        self.altitude_kp = 0.8
        self.max_takeoff_z_speed = 3.0

        # ── Core command state ──────────────────────────────────────────────
        self.hover_command = VelocityCommand()
        self.active_command: VelocityCommand = self.hover_command
        self.active_command_label: str = 'IDLE'
        self.command_deadline_ns: Optional[int] = None
        self.publish_report_pending: bool = False
        self.planner_control_active: bool = False

        # ── TAKEOFF (altitude hold + ARM phase) ─────────────────────────────
        self.takeoff_target_z_ned: Optional[float] = None
        self.takeoff_start_ns: Optional[int] = None

        # ── GOTO_NED state (NED x, y, z) ────────────────────────────────────
        self.goto_target: Optional[Tuple[float, float, float]] = None

        # ── ORBIT state ──────────────────────────────────────────────────────
        self.orbit_center: Optional[Tuple[float, float]] = None
        self.orbit_radius: float = 5.0
        self.orbit_speed: float = 2.0
        self.orbit_angular_speed: float = 0.4   # rad/s
        self.orbit_angle: float = 0.0            # current angle on circle
        self.orbit_z_ned: float = 0.0            # altitude hold during orbit

        # ── YAW_TO state ─────────────────────────────────────────────────────
        self.yaw_target: Optional[float] = None  # radians (NED heading, 0=North)

        # ── Telemetry ────────────────────────────────────────────────────────
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.has_local_position: bool = False

        # ── Visual-semantic search state ─────────────────────────────────────
        self._search_active: bool = False
        self._search_query: str = ''
        self._search_deadline_ns: Optional[int] = None
        self._search_timeout_sec: float = 20.0  # seconds before giving up

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(ORBIT_TIMER_DT, self.publish_active_command)
        self.create_timer(0.2, self.publish_telemetry_status)

        self.get_logger().info('Text command bridge ready on /uav/user_command')
        self.get_logger().info(
            f'Planner integration: publish_target_goal_on_goto={self.publish_target_goal_on_goto}, '
            f'planner_mode_for_goto={self.planner_mode_for_goto}, '
            f'target_goal_topic={self.target_goal_topic}'
        )
        self.publish_status('READY', 'init')

    # ═══════════════════════════════ Callbacks ═══════════════════════════════

    def user_command_callback(self, msg: String) -> None:
        command_text = msg.data.strip()
        self.publish_status('RECEIVED', command_text)
        parsed = self.parse_command(command_text)
        if parsed is None:
            self.switch_to_hover()
            self.publish_status('UNKNOWN_COMMAND', command_text, {'fallback': 'HOVER'})
            return
        action, params = parsed
        self.apply_command(action, params, command_text)

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg
        self.has_local_position = True

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    # ═══════════════════════════ 10 Hz control loop ══════════════════════════

    def publish_active_command(self) -> None:
        if self.planner_control_active:
            if self.publish_report_pending:
                self.publish_status('PUBLISHED', self.active_command_label)
                self.publish_report_pending = False
            return

        # ── Visual search timeout ────────────────────────────────────────────
        if self._search_active and self._search_deadline_ns is not None:
            if self.get_clock().now().nanoseconds >= self._search_deadline_ns:
                self._cancel_search(reason='timeout')

        # ── Deadline check ──────────────────────────────────────────────────
        if (
            self.command_deadline_ns is not None
            and self.get_clock().now().nanoseconds >= self.command_deadline_ns
        ):
            self.switch_to_hover()
            self.publish_status('MAPPED', 'AUTO_HOVER_TIMEOUT', {'fallback': 'HOVER'})
            self.publish_report_pending = True

        # ── Select command to publish (priority order) ──────────────────────
        command_to_publish = self.active_command

        if self.takeoff_target_z_ned is not None:
            command_to_publish = self._compute_takeoff_velocity()

        elif self.orbit_center is not None and self.has_local_position:
            command_to_publish = self._compute_orbit_velocity()

        elif self.goto_target is not None and self.has_local_position:
            command_to_publish = self._compute_goto_velocity()

        elif self.yaw_target is not None and self.has_local_position:
            command_to_publish = self._compute_yaw_velocity()

        # ── Publish to move_velocity ─────────────────────────────────────────
        msg = HWSimpleKeyboardInfo()
        msg.x = float(command_to_publish.x)
        msg.y = float(command_to_publish.y)
        msg.z = float(command_to_publish.z)
        msg.yaw = float(command_to_publish.yaw)
        self.command_publisher.publish(msg)

        if self.publish_report_pending:
            self.publish_status('PUBLISHED', self.active_command_label)
            self.publish_report_pending = False

    # ═══════════════════════ Per-mode velocity computers ═════════════════════

    def _compute_takeoff_velocity(self) -> VelocityCommand:
        """ARM phase → altitude hold."""
        if self._in_arm_phase():
            return VelocityCommand(0.0, 0.0, ARM_TRIGGER_Z, 0.0, None)
        if not self.has_local_position:
            return VelocityCommand(0.0, 0.0, ARM_TRIGGER_Z, 0.0, None)
        z_error = self.takeoff_target_z_ned - self.vehicle_local_position.z  # type: ignore[operator]
        if abs(z_error) <= self.altitude_tolerance:
            return VelocityCommand(0.0, 0.0, 0.0, 0.0, None)
        z_cmd = max(-self.max_takeoff_z_speed,
                    min(self.max_takeoff_z_speed, self.altitude_kp * z_error))
        return VelocityCommand(0.0, 0.0, z_cmd, 0.0, None)

    def _compute_orbit_velocity(self) -> VelocityCommand:
        """
        Orbit around orbit_center at orbit_radius.

        Algorithm (from LLM-controlled-drone / classic UAV orbit):
          vt = tangential velocity (perpendicular to radius vector)
          vc = corrective velocity (pulls drone back onto circle)
          vz = altitude P-hold

        theta advances by angular_speed * dt each tick.
        """
        self.orbit_angle += self.orbit_angular_speed * ORBIT_TIMER_DT
        cx, cy = self.orbit_center  # type: ignore[misc]

        desired_x = cx + self.orbit_radius * math.cos(self.orbit_angle)
        desired_y = cy + self.orbit_radius * math.sin(self.orbit_angle)

        tang_vx = -self.orbit_speed * math.sin(self.orbit_angle)
        tang_vy = self.orbit_speed * math.cos(self.orbit_angle)

        corr_vx = ORBIT_KP * (desired_x - self.vehicle_local_position.x)
        corr_vy = ORBIT_KP * (desired_y - self.vehicle_local_position.y)

        z_err = self.orbit_z_ned - self.vehicle_local_position.z
        vz = max(-self.default_speed_z,
                 min(self.default_speed_z, self.altitude_kp * z_err))

        vx = max(-MAX_H_SPEED, min(MAX_H_SPEED, tang_vx + corr_vx))
        vy = max(-MAX_H_SPEED, min(MAX_H_SPEED, tang_vy + corr_vy))
        return VelocityCommand(vx, vy, vz, 0.0, None)

    def _compute_goto_velocity(self) -> VelocityCommand:
        """
        Proportional navigation to 3-D NED waypoint.
        Horizontal and vertical channels are independently P-controlled and clamped.
        When within GOTO_TOLERANCE the target is cleared and the drone hovers.
        """
        tx, ty, tz_ned = self.goto_target  # type: ignore[misc]
        ex = tx - self.vehicle_local_position.x
        ey = ty - self.vehicle_local_position.y
        ez = tz_ned - self.vehicle_local_position.z

        dist_xy = math.sqrt(ex ** 2 + ey ** 2)
        dist_3d = math.sqrt(ex ** 2 + ey ** 2 + ez ** 2)

        if dist_3d <= GOTO_TOLERANCE:
            self.goto_target = None
            self.active_command_label = 'HOVER'
            self.publish_status('TELEMETRY', 'GOTO_NED_ARRIVED', {
                'position': {'x': self.vehicle_local_position.x,
                             'y': self.vehicle_local_position.y,
                             'z': self.vehicle_local_position.z},
            })
            return VelocityCommand(0.0, 0.0, 0.0, 0.0, None)

        h_speed = min(self.default_speed_xy, GOTO_KP * dist_xy)
        if dist_xy > 0.01:
            vx = h_speed * ex / dist_xy
            vy = h_speed * ey / dist_xy
        else:
            vx = vy = 0.0
        vz = max(-self.default_speed_z,
                 min(self.default_speed_z, GOTO_KP * ez))
        return VelocityCommand(
            max(-MAX_H_SPEED, min(MAX_H_SPEED, vx)),
            max(-MAX_H_SPEED, min(MAX_H_SPEED, vy)),
            vz, 0.0, None,
        )

    def _compute_yaw_velocity(self) -> VelocityCommand:
        """P-controller: rotate to target NED heading (radians)."""
        current = float(self.vehicle_local_position.heading)
        yaw_error = self._angle_diff(self.yaw_target, current)  # type: ignore[arg-type]

        if abs(yaw_error) <= YAW_TOLERANCE:
            self.yaw_target = None
            self.active_command_label = 'HOVER'
            return VelocityCommand(0.0, 0.0, 0.0, 0.0, None)

        yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, YAW_KP * yaw_error))
        return VelocityCommand(0.0, 0.0, 0.0, yaw_rate, None)

    # ═══════════════════════════ Command parsing ══════════════════════════════

    def parse_command(self, raw_text: str) -> Optional[Tuple[str, Dict]]:
        if not raw_text:
            return None

        # JSON protocol (primary – for LLM integration)
        if raw_text.startswith('{'):
            try:
                data = json.loads(raw_text)
                action = str(data.get('action', '')).strip().upper()
                params = data.get('params', {})
                if not isinstance(params, dict) or not action:
                    return None
                return action, params
            except json.JSONDecodeError:
                return None

        tokens = raw_text.split()
        if not tokens:
            return None

        # Chinese text back-compat
        legacy = tokens[0]
        if legacy == '起飞':
            alt = self.parse_float(tokens[1], self.takeoff_altitude_default) if len(tokens) > 1 else self.takeoff_altitude_default
            return 'TAKEOFF', {'altitude': alt}
        if legacy == '起飞悬停':
            return 'TAKEOFF', {'altitude': self.takeoff_altitude_default}
        if legacy == '降落':
            return 'LAND', {}
        if legacy in ('悬停', '停止'):
            dur = self.parse_float(tokens[1], 0.0) if len(tokens) > 1 else 0.0
            return 'HOVER', {'duration': dur}
        if legacy in ('前进', '后退', '左移', '右移', '上升', '下降', '左转', '右转'):
            dur = self.parse_float(tokens[1], self.default_duration) if len(tokens) > 1 else self.default_duration
            spd = self.parse_float(tokens[2], 0.0) if len(tokens) > 2 else 0.0
            return self.map_legacy_move(legacy, dur, spd)
        if legacy in ('返航', 'RTL', 'rtl'):
            return 'RTL', {}
        if legacy in ('紧急停止', '急停', 'ESTOP'):
            return 'EMERGENCY_STOP', {}
        if legacy in ('设速',):
            spd = self.parse_float(tokens[1], self.default_speed_xy) if len(tokens) > 1 else self.default_speed_xy
            return 'SET_SPEED', {'speed': spd}

        # Generic KEY=VALUE syntax
        action = tokens[0].upper()
        params: Dict[str, float] = {}
        for token in tokens[1:]:
            if '=' not in token:
                continue
            k, v = token.split('=', 1)
            try:
                params[k] = float(v)
            except ValueError:
                pass
        return action, params

    def map_legacy_move(self, legacy: str, duration: float, speed: float) -> Tuple[str, Dict]:
        duration = max(0.1, duration)
        if legacy in ('前进', '后退'):
            vx = speed if speed > 0 else self.default_speed_xy
            if legacy == '后退':
                vx = -vx
            return 'MOVE_VELOCITY', {'vx': vx, 'vy': 0.0, 'vz': 0.0, 'yaw_rate': 0.0, 'duration': duration}
        if legacy in ('左移', '右移'):
            vy = speed if speed > 0 else self.default_speed_xy
            if legacy == '左移':
                vy = -vy
            return 'MOVE_VELOCITY', {'vx': 0.0, 'vy': vy, 'vz': 0.0, 'yaw_rate': 0.0, 'duration': duration}
        if legacy in ('上升', '下降'):
            vz = speed if speed > 0 else self.default_speed_z
            if legacy == '上升':
                vz = -vz
            return 'MOVE_VELOCITY', {'vx': 0.0, 'vy': 0.0, 'vz': vz, 'yaw_rate': 0.0, 'duration': duration}
        yaw_rate = speed if speed > 0 else self.default_yaw_rate
        if legacy == '左转':
            yaw_rate = -yaw_rate
        return 'MOVE_VELOCITY', {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'yaw_rate': yaw_rate, 'duration': duration}

    # ═══════════════════════════ Command dispatch ═════════════════════════════

    def apply_command(self, action: str, params: Dict, raw_command: str) -> None:
        action = action.upper()
        if action != 'GOTO_NED':
            self.planner_control_active = False

        # Cancel any active visual search when a new command arrives
        if self._search_active and action != 'FIND_AND_GOTO':
            self.get_logger().info(
                f'[FIND_AND_GOTO] Search for "{self._search_query}" cancelled by {action}'
            )
            self._search_active = False
            self._search_query = ''
            self._search_deadline_ns = None

        # ── TAKEOFF ──────────────────────────────────────────────────────────
        if action == 'TAKEOFF':
            altitude = abs(self.param_float(params, 'altitude', self.takeoff_altitude_default))
            self._clear_nav_state()
            self.takeoff_target_z_ned = -altitude
            self.takeoff_start_ns = self.get_clock().now().nanoseconds
            self.active_command = self.hover_command
            self.active_command_label = 'TAKEOFF'
            self.publish_status('MAPPED', raw_command, {
                'action': action, 'target_altitude': altitude,
                'target_z_ned': self.takeoff_target_z_ned,
            })
            self.publish_report_pending = True
            return

        # ── LAND ─────────────────────────────────────────────────────────────
        if action == 'LAND':
            self._clear_nav_state()
            self.active_command = VelocityCommand(0.0, 0.0, 8.0, 0.0, None)
            self.active_command_label = 'LAND'
            self.publish_status('MAPPED', raw_command,
                                {'action': action, 'mapped': {'z': 8.0}})
            self.publish_report_pending = True
            return

        # ── HOVER ─────────────────────────────────────────────────────────────
        if action == 'HOVER':
            self._clear_nav_state()
            self.active_command = self.hover_command
            self.active_command_label = 'HOVER'
            duration = max(0.0, self.param_float(params, 'duration', 0.0))
            if duration > 0.0:
                self.command_deadline_ns = (
                    self.get_clock().now().nanoseconds + int(duration * 1e9)
                )
            self.publish_status('MAPPED', raw_command, {'action': action})
            self.publish_report_pending = True
            return

        # ── MOVE_VELOCITY ─────────────────────────────────────────────────────
        if action == 'MOVE_VELOCITY':
            self._clear_nav_state()
            vx = self.param_float(params, 'vx', 0.0)
            vy = self.param_float(params, 'vy', 0.0)
            vz = self.param_float(params, 'vz', 0.0)
            yaw_rate = self.param_float(params, 'yaw_rate', 0.0)
            duration = max(0.1, self.param_float(params, 'duration', self.default_duration))

            # Convert body-frame vx/vy (forward/right) to NED world-frame using
            # current drone heading.  This makes "前进" always go toward the
            # drone's nose regardless of which direction it faces.
            if self.has_local_position:
                h = float(self.vehicle_local_position.heading)
                cos_h = math.cos(h)
                sin_h = math.sin(h)
                vx_ned = cos_h * vx - sin_h * vy
                vy_ned = sin_h * vx + cos_h * vy
                vx, vy = vx_ned, vy_ned

            self.active_command = VelocityCommand(vx, vy, vz, yaw_rate, duration)
            self.active_command_label = 'MOVE_VELOCITY'
            self.command_deadline_ns = (
                self.get_clock().now().nanoseconds + int(duration * 1e9)
            )
            self.publish_status('MAPPED', raw_command, {
                'action': action,
                'mapped': {'x': vx, 'y': vy, 'z': vz, 'yaw': yaw_rate},
                'duration': duration,
            })
            self.publish_report_pending = True
            return

        # ── MOVE_REL ──────────────────────────────────────────────────────────
        if action == 'MOVE_REL':
            if not self.has_local_position:
                self.publish_status('UNKNOWN_COMMAND', raw_command,
                                    {'reason': 'NO_LOCAL_POSITION', 'fallback': 'HOVER'})
                self.switch_to_hover()
                return
            dx = self.param_float(params, 'dx', 0.0)
            dy = self.param_float(params, 'dy', 0.0)
            dz = self.param_float(params, 'dz', 0.0)
            duration = max(0.2, self.param_float(params, 'duration', 2.0))
            self.apply_command(
                'MOVE_VELOCITY',
                {'vx': dx / duration, 'vy': dy / duration,
                 'vz': dz / duration, 'yaw_rate': 0.0, 'duration': duration},
                raw_command,
            )
            return

        # ── RTL (Return to Launch) ────────────────────────────────────────────
        if action == 'RTL':
            self.switch_to_hover()  # stop velocity stream first
            self.active_command_label = 'RTL'
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            self.publish_status('MAPPED', raw_command, {'action': action})
            self.publish_report_pending = True
            return

        # ── EMERGENCY_STOP ────────────────────────────────────────────────────
        if action == 'EMERGENCY_STOP':
            self.switch_to_hover()
            self.active_command_label = 'EMERGENCY_STOP'
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
            )
            self.publish_status('MAPPED', raw_command, {'action': action})
            self.publish_report_pending = True
            return

        # ── SET_SPEED ─────────────────────────────────────────────────────────
        if action == 'SET_SPEED':
            speed = abs(self.param_float(params, 'speed', self.default_speed_xy))
            speed = max(0.5, min(MAX_H_SPEED, speed))
            self.default_speed_xy = speed
            self.default_speed_z = max(0.5, speed * 0.6)
            self.active_command_label = f'SET_SPEED({speed:.1f})'
            self.publish_status('MAPPED', raw_command, {
                'action': action,
                'speed_xy': self.default_speed_xy,
                'speed_z': self.default_speed_z,
            })
            self.publish_report_pending = True
            return

        # ── YAW_TO ────────────────────────────────────────────────────────────
        if action == 'YAW_TO':
            angle_deg = self.param_float(params, 'angle', 0.0)
            self._clear_nav_state()
            self.yaw_target = math.radians(angle_deg % 360.0)
            self.active_command = self.hover_command
            self.active_command_label = 'YAW_TO'
            self.publish_status('MAPPED', raw_command, {
                'action': action, 'target_deg': angle_deg,
                'target_rad': self.yaw_target,
            })
            self.publish_report_pending = True
            return

        # ── GOTO_NED ──────────────────────────────────────────────────────────
        if action == 'GOTO_NED':
            if not self.has_local_position and (
                'x' not in params or 'y' not in params or 'altitude' not in params
            ):
                self.publish_status('UNKNOWN_COMMAND', raw_command,
                                    {'reason': 'NO_LOCAL_POSITION', 'fallback': 'HOVER'})
                self.switch_to_hover()
                return
            current_x = float(self.vehicle_local_position.x) if self.has_local_position else 0.0
            current_y = float(self.vehicle_local_position.y) if self.has_local_position else 0.0
            current_alt = float(-self.vehicle_local_position.z) if self.has_local_position else 0.0
            tx = self.param_float(params, 'x', current_x)
            ty = self.param_float(params, 'y', current_y)
            alt = self.param_float(params, 'altitude', current_alt)
            tz_ned = -alt  # ENU altitude → NED z

            if self.publish_target_goal_on_goto:
                self._publish_target_goal(tx, ty, alt)

            if self.planner_mode_for_goto:
                self._clear_nav_state()
                self.planner_control_active = True
                self.active_command = self.hover_command
                self.active_command_label = 'PLANNER_GOTO'
                self.publish_status('MAPPED', raw_command, {
                    'action': action,
                    'planner_mode': True,
                    'target': {'x': tx, 'y': ty, 'altitude': alt, 'z_ned': tz_ned},
                    'target_goal_topic': self.target_goal_topic,
                })
                self.publish_report_pending = True
                return

            self._clear_nav_state()
            self.goto_target = (tx, ty, tz_ned)
            self.active_command = self.hover_command
            self.active_command_label = 'GOTO_NED'
            self.publish_status('MAPPED', raw_command, {
                'action': action,
                'target': {'x': tx, 'y': ty, 'altitude': alt, 'z_ned': tz_ned},
            })
            self.publish_report_pending = True
            return

        # ── ORBIT ─────────────────────────────────────────────────────────────
        if action == 'ORBIT':
            if not self.has_local_position:
                self.publish_status('UNKNOWN_COMMAND', raw_command,
                                    {'reason': 'NO_LOCAL_POSITION', 'fallback': 'HOVER'})
                self.switch_to_hover()
                return
            cx = self.param_float(params, 'cx', self.vehicle_local_position.x)
            cy = self.param_float(params, 'cy', self.vehicle_local_position.y)
            radius = max(1.0, self.param_float(params, 'radius', 5.0))
            speed = max(0.5, self.param_float(params, 'speed', self.default_speed_xy * 0.5))
            duration = self.param_float(params, 'duration', 0.0)

            self._clear_nav_state()
            self.orbit_center = (cx, cy)
            self.orbit_radius = radius
            self.orbit_speed = speed
            self.orbit_angular_speed = speed / radius
            # Init angle from current drone position relative to center
            dx = self.vehicle_local_position.x - cx
            dy = self.vehicle_local_position.y - cy
            self.orbit_angle = math.atan2(dy, dx)
            self.orbit_z_ned = self.vehicle_local_position.z
            self.active_command = self.hover_command
            self.active_command_label = 'ORBIT'
            if duration > 0.0:
                self.command_deadline_ns = (
                    self.get_clock().now().nanoseconds + int(duration * 1e9)
                )
            self.publish_status('MAPPED', raw_command, {
                'action': action,
                'center': {'x': cx, 'y': cy},
                'radius': radius, 'speed': speed, 'duration': duration,
            })
            self.publish_report_pending = True
            return

        # ── FIND_AND_GOTO ─────────────────────────────────────────────────────
        if action == 'FIND_AND_GOTO':
            query = str(params.get('query', '')).strip()
            if not query:
                self.publish_status('UNKNOWN_COMMAND', raw_command,
                                    {'reason': 'EMPTY_VISUAL_QUERY', 'fallback': 'HOVER'})
                self.switch_to_hover()
                return
            # Stop any existing navigation, hover while searching
            self._clear_nav_state()
            self._search_active = True
            self._search_query = query
            self._search_deadline_ns = (
                self.get_clock().now().nanoseconds + int(self._search_timeout_sec * 1e9)
            )
            self.active_command = self.hover_command
            self.active_command_label = 'SEARCHING'
            # Publish search query to YOLO-World detector
            q_msg = String()
            q_msg.data = query
            self._target_query_pub.publish(q_msg)
            self.publish_status('MAPPED', raw_command, {
                'action': action,
                'query': query,
                'timeout_sec': self._search_timeout_sec,
            })
            self.get_logger().info(
                f'[FIND_AND_GOTO] Searching for "{query}" '
                f'(timeout={self._search_timeout_sec:.0f}s)'
            )
            self.publish_report_pending = True
            return

        # ── Unknown ───────────────────────────────────────────────────────────
        self.publish_status('UNKNOWN_COMMAND', raw_command,
                            {'reason': f'UNSUPPORTED_ACTION_{action}', 'fallback': 'HOVER'})
        self.switch_to_hover()

    # ═══════════════════════════ State management ════════════════════════════

    def _clear_nav_state(self) -> None:
        """Clear all continuous-nav state without resetting active_command."""
        self.takeoff_target_z_ned = None
        self.takeoff_start_ns = None
        self.goto_target = None
        self.orbit_center = None
        self.yaw_target = None
        self.command_deadline_ns = None

    def switch_to_hover(self) -> None:
        self._clear_nav_state()
        self.active_command = self.hover_command
        self.active_command_label = 'IDLE'
        self.publish_report_pending = True

    def _cancel_search(self, reason: str = 'cancelled') -> None:
        """Cancel an in-progress visual search and revert to HOVER."""
        query = self._search_query
        self._search_active = False
        self._search_query = ''
        self._search_deadline_ns = None
        self.publish_status('SEARCH_FAILED', 'FIND_AND_GOTO', {
            'query': query, 'reason': reason,
        })
        if reason == 'timeout':
            self.get_logger().warn(
                f'[FIND_AND_GOTO] Timeout — target "{query}" not found within '
                f'{self._search_timeout_sec:.0f}s, reverting to HOVER'
            )
            self.switch_to_hover()

    # ─────────────────────── Visual semantic search callback ─────────────────

    def _on_semantic_targets_world(self, msg: String) -> None:
        """Handle detections from semantic_target_tf_node.

        Called when YOLO-World + grounding + TF pipeline publishes a world-frame
        target list on /uav/semantic_targets_world.  If a FIND_AND_GOTO search
        is active, selects the best detection and transitions to GOTO_NED.
        """
        if not self._search_active:
            return  # no active search — ignore detections

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'[FIND_AND_GOTO] JSON parse error: {exc}')
            return

        targets = data.get('targets', [])
        if not targets:
            return  # wait for the next frame

        # Select highest-confidence target
        best = max(targets, key=lambda t: float(t.get('score', 0.0)))
        score = float(best.get('score', 0.0))
        if score < 0.15:
            return  # confidence too low — keep searching

        # ── Coordinate conversion: ENU world → NED for GOTO_NED ──────────────
        # semantic_target_tf_node outputs ENU: x_world=East, y_world=North, z_world=Up
        # GOTO_NED uses:  x=North, y=East, altitude=positive_up
        x_enu = float(best.get('x_world', 0.0))  # East
        y_enu = float(best.get('y_world', 0.0))  # North
        z_enu = float(best.get('z_world', 0.0))  # Up

        x_ned = y_enu   # North
        y_ned = x_enu   # East

        # Use current drone altitude so we fly to the target's XY but maintain
        # our height (prevents flying into the ground when target is at z≈0).
        if self.has_local_position:
            current_alt = max(2.0, -float(self.vehicle_local_position.z))
        else:
            current_alt = 6.0  # fallback

        label = str(best.get('label', '?'))
        depth = float(best.get('depth_m', 0.0))
        query = self._search_query

        self.get_logger().info(
            f'[FIND_AND_GOTO] Found "{label}" score={score:.2f} depth={depth:.1f}m '
            f'-> GOTO_NED x={x_ned:.2f} y={y_ned:.2f} alt={current_alt:.1f}m'
        )

        # Clear search state BEFORE calling apply_command
        self._search_active = False
        self._search_query = ''
        self._search_deadline_ns = None

        self.publish_status('SEARCH_FOUND', 'FIND_AND_GOTO', {
            'query': query,
            'label': label,
            'score': score,
            'depth_m': depth,
            'target_ned': {'x': x_ned, 'y': y_ned, 'altitude': current_alt},
        })

        # Transition to GOTO_NED
        self.apply_command(
            'GOTO_NED',
            {'x': x_ned, 'y': y_ned, 'altitude': current_alt},
            f'FIND_AND_GOTO({query})->GOTO_NED({label})',
        )

    # ═══════════════════════════ Telemetry ═══════════════════════════════════

    def publish_telemetry_status(self) -> None:
        """
        Rich telemetry message for LLM context (follows EchoPilot / Lim 2025
        pattern: position + velocity + heading + active-mode context).
        """
        pos = vel = heading_deg = None
        goto_dist = orbit_progress = None

        if self.has_local_position:
            lp = self.vehicle_local_position
            pos = {'x': round(float(lp.x), 2),
                   'y': round(float(lp.y), 2),
                   'z': round(float(lp.z), 2)}
            vel = {'vx': round(float(lp.vx), 2),
                   'vy': round(float(lp.vy), 2),
                   'vz': round(float(lp.vz), 2)}
            heading_deg = round(math.degrees(float(lp.heading)) % 360, 1)

            if self.goto_target is not None:
                tx, ty, tz = self.goto_target
                goto_dist = round(math.sqrt(
                    (tx - lp.x) ** 2 + (ty - lp.y) ** 2 + (tz - lp.z) ** 2
                ), 2)

            if self.orbit_center is not None:
                orbit_progress = round(
                    math.degrees(self.orbit_angle) % 360, 1
                )

        extra: Dict = {
            'position': pos,
            'velocity': vel,
            'heading_deg': heading_deg,
            'arming_state': int(self.vehicle_status.arming_state),
            'nav_state': int(self.vehicle_status.nav_state),
            'target_z_ned': self.takeoff_target_z_ned,
            'default_speed_xy': self.default_speed_xy,
        }
        if goto_dist is not None:
            extra['goto_distance_m'] = goto_dist
        if orbit_progress is not None:
            extra['orbit_angle_deg'] = orbit_progress
        if self.goto_target is not None:
            extra['goto_target'] = {
                'x': self.goto_target[0], 'y': self.goto_target[1],
                'altitude': -self.goto_target[2],
            }
        if self._search_active and self._search_deadline_ns is not None:
            remaining = (self._search_deadline_ns - self.get_clock().now().nanoseconds) / 1e9
            extra['searching_query'] = self._search_query
            extra['search_remaining_sec'] = round(max(0.0, remaining), 1)

        self.publish_status('TELEMETRY', self.active_command_label, extra)

    # ═══════════════════════════ Helpers ═════════════════════════════════════

    def _in_arm_phase(self) -> bool:
        if self.takeoff_start_ns is None:
            return False
        return (self.get_clock().now().nanoseconds - self.takeoff_start_ns) / 1e9 < ARM_TRIGGER_SEC

    def _publish_vehicle_command(self, command: int, **kw) -> None:
        """Direct PX4 command (RTL, EMERGENCY_STOP only – keep path short)."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(kw.get('param1', 0.0))
        msg.param2 = float(kw.get('param2', 0.0))
        msg.param3 = float(kw.get('param3', 0.0))
        msg.param4 = float(kw.get('param4', 0.0))
        msg.param5 = float(kw.get('param5', 0.0))
        msg.param6 = float(kw.get('param6', 0.0))
        msg.param7 = float(kw.get('param7', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def _publish_target_goal(self, x_north: float, y_east: float, altitude: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x_north)
        msg.pose.position.y = float(y_east)
        msg.pose.position.z = float(altitude)
        msg.pose.orientation.w = 1.0
        self.target_goal_pub.publish(msg)

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        """Shortest signed angular difference in [-π, π]."""
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def param_float(self, params: Dict, key: str, default: float) -> float:
        try:
            return float(params.get(key, default))
        except (TypeError, ValueError):
            return float(default)

    def parse_float(self, text: str, default: float) -> float:
        try:
            return float(text)
        except ValueError:
            return default

    def publish_status(self, status: str, command: str,
                       extra: Optional[Dict] = None) -> None:
        payload: Dict = {
            'status': status,
            'command': command,
            'ts': self.get_clock().now().nanoseconds,
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TextCommandBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(exc)
