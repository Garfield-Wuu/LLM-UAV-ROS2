#!/usr/bin/env python3
"""
Closed-loop flight regression runner.

Each step sends a command and then waits for a real-world condition to be met
(e.g. altitude reached, motion duration elapsed, landing detected) before
marking it PASS.  A simple "PUBLISHED" acknowledgement is NOT enough.
"""

import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus


@dataclass
class StepResult:
    command: str
    passed: bool = False
    latency_ms: float = 0.0
    detail: str = 'timeout'
    checks: List[str] = field(default_factory=list)


ARMING_DISARMED = 1
ARMING_ARMED = 2
NAV_OFFBOARD = 14
NAV_AUTO_LAND = 18


class FlightRegressionRunner(Node):
    def __init__(self) -> None:
        super().__init__('flight_regression_runner')

        self.declare_parameter('sequence', [
            '{"action":"TAKEOFF","params":{"altitude":6.0}}',
            '{"action":"HOVER","params":{"duration":3.0}}',
            '{"action":"MOVE_VELOCITY","params":{"vx":2.0,"vy":0.0,"vz":0.0,"yaw_rate":0.0,"duration":3.0}}',
            '{"action":"HOVER","params":{"duration":3.0}}',
            '{"action":"LAND","params":{}}',
        ])
        self.declare_parameter('step_timeout_sec', 25.0)
        self.declare_parameter('alt_tolerance', 0.6)
        self.declare_parameter('land_z_threshold', -0.3)
        self.declare_parameter('inter_step_wait_sec', 1.5)

        self.sequence: List[str] = [str(s) for s in self.get_parameter('sequence').value]
        self.step_timeout = float(self.get_parameter('step_timeout_sec').value)
        self.alt_tol = float(self.get_parameter('alt_tolerance').value)
        self.land_z = float(self.get_parameter('land_z_threshold').value)
        self.inter_wait = float(self.get_parameter('inter_step_wait_sec').value)

        self.cmd_pub = self.create_publisher(String, '/uav/user_command', 10)
        self.create_subscription(String, '/uav/llm_task_status', self.on_status, 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_pos, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.on_vs, qos)

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.has_pos = False
        self.arm_state = 0
        self.nav_state = 0
        self.published_ack = False
        self.results: List[StepResult] = []

    def on_status(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get('status') == 'PUBLISHED':
            self.published_ack = True
        if d.get('status') == 'TELEMETRY':
            p = d.get('position')
            if isinstance(p, dict):
                self.pos_x = float(p.get('x', self.pos_x))
                self.pos_y = float(p.get('y', self.pos_y))
                self.pos_z = float(p.get('z', self.pos_z))
                self.has_pos = True
            if d.get('arming_state') is not None:
                self.arm_state = int(d['arming_state'])
            if d.get('nav_state') is not None:
                self.nav_state = int(d['nav_state'])

    def on_pos(self, msg: VehicleLocalPosition) -> None:
        self.pos_x = float(msg.x)
        self.pos_y = float(msg.y)
        self.pos_z = float(msg.z)
        self.has_pos = True

    def on_vs(self, msg: VehicleStatus) -> None:
        self.arm_state = int(msg.arming_state)
        self.nav_state = int(msg.nav_state)

    def spin_for(self, sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, end - time.monotonic()))

    def send(self, cmd: str) -> None:
        m = String()
        m.data = cmd
        self.published_ack = False
        self.cmd_pub.publish(m)

    def wait_published(self, timeout: float = 5.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.published_ack:
                return True
        return False

    def run(self) -> int:
        self.get_logger().info('=== Closed-loop flight regression START ===')
        self.spin_for(1.0)

        for idx, cmd_str in enumerate(self.sequence, 1):
            res = self.execute_step(idx, cmd_str)
            self.results.append(res)
            if not res.passed:
                self.get_logger().warn(f'STEP {idx} FAILED — aborting remaining steps.')
                break
            self.spin_for(self.inter_wait)

        return self.print_summary()

    def execute_step(self, idx: int, cmd_str: str) -> StepResult:
        self.get_logger().info(f'STEP {idx}: {cmd_str}')
        res = StepResult(command=cmd_str)
        t0 = time.monotonic()

        try:
            data = json.loads(cmd_str)
        except json.JSONDecodeError:
            res.detail = 'BAD_JSON'
            return res
        action = data.get('action', '').upper()
        params = data.get('params', {})

        self.send(cmd_str)

        if not self.wait_published(timeout=5.0):
            res.detail = 'NO_PUBLISH_ACK'
            res.latency_ms = (time.monotonic() - t0) * 1000
            return res
        res.checks.append('PUBLISH_ACK')

        if action == 'TAKEOFF':
            res = self.verify_takeoff(res, params, t0)
        elif action == 'LAND':
            res = self.verify_land(res, t0)
        elif action == 'MOVE_VELOCITY':
            res = self.verify_move(res, params, t0)
        elif action == 'HOVER':
            res = self.verify_hover(res, params, t0)
        else:
            res.passed = True
            res.detail = f'PUBLISH_ACK (no closed-loop check for {action})'

        res.latency_ms = (time.monotonic() - t0) * 1000
        return res

    def verify_takeoff(self, res: StepResult, params: dict, t0: float) -> StepResult:
        target_alt = abs(float(params.get('altitude', 6.0)))
        target_z_ned = -target_alt
        deadline = t0 + self.step_timeout

        armed = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.arm_state == ARMING_ARMED:
                if not armed:
                    res.checks.append('ARMED')
                    armed = True
                alt_err = abs(self.pos_z - target_z_ned)
                if alt_err <= self.alt_tol:
                    res.checks.append(f'ALT_REACHED z={self.pos_z:.2f} tgt={target_z_ned:.2f}')
                    res.passed = True
                    res.detail = f'Alt OK ({-self.pos_z:.1f}m)'
                    return res

        if not armed:
            res.detail = 'NEVER_ARMED'
        else:
            res.detail = f'ALT_NOT_REACHED z={self.pos_z:.2f} tgt={target_z_ned:.2f}'
        return res

    def verify_land(self, res: StepResult, t0: float) -> StepResult:
        deadline = t0 + self.step_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pos_z > self.land_z or self.arm_state == ARMING_DISARMED:
                res.checks.append(f'ON_GROUND z={self.pos_z:.2f} arm={self.arm_state}')
                res.passed = True
                res.detail = f'Landed z={self.pos_z:.2f}'
                return res

        res.detail = f'NOT_LANDED z={self.pos_z:.2f}'
        return res

    def verify_move(self, res: StepResult, params: dict, t0: float) -> StepResult:
        duration = max(0.5, float(params.get('duration', 2.0)))
        x0, y0, z0 = self.pos_x, self.pos_y, self.pos_z
        vx = float(params.get('vx', 0))
        vy = float(params.get('vy', 0))
        vz = float(params.get('vz', 0))
        expected_speed = math.sqrt(vx * vx + vy * vy + vz * vz)

        self.spin_for(duration + 0.5)

        dx = self.pos_x - x0
        dy = self.pos_y - y0
        dz = self.pos_z - z0
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        min_dist = expected_speed * duration * 0.25 if expected_speed > 0.1 else 0.0
        res.checks.append(f'DISPLACEMENT d={dist:.2f} min={min_dist:.2f}')

        if dist >= min_dist:
            res.passed = True
            res.detail = f'Moved {dist:.2f}m in {duration:.1f}s'
        else:
            res.detail = f'INSUFFICIENT_MOVE d={dist:.2f} < min={min_dist:.2f}'
        return res

    def verify_hover(self, res: StepResult, params: dict, t0: float) -> StepResult:
        duration = max(0.5, float(params.get('duration', 2.0)))
        x0, y0 = self.pos_x, self.pos_y

        self.spin_for(duration)

        drift = math.sqrt((self.pos_x - x0) ** 2 + (self.pos_y - y0) ** 2)
        res.checks.append(f'HOVER_DRIFT={drift:.2f}m')

        if self.arm_state == ARMING_ARMED:
            res.passed = True
            res.detail = f'Hover OK drift={drift:.2f}m'
        else:
            res.detail = 'DISARMED_DURING_HOVER'
        return res

    def print_summary(self) -> int:
        sep = '=' * 76
        print(f'\n{sep}')
        print('  Closed-Loop Flight Regression Summary')
        print(sep)
        print(f'  {"#":<3} {"Result":<6} {"Time(s)":<9} {"Action":<16} Detail')
        print('-' * 76)
        ok = True
        for i, r in enumerate(self.results, 1):
            tag = 'PASS' if r.passed else 'FAIL'
            try:
                act = json.loads(r.command).get('action', '?')
            except Exception:
                act = '?'
            t_sec = r.latency_ms / 1000.0
            print(f'  {i:<3} {tag:<6} {t_sec:<9.1f} {act:<16} {r.detail}')
            checks_str = ', '.join(r.checks) if r.checks else '-'
            print(f'      checks: {checks_str}')
            if not r.passed:
                ok = False
        print('-' * 76)
        overall = 'PASS' if ok else 'FAIL'
        print(f'  Overall: {overall}')
        print(f'{sep}\n')
        return 0 if ok else 1


def main(args=None) -> None:
    rclpy.init(args=args)
    runner = FlightRegressionRunner()
    code = runner.run()
    runner.destroy_node()
    rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
