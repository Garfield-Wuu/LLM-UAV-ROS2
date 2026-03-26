#!/usr/bin/env python3

import json
from datetime import datetime
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus


ARMING_MAP = {1: 'DISARMED', 2: 'ARMED'}
NAV_MAP = {
    0: 'MANUAL', 2: 'POSCTL', 3: 'MISSION', 4: 'LOITER',
    5: 'RTL', 14: 'OFFBOARD', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND',
}
RST = '\033[0m'
B = '\033[1m'
R = '\033[31m'
G = '\033[32m'
Y = '\033[33m'
C = '\033[36m'
DIM = '\033[2m'

W = 72


class GCSDashboard(Node):
    def __init__(self) -> None:
        super().__init__('gcs_dashboard')

        self.declare_parameter('refresh_rate_hz', 4.0)
        self.declare_parameter('event_rows', 6)
        self.declare_parameter('clear_screen', True)
        self.declare_parameter('use_color', True)
        self.declare_parameter('stale_timeout_sec', 1.5)

        hz = max(1.0, float(self.get_parameter('refresh_rate_hz').value))
        self.max_events = max(2, int(self.get_parameter('event_rows').value))
        self.clear = bool(self.get_parameter('clear_screen').value)
        self.color = bool(self.get_parameter('use_color').value)
        self.stale_sec = float(self.get_parameter('stale_timeout_sec').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(String, '/uav/llm_task_status', self.on_status, 10)
        self.create_subscription(String, '/uav/user_command', self.on_cmd, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_pos, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.on_vs, qos)
        self.create_subscription(Range, '/airsim_node/PX4/distance/DistanceDown', self.on_dist, 1)

        self.pos = [0.0, 0.0, 0.0]
        self.arm: int = 0
        self.nav: int = 0
        self.dist_down: Optional[float] = None
        self.target_z: Optional[float] = None
        self.last_cmd = '-'
        self.last_action = 'IDLE'
        self.tele_wall: Optional[int] = None
        self.events: List[str] = []

        self.create_timer(1.0 / hz, self.render)

    def on_status(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        st = d.get('status', '')
        if st == 'TELEMETRY':
            self.tele_wall = self.get_clock().now().nanoseconds
            p = d.get('position')
            if isinstance(p, dict):
                self.pos = [float(p.get('x', 0)), float(p.get('y', 0)), float(p.get('z', 0))]
            if d.get('arming_state') is not None:
                self.arm = int(d['arming_state'])
            if d.get('nav_state') is not None:
                self.nav = int(d['nav_state'])
            if d.get('target_z_ned') is not None:
                try:
                    self.target_z = float(d['target_z_ned'])
                except (TypeError, ValueError):
                    pass
            else:
                self.target_z = d.get('target_z_ned')
            cmd = d.get('command', '')
            if cmd:
                self.last_action = cmd
            return
        cmd = d.get('command', '-')
        self.stamp_event(f'{st}: {cmd}')

    def on_cmd(self, msg: String) -> None:
        self.last_cmd = msg.data.strip() or '-'
        self.stamp_event(f'CMD > {self.last_cmd}')

    def on_pos(self, msg: VehicleLocalPosition) -> None:
        self.pos = [float(msg.x), float(msg.y), float(msg.z)]

    def on_vs(self, msg: VehicleStatus) -> None:
        self.arm = int(msg.arming_state)
        self.nav = int(msg.nav_state)

    def on_dist(self, msg: Range) -> None:
        self.dist_down = float(msg.range)

    def stamp_event(self, text: str) -> None:
        t = datetime.now().strftime('%H:%M:%S')
        self.events.append(f'{t} {text}')
        if len(self.events) > self.max_events:
            self.events = self.events[-self.max_events:]

    def s(self, text: str, c: str) -> str:
        return f'{c}{text}{RST}' if self.color else text

    def flight_phase(self) -> str:
        if self.arm == 1:
            return self.s('GROUND', DIM)
        nav_name = NAV_MAP.get(self.nav, f'NAV_{self.nav}')
        if self.nav == 14:
            if self.last_action == 'TAKEOFF':
                return self.s('TAKEOFF', Y)
            if self.last_action == 'LAND':
                return self.s('LANDING', Y)
            if self.last_action in ('MOVE_VELOCITY', 'MOVE_REL'):
                return self.s('MOVING', C)
            return self.s('OFFBOARD', G)
        if self.nav == 18:
            return self.s('LANDING', Y)
        return self.s(nav_name, DIM)

    def link_tag(self) -> str:
        if self.tele_wall is None:
            return self.s('NO_LINK', R)
        age = (self.get_clock().now().nanoseconds - self.tele_wall) / 1e9
        if age > self.stale_sec:
            return self.s('STALE', R)
        return self.s('LIVE', G)

    def render(self) -> None:
        L: List[str] = []
        now = datetime.now().strftime('%H:%M:%S')
        L.append(self.s(f' UAV Ground Station  {now} ', B + C))
        L.append('=' * W)

        arm_s = ARMING_MAP.get(self.arm, '?')
        arm_c = G if self.arm == 2 else DIM
        nav_s = NAV_MAP.get(self.nav, f'NAV_{self.nav}')
        phase = self.flight_phase()
        link = self.link_tag()

        L.append(
            f' Link {link}  '
            f'Arm {self.s(arm_s, arm_c)}  '
            f'Nav {self.s(nav_s, C)}  '
            f'Phase {phase}'
        )
        L.append('-' * W)

        alt_enu = -self.pos[2]
        tgt_str = '-'
        err_str = '-'
        bar = ''
        if self.target_z is not None:
            tgt_enu = -self.target_z
            err = tgt_enu - alt_enu
            tgt_str = f'{tgt_enu:.1f}'
            err_str = f'{err:+.2f}'
            bar = self.alt_bar(err)

        L.append(
            f' X {self.pos[0]:>7.2f}  '
            f'Y {self.pos[1]:>7.2f}  '
            f'Alt {alt_enu:>6.2f}  '
            f'Tgt {tgt_str:>5}  '
            f'Err {err_str:>6}  '
            f'GND {"-" if self.dist_down is None else f"{self.dist_down:.1f}":>4}'
        )
        if bar:
            L.append(f' {bar}')
        L.append('-' * W)

        cmd_short = self.last_cmd[:50]
        L.append(f' Cmd: {cmd_short}')
        L.append(f' Act: {self.last_action}')
        L.append('-' * W)

        for ev in self.events[-self.max_events:]:
            L.append(f' {self.s(ev, DIM)}')
        if not self.events:
            L.append(f' {self.s("(no events)", DIM)}')
        L.append('=' * W)

        out = '\n'.join(L)
        if self.clear:
            print('\033[2J\033[H', end='')
        print(out, flush=True)

    def alt_bar(self, err: float) -> str:
        bw = 40
        clamped = max(-5.0, min(5.0, err))
        idx = int((clamped + 5.0) / 10.0 * (bw - 1))
        center = bw // 2
        chars = [' '] * bw
        chars[center] = '|'
        chars[idx] = '*'
        bar = ''.join(chars)
        if abs(err) <= 0.35:
            tag = self.s('OK', G)
        else:
            tag = self.s('ADJ', Y)
        return f'[-5 {bar} +5] {tag}'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GCSDashboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(exc)
