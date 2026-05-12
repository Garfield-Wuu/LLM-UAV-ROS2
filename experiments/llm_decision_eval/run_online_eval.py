#!/usr/bin/env python3
"""Online closed-loop evaluator for thesis Section 5.4."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from common import (
    direct_parse,
    flatten_command,
    robust_parse,
    validate_baseline_command,
    validate_proposed_command,
)


ARMING_DISARMED = 1
ARMING_ARMED = 2
NAV_AUTO_LAND = 18


class OnlineEvalRunner(Node):
    def __init__(self) -> None:
        super().__init__("llm_online_eval_runner")
        self.cmd_pub = self.create_publisher(String, "/uav/user_command", 10)
        self.create_subscription(String, "/uav/llm_task_status", self.on_status, 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.on_pos, qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.on_vs, qos)

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.heading_deg = 0.0
        self.has_pos = False
        self.arm_state = 0
        self.nav_state = 0
        self.published_ack = False
        self.last_status: Dict[str, Any] = {}

    def on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_status = data
        if data.get("status") == "PUBLISHED":
            self.published_ack = True
        if data.get("status") == "TELEMETRY":
            pos = data.get("position")
            if isinstance(pos, dict) and pos.get("x") is not None:
                self.pos_x = float(pos.get("x", self.pos_x))
                self.pos_y = float(pos.get("y", self.pos_y))
                self.pos_z = float(pos.get("z", self.pos_z))
                self.has_pos = True
            if data.get("heading_deg") is not None:
                self.heading_deg = float(data["heading_deg"])
            if data.get("arming_state") is not None:
                self.arm_state = int(data["arming_state"])
            if data.get("nav_state") is not None:
                self.nav_state = int(data["nav_state"])

    def on_pos(self, msg: VehicleLocalPosition) -> None:
        self.pos_x = float(msg.x)
        self.pos_y = float(msg.y)
        self.pos_z = float(msg.z)
        self.heading_deg = math.degrees(float(msg.heading))
        self.has_pos = True

    def on_vs(self, msg: VehicleStatus) -> None:
        self.arm_state = int(msg.arming_state)
        self.nav_state = int(msg.nav_state)

    def spin_for(self, sec: float) -> None:
        deadline = time.monotonic() + sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))

    def wait_for_position(self, timeout_sec: float = 8.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.has_pos:
                return True
        return False

    def send(self, command: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(command, ensure_ascii=False)
        self.published_ack = False
        self.cmd_pub.publish(msg)

    def wait_published(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.published_ack:
                return True
        return False

    @staticmethod
    def _require_float(params: Dict[str, Any], key: str) -> Optional[float]:
        if key not in params:
            return None
        try:
            return float(params[key])
        except (TypeError, ValueError):
            return None

    def verify_takeoff(self, params: Dict[str, Any], timeout_sec: float = 25.0) -> bool:
        alt = self._require_float(params, "altitude")
        if alt is None:
            return False
        target_alt = abs(alt)
        target_z_ned = -target_alt
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.arm_state == ARMING_ARMED and abs(self.pos_z - target_z_ned) <= 0.6:
                return True
        return False

    def verify_land(self, timeout_sec: float = 25.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pos_z > -0.3 or self.arm_state == ARMING_DISARMED or self.nav_state == NAV_AUTO_LAND:
                return True
        return False

    def verify_hover(self, params: Dict[str, Any]) -> bool:
        duration = self._require_float(params, "duration")
        if duration is None:
            duration = 2.0
        duration = max(0.5, duration)
        x0, y0 = self.pos_x, self.pos_y
        self.spin_for(duration)
        drift = math.sqrt((self.pos_x - x0) ** 2 + (self.pos_y - y0) ** 2)
        return self.arm_state == ARMING_ARMED and drift <= 2.5

    def verify_move_velocity(self, params: Dict[str, Any]) -> bool:
        duration = self._require_float(params, "duration")
        if duration is None:
            return False
        duration = max(0.5, duration)
        vx = float(params.get("vx", 0.0))
        vy = float(params.get("vy", 0.0))
        vz = float(params.get("vz", 0.0))
        expected_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        x0, y0, z0 = self.pos_x, self.pos_y, self.pos_z
        self.spin_for(duration + 0.5)
        dist = math.sqrt((self.pos_x - x0) ** 2 + (self.pos_y - y0) ** 2 + (self.pos_z - z0) ** 2)
        min_dist = expected_speed * duration * 0.25 if expected_speed > 0.1 else 0.0
        return dist >= min_dist

    def verify_move_rel(self, params: Dict[str, Any]) -> bool:
        duration = self._require_float(params, "duration")
        if duration is None:
            return False
        duration = max(0.5, duration)
        expected_dist = math.sqrt(
            float(params.get("dx", 0.0)) ** 2
            + float(params.get("dy", 0.0)) ** 2
            + float(params.get("dz", 0.0)) ** 2
        )
        x0, y0, z0 = self.pos_x, self.pos_y, self.pos_z
        self.spin_for(duration + 0.5)
        dist = math.sqrt((self.pos_x - x0) ** 2 + (self.pos_y - y0) ** 2 + (self.pos_z - z0) ** 2)
        return dist >= expected_dist * 0.25

    def verify_goto(self, params: Dict[str, Any], timeout_sec: float = 35.0) -> bool:
        tx = self._require_float(params, "x")
        ty = self._require_float(params, "y")
        altitude = self._require_float(params, "altitude")
        if tx is None or ty is None or altitude is None:
            return False
        tz = -abs(altitude)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            dist = math.sqrt((self.pos_x - tx) ** 2 + (self.pos_y - ty) ** 2 + (self.pos_z - tz) ** 2)
            if dist <= 0.8:
                return True
        return False

    def verify_yaw(self, params: Dict[str, Any], timeout_sec: float = 15.0) -> bool:
        target = self._require_float(params, "angle")
        if target is None:
            return False
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            diff = ((self.heading_deg - target + 180.0) % 360.0) - 180.0
            if abs(diff) <= 10.0:
                return True
        return False

    def verify_orbit(self, params: Dict[str, Any]) -> bool:
        for key in ("cx", "cy", "radius", "speed", "duration"):
            if self._require_float(params, key) is None:
                return False
        duration = max(1.0, float(params["duration"]))
        x0, y0 = self.pos_x, self.pos_y
        self.spin_for(duration + 0.5)
        dist = math.sqrt((self.pos_x - x0) ** 2 + (self.pos_y - y0) ** 2)
        return self.arm_state == ARMING_ARMED and dist >= 1.0

    def verify_ack_only(self) -> bool:
        self.spin_for(1.0)
        return True

    def execute_command(self, command: Dict[str, Any]) -> bool:
        self.send(command)
        if not self.wait_published():
            return False

        action = command["action"]
        params = command.get("params", {})
        if action == "TAKEOFF":
            return self.verify_takeoff(params)
        if action == "LAND":
            return self.verify_land()
        if action == "HOVER":
            return self.verify_hover(params)
        if action == "MOVE_VELOCITY":
            return self.verify_move_velocity(params)
        if action == "MOVE_REL":
            return self.verify_move_rel(params)
        if action == "GOTO_NED":
            return self.verify_goto(params)
        if action == "YAW_TO":
            return self.verify_yaw(params)
        if action == "ORBIT":
            return self.verify_orbit(params)
        return self.verify_ack_only()


def load_tasks(path: Path) -> List[Dict[str, Any]]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_jsonl(path: Path, records: List[Dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        for row in records:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")


def read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def summarize(records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[tuple, List[Dict[str, Any]]] = {}
    for row in records:
        grouped.setdefault((row["model_name"], row["group"]), []).append(row)

    out = []
    for (model_name, group), rows in grouped.items():
        total = len(rows)
        success_rows = [r for r in rows if r["mission_success"]]
        mcr = len(success_rows) / total * 100.0 if total else 0.0
        asc = sum(int(r["steps"]) for r in success_rows) / len(success_rows) if success_rows else 0.0
        out.append(
            {
                "model_name": model_name,
                "group": group,
                "episode_count": total,
                "mcr_pct": round(mcr, 2),
                "asc": round(asc, 2),
            }
        )
    return out


def parse_group(raw_output: str, group: str) -> Optional[Dict[str, Any]]:
    parsed = direct_parse(raw_output) if group == "baseline" else robust_parse(raw_output)
    if not parsed.ok:
        return None
    if group == "baseline":
        return validate_baseline_command(parsed.parsed)
    return validate_proposed_command(parsed.parsed)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", default="tasks_online.json")
    parser.add_argument("--raw-cache", required=True)
    parser.add_argument("--result-jsonl", required=True)
    parser.add_argument("--result-csv", required=True)
    parser.add_argument("--models", nargs="+", required=True)
    parser.add_argument("--group", choices=["baseline", "proposed"], required=True)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()

    base_dir = Path(__file__).resolve().parent
    tasks = load_tasks(base_dir / args.tasks)
    raw_rows = read_jsonl(base_dir / args.raw_cache)
    if args.limit > 0:
        tasks = tasks[: args.limit]

    jsonl_path = base_dir / args.result_jsonl
    csv_path = base_dir / args.result_csv
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = OnlineEvalRunner()
    records: List[Dict[str, Any]] = []
    raw_map = {(row["model_name"], row["task_id"]): row for row in raw_rows}

    try:
        if not node.wait_for_position():
            raise RuntimeError("No local position stream. Start AirSim/Unreal and wait for PX4 connection.")

        for model_name in args.models:
            for task in tasks:
                resp = raw_map[(model_name, task["id"])]
                parsed = parse_group(resp["raw_output"], args.group)
                if parsed is None:
                    records.append(
                        {
                            "episode_id": task["id"],
                            "task_text": task["text"],
                            "model_name": model_name,
                            "group": args.group,
                            "latency_sec": resp["latency_sec"],
                            "raw_output": resp["raw_output"],
                            "command_json": None,
                            "mission_success": False,
                            "steps": 0,
                            "failure_reason": "INVALID_JSON",
                        }
                    )
                    write_jsonl(jsonl_path, records)
                    write_csv(csv_path, summarize(records))
                    print(f"[{args.group}] {model_name} {task['id']} INVALID_JSON")
                    continue

                commands = flatten_command(parsed)
                mission_success = True
                for command in commands:
                    if not node.execute_command(command):
                        mission_success = False
                        break
                    node.spin_for(1.0)

                records.append(
                    {
                        "episode_id": task["id"],
                        "task_text": task["text"],
                        "model_name": model_name,
                        "group": args.group,
                        "latency_sec": resp["latency_sec"],
                        "raw_output": resp["raw_output"],
                        "command_json": parsed,
                        "mission_success": mission_success,
                        "steps": len(commands) if mission_success else 0,
                        "failure_reason": "" if mission_success else "EXECUTION_FAILED",
                    }
                )
                write_jsonl(jsonl_path, records)
                write_csv(csv_path, summarize(records))
                print(
                    f"[{args.group}] {model_name} {task['id']} "
                    f"{'PASS' if mission_success else 'FAIL'} steps={len(commands) if mission_success else 0}"
                )

                # Leave enough time for the drone to settle between episodes.
                node.spin_for(2.0)
    finally:
        write_jsonl(jsonl_path, records)
        write_csv(csv_path, summarize(records))
        node.destroy_node()
        rclpy.shutdown()

    print(f"Wrote {len(records)} online records to {args.result_jsonl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
