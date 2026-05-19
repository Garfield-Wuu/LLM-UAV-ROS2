#!/usr/bin/env python3
import argparse
import datetime as dt
import glob
import io
import os
import re
import time
from collections import deque

PATTERNS = [
    ("用户输入指令", re.compile(r"^\s*(?:\$|#)\s+.+|^\s*(?:ros2|python3?|colcon|source|export|cd|./\S+)(?:\s+.+)?$", re.IGNORECASE)),
    ("LLM推理开始", re.compile(r"\bllm\b.*(?:start|begin|开始|请求|request|infer|推理)", re.IGNORECASE)),
    ("LLM推理完成", re.compile(r"\bllm\b.*(?:done|finish|完成|response|结果|输出)", re.IGNORECASE)),
    ("UAV动作", re.compile(r"\b(?:TAKEOFF|LAND|MOVE_TO|STAY|RETURN_HOME|ABORT|offboard|arm|disarm|rtl|hover|mission)\b", re.IGNORECASE)),
    ("ERROR/WARN", re.compile(r"\b(?:ERROR|WARN|FATAL|Traceback|Exception)\b", re.IGNORECASE)),
    ("节点退出/重启", re.compile(r"\b(?:respawn|restarting|restart|exited|exit code|has died|died|shutdown)\b", re.IGNORECASE)),
    ("topic hz异常", re.compile(r"(?:topic\s+hz|hz\s*[:=]).*(?:异常|abnormal|drop|low|warn|error)|(?:异常|abnormal).*(?:topic\s+hz|hz)", re.IGNORECASE)),
]

META_CMD_RE = re.compile(r"^last_command:\s*(.*)$")


def now():
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def write_log(fp, terminal_name, event_type, message):
    fp.write(f"- [{now()}] [{terminal_name}] [{event_type}] {message.strip()}\n")
    fp.flush()


def parse_meta_snapshot(path):
    terminal = os.path.basename(path)
    last_command = None
    try:
        with io.open(path, "r", encoding="utf-8", errors="replace") as f:
            for _ in range(12):
                line = f.readline()
                if not line:
                    break
                m = META_CMD_RE.match(line.strip())
                if m:
                    last_command = m.group(1).strip() or None
                    break
    except OSError:
        return terminal, None
    return terminal, last_command


def classify(line):
    text = line.strip()
    if not text:
        return None
    for event_type, pattern in PATTERNS:
        if pattern.search(text):
            return event_type
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--terminals_glob", required=True)
    parser.add_argument("--log_file", required=True)
    parser.add_argument("--poll", type=float, default=1.0)
    args = parser.parse_args()

    offsets = {}
    recent = deque(maxlen=500)

    with io.open(args.log_file, "a", encoding="utf-8") as log_fp:
        write_log(log_fp, "monitor", "系统", "监控进程已启动，开始增量采集终端关键事件")

        initial_files = sorted(glob.glob(args.terminals_glob))
        for p in initial_files:
            terminal, last_cmd = parse_meta_snapshot(p)
            if last_cmd:
                sig = (terminal, "用户输入指令", last_cmd)
                if sig not in recent:
                    write_log(log_fp, terminal, "用户输入指令", f"(metadata) {last_cmd}")
                    recent.append(sig)
            try:
                offsets[p] = os.path.getsize(p)
            except OSError:
                offsets[p] = 0

        while True:
            files = sorted(glob.glob(args.terminals_glob))
            for path in files:
                if path not in offsets:
                    offsets[path] = 0
                    write_log(log_fp, os.path.basename(path), "系统", "发现新终端文件，开始监控")

                try:
                    size = os.path.getsize(path)
                except OSError:
                    continue

                if size < offsets[path]:
                    offsets[path] = 0
                    write_log(log_fp, os.path.basename(path), "系统", "终端文件被截断，已重置读取位置")

                if size == offsets[path]:
                    continue

                try:
                    with io.open(path, "r", encoding="utf-8", errors="replace") as f:
                        f.seek(offsets[path])
                        chunk = f.read()
                except OSError:
                    continue

                offsets[path] = size
                terminal = os.path.basename(path)
                for raw in chunk.splitlines():
                    evt = classify(raw)
                    if not evt:
                        continue
                    msg = raw.strip()
                    sig = (terminal, evt, msg)
                    if sig in recent:
                        continue
                    write_log(log_fp, terminal, evt, msg)
                    recent.append(sig)

            time.sleep(args.poll)


if __name__ == "__main__":
    main()
