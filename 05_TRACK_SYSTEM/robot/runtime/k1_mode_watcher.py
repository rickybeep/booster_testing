#!/usr/bin/env python3
"""Standalone ROS2 process: mirrors /robot_states mode into protected runtime state.

Runs with the ROS2 environment sourced. Kept separate from the control server
so ROS2 DDS settings cannot degrade the SDK RPC channel.
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from booster_interface.msg import RobotStatesMsg

RUNTIME_DIR = os.environ.get("K1_TRACK_RUNTIME_DIR", "/run/booster-track-v10")
MODE_FILE = os.path.join(RUNTIME_DIR, "mode.json")
TMP = MODE_FILE + ".tmp"


def main():
    rclpy.init()
    node = Node("k1_mode_watcher")
    last_write = {"t": 0.0}

    def cb(msg):
        now = time.time()
        if now - last_write["t"] < 0.3:
            return
        last_write["t"] = now
        with open(TMP, "w") as f:
            json.dump({"mode": msg.current_mode, "ts": now}, f)
        os.replace(TMP, MODE_FILE)

    node.create_subscription(
        RobotStatesMsg,
        "/robot_states",
        cb,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        ),
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
