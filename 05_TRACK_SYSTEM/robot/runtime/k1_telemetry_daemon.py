#!/usr/bin/env python3
"""Standalone telemetry daemon: SDK subscribers -> protected runtime snapshots.

Must run in its own process: the preinstalled SDK build starves its RPC
channel when subscribers live in the same process as a B1LocoClient.
"""
import json
import os
import time

import booster_robotics_sdk_python as sdk

try:
    from .k1_guard_protocol import require_sdk_module_path, require_sdk_success
except ImportError:
    from k1_guard_protocol import (  # type: ignore
        require_sdk_module_path,
        require_sdk_success,
    )

RUNTIME_DIR = os.environ.get("K1_TRACK_RUNTIME_DIR", "/run/booster-track-v10")
OUT = os.path.join(RUNTIME_DIR, "telemetry.json")
TMP = OUT + ".tmp"
# Compact nav state (yaw/gyro/odom) written at a much higher rate than the
# full telemetry snapshot -- consumed by BOOSTER_TRACK via GET /api/nav.
NAV_OUT = os.path.join(RUNTIME_DIR, "nav.json")
NAV_TMP = NAV_OUT + ".tmp"
NAV_PERIOD_S = 0.05  # 20 Hz

JOINT_NAMES = [
    "HeadYaw", "HeadPitch",
    "L.ShoulderPitch", "L.ShoulderRoll", "L.ElbowPitch", "L.ElbowYaw",
    "R.ShoulderPitch", "R.ShoulderRoll", "R.ElbowPitch", "R.ElbowYaw",
    "L.HipPitch", "L.HipRoll", "L.HipYaw", "L.KneePitch", "L.AnklePitch", "L.AnkleRoll",
    "R.HipPitch", "R.HipRoll", "R.HipYaw", "R.KneePitch", "R.AnklePitch", "R.AnkleRoll",
]

snap = {
    "ts": 0.0, "rate_hz": 0.0,
    "imu": {"rpy": [0, 0, 0], "gyro": [0, 0, 0], "acc": [0, 0, 0]},
    "joints": [],
    "odom": {"x": 0.0, "y": 0.0, "theta": 0.0},
}
rate = {"n": 0, "t0": time.time()}
last_write = {"t": 0.0}
last_nav_write = {"t": 0.0}


def write_snapshot():
    with open(TMP, "w") as f:
        json.dump(snap, f)
    os.replace(TMP, OUT)


def write_nav(now, imu):
    """High-rate compact nav state for the arena tracker (tiny tmpfs write)."""
    if now - last_nav_write["t"] < NAV_PERIOD_S:
        return
    last_nav_write["t"] = now
    nav = {
        "ts": round(now, 3),
        "yaw": round(float(imu.rpy[2]), 5),
        "gyro_z": round(float(imu.gyro[2]), 4),
        "odom": snap["odom"],
        "rate_hz": round(snap["rate_hz"], 1),
    }
    with open(NAV_TMP, "w") as f:
        json.dump(nav, f)
    os.replace(NAV_TMP, NAV_OUT)


def on_low_state(msg):
    rate["n"] += 1
    now = time.time()
    if now - rate["t0"] >= 1.0:
        snap["rate_hz"] = rate["n"] / (now - rate["t0"])
        rate["n"], rate["t0"] = 0, now
    write_nav(now, msg.imu_state)
    if now - last_write["t"] < 0.2:
        return
    last_write["t"] = now
    imu = msg.imu_state
    snap["ts"] = now
    snap["imu"] = {
        "rpy": [round(v, 4) for v in imu.rpy],
        "gyro": [round(v, 4) for v in imu.gyro],
        "acc": [round(v, 4) for v in imu.acc],
    }
    snap["joints"] = [
        {
            "name": JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"j{i}",
            "q": round(m.q, 3), "dq": round(m.dq, 3),
            "tau": round(m.tau_est, 2), "temp": round(m.temperature, 1),
            "lost": bool(m.lost),
        }
        for i, m in enumerate(msg.motor_state_parallel)
    ]
    write_snapshot()


def on_odometer(msg):
    snap["odom"] = {"x": round(msg.x, 3), "y": round(msg.y, 3), "theta": round(msg.theta, 3)}


def main():
    require_sdk_module_path(sdk)
    require_sdk_success(
        sdk.ChannelFactory.Instance().Init(0, "127.0.0.1"),
        "ChannelFactory.Init",
    )
    low = sdk.B1LowStateSubscriber(on_low_state)
    require_sdk_success(low.InitChannel(), "B1LowStateSubscriber.InitChannel")
    odom = sdk.B1OdometerStateSubscriber(on_odometer)
    require_sdk_success(
        odom.InitChannel(),
        "B1OdometerStateSubscriber.InitChannel",
    )
    print("telemetry daemon up", flush=True)
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
