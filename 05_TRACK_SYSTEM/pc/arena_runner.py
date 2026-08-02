"""Small, partner-facing V10 waypoint runner.

The default invocation only validates configuration.  Live motion requires
three independent gates: ``--live``, an exact acknowledgement phrase, and
``live_motion_enabled: true`` in a local (ignored) configuration.  The runner
never changes robot mode; every enabled robot must already report WALK.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time
from typing import Callable, Iterable

import booster_config as cfg
from booster_nav import Navigator, bearing_deg, normalize_point, peer_hold_active
from tracker_config import (
    ConfigError,
    Keepout,
    RobotConfig,
    TrackerConfig,
    load_config,
    read_auth_token,
    validate_enabled_safety_relationships,
)


LIVE_ACK = "I_HAVE_EXPLICIT_LIVE_MOTION_APPROVAL"
MOVABLE_MODES = {"recover", "align", "turn", "walk", "escaping_bubble"}
PRIVATE_IPV4_NETWORKS = tuple(
    ipaddress.ip_network(value)
    for value in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16")
)
DOCUMENTATION_NETWORKS = tuple(
    ipaddress.ip_network(value)
    for value in ("192.0.2.0/24", "198.51.100.0/24", "203.0.113.0/24")
)


@dataclass(frozen=True)
class Observation:
    position: tuple[float, float] | None
    timestamp: float
    trusted: bool
    reason: str = ""


@dataclass(frozen=True)
class Intent:
    forward_mps: float
    turn_rad_s: float
    phase: str
    reason: str
    head_yaw_rad: float | None = None


def project_tag_position(
    raw: tuple[float, float],
    heading_deg: float,
    forward_m: float,
    left_m: float,
) -> tuple[float, float]:
    """Project a backpack/tag fix to the configured robot-center point."""
    theta = math.radians(float(heading_deg))
    return (
        raw[0] + forward_m * math.cos(theta) - left_m * math.sin(theta),
        raw[1] + forward_m * math.sin(theta) + left_m * math.cos(theta),
    )


class CommandSmoother:
    """Ramp increases; stop/decrease immediately and reverse through zero."""

    def __init__(self, forward_slew_per_s: float = 0.35, turn_slew_per_s: float = 2.0):
        self.forward_slew_per_s = float(forward_slew_per_s)
        self.turn_slew_per_s = float(turn_slew_per_s)
        self.forward = 0.0
        self.turn = 0.0
        self.last_time: float | None = None

    @staticmethod
    def _axis(current: float, target: float, step: float) -> float:
        if current * target < 0.0:
            return 0.0
        if target == 0.0 or abs(target) <= abs(current):
            return target
        return current + math.copysign(min(abs(target - current), step), target - current)

    def reset(self) -> None:
        self.forward = self.turn = 0.0
        self.last_time = None

    def step(self, forward: float, turn: float, now: float) -> tuple[float, float]:
        dt = 0.0 if self.last_time is None else max(0.0, min(0.25, now - self.last_time))
        self.last_time = now
        self.forward = self._axis(self.forward, forward, self.forward_slew_per_s * dt)
        self.turn = self._axis(self.turn, turn, self.turn_slew_per_s * dt)
        return self.forward, self.turn


class PatrolController:
    """Pure turn/walk/dwell state machine around V10 Navigator."""

    def __init__(
        self,
        robot: RobotConfig,
        tracker: TrackerConfig,
        obstacle_supplier: Callable[[], Iterable[tuple]],
    ):
        self.robot = robot
        self.tracker = tracker
        self._obstacle_supplier = obstacle_supplier
        self.navigator = Navigator(peer_supplier=lambda: tuple(self._obstacle_supplier()))
        self.smoother = CommandSmoother()
        self.phase = "WAIT"
        self.waypoint_index = 0
        self.dwell_until = 0.0
        self.next_head_at = 0.0
        self.head_index = 0
        self.peer_hold = False

    def _zero(self, phase: str, reason: str, head_yaw=None) -> Intent:
        self.smoother.reset()
        return Intent(0.0, 0.0, phase, reason, head_yaw)

    def start(self) -> None:
        self.navigator.set_target(self.robot.waypoints[0], source="manual")
        self.phase = "TURN"

    def _dwell(self, now: float) -> Intent:
        head = None
        if now >= self.next_head_at:
            head = self.tracker.head_scan_yaw_rad[self.head_index]
            self.head_index = (self.head_index + 1) % len(self.tracker.head_scan_yaw_rad)
            self.next_head_at = now + self.tracker.head_scan_interval_s
        if now < self.dwell_until:
            return self._zero("DWELL", "waypoint dwell", head)
        self.waypoint_index = (self.waypoint_index + 1) % len(self.robot.waypoints)
        self.navigator.set_target(self.robot.waypoints[self.waypoint_index], source="manual")
        self.phase = "TURN"
        return self._zero("TURN", "next waypoint selected", 0.0)

    def step(
        self,
        now: float,
        position: tuple[float, float] | None,
        heading_deg: float | None,
        *,
        obstacle_feed_healthy: bool,
    ) -> Intent:
        point = normalize_point(position)
        if point is None or heading_deg is None or not math.isfinite(float(heading_deg)):
            return self._zero("HOLD", "invalid or stale own pose")
        x0, y0, x1, y1 = self.tracker.arena
        margin = self.tracker.safe_margin_m
        if not (
            x0 + margin <= point[0] <= x1 - margin
            and y0 + margin <= point[1] <= y1 - margin
        ):
            return self._zero("HOLD", "outside safe arena boundary")
        if not obstacle_feed_healthy:
            return self._zero("HOLD", "obstacle feed unhealthy")
        obstacles = tuple(self._obstacle_supplier())
        self.peer_hold = peer_hold_active(point, obstacles, currently_holding=self.peer_hold)
        if self.peer_hold:
            return self._zero("HOLD", "peer/static hard-hold ring")
        if self.phase == "WAIT":
            self.start()
        if self.phase == "DWELL":
            return self._dwell(now)
        if self.phase == "TURN":
            self.navigator.plan_path(point)
            if self.navigator.target_blocked_reason:
                return self._zero("HOLD", self.navigator.target_blocked_reason)
            carrot = self.navigator.steer_point(point)
            desired = bearing_deg(point, carrot) if carrot is not None else None
            if desired is None:
                return self._zero("HOLD", "no safe steering carrot")
            turn, aligned = self.navigator.face(float(heading_deg), desired)
            if aligned:
                self.phase = "WALK"
                return self._zero("WALK", "heading aligned", 0.0)
            _forward, smooth_turn = self.smoother.step(0.0, turn, now)
            return Intent(0.0, smooth_turn, "TURN", "face steering carrot")
        fraction, turn, info = self.navigator.update(
            point, float(heading_deg), auto_repick=False
        )
        mode = str(info.get("mode", "unknown"))
        if mode == "arrived":
            self.phase = "DWELL"
            self.dwell_until = now + self.tracker.dwell_s
            self.next_head_at = now
            return self._zero("DWELL", "waypoint reached", 0.0)
        if mode not in MOVABLE_MODES:
            return self._zero("HOLD", f"navigator blocked: {mode}")
        forward = max(0.0, min(1.0, float(fraction))) * self.tracker.max_forward_mps
        turn = max(-self.tracker.max_turn_rad_s, min(self.tracker.max_turn_rad_s, float(turn)))
        forward, turn = self.smoother.step(forward, turn, now)
        return Intent(forward, turn, "WALK", mode)


class LiveRobot:
    def __init__(self, robot: RobotConfig, tracker: TrackerConfig):
        from booster_motion import MotionController

        self.config = robot
        auth_token = read_auth_token(tracker, robot)
        self.motion = MotionController(
            hosts=robot.panel_hosts,
            panel_port=robot.panel_port,
            robot_name=robot.name,
            nav_yaw_stale_s=robot.nav_yaw_stale_s,
            auth_token=auth_token,
        )
        self.motion.set_imu_calibration(
            sign=robot.imu_yaw_sign,
            offset_deg=robot.imu_yaw_offset_deg,
        )
        self.motion.set_caps(tracker.max_forward_mps, tracker.max_turn_rad_s)
        self.obstacles: tuple[tuple, ...] = ()
        self.patrol = PatrolController(robot, tracker, lambda: self.obstacles)
        self.observation = Observation(None, 0.0, False, "not sampled")

    def readiness_error(self) -> str:
        if not self.motion.connected:
            return "control server/nav feedback not connected"
        if not self.motion.mode_fresh:
            return "mode feedback is stale"
        if self.motion.actual_mode != "walk":
            return f"robot mode is {self.motion.actual_mode!r}, expected 'walk'"
        if self.motion.heading_deg() is None:
            return "heading feedback is missing/stale"
        if not (self.motion.session_healthy or self.motion.guard_acquirable):
            return "motion guard is neither session-owned nor acquirable"
        if not self.observation.trusted:
            return f"UWB position is not trusted: {self.observation.reason}"
        return ""


class LiveFleet:
    def __init__(self, tracker: TrackerConfig):
        from track_uwb import UwbTracker

        self.config = tracker
        if tracker.uwb_port.upper() == "AUTO":
            raise RuntimeError("live tracking requires an explicit UWB serial port")
        port = tracker.uwb_port
        self.uwb = UwbTracker(
            port,
            tracker.uwb_baud,
            list(tracker.anchors),
            dz=tracker.anchor_to_tag_height_m,
            minimum_anchors=tracker.minimum_anchors,
            maximum_residual_m=tracker.maximum_residual_m,
            allow_unverified_lo=tracker.allow_unverified_lo,
        )
        self.robots = [LiveRobot(robot, tracker) for robot in tracker.enabled_robots]
        self.stop_event = threading.Event()

    def _sample(self, robot: LiveRobot, wall_now: float | None = None) -> Observation:
        measurement = self.uwb.measurement(robot.config.uwb_tag)
        raw = measurement.position
        timestamp = measurement.timestamp
        if raw is None or timestamp <= 0.0:
            return Observation(None, timestamp, False, "no fix")
        if not math.isfinite(float(timestamp)):
            return Observation(None, timestamp, False, "non-finite fix timestamp")
        try:
            raw_point = (float(raw[0]), float(raw[1]))
        except (TypeError, ValueError, OverflowError, IndexError):
            return Observation(None, timestamp, False, "malformed fix")
        if not all(math.isfinite(value) for value in raw_point):
            return Observation(None, timestamp, False, "non-finite fix")
        observed_at = time.time() if wall_now is None else float(wall_now)
        age = observed_at - timestamp
        if age < 0.0:
            return Observation(None, timestamp, False, "future-dated fix")
        if age > self.config.position_stale_s:
            return Observation(None, timestamp, False, "stale fix")
        if measurement.source == "lo":
            if not self.config.allow_unverified_lo:
                return Observation(
                    None,
                    timestamp,
                    False,
                    "device-solved LO fix is not explicitly allowed",
                )
        elif measurement.source == "mc":
            if measurement.n_anchors < self.config.minimum_anchors:
                return Observation(
                    None,
                    timestamp,
                    False,
                    f"only {measurement.n_anchors} anchors",
                )
            if measurement.residual_m > self.config.maximum_residual_m:
                return Observation(
                    None,
                    timestamp,
                    False,
                    f"residual {measurement.residual_m:.3f} m",
                )
        else:
            return Observation(None, timestamp, False, "unknown fix provenance")
        heading = robot.motion.heading_deg()
        if heading is None:
            return Observation(None, timestamp, False, "heading unavailable for tag offset")
        point = project_tag_position(
            raw_point,
            heading,
            robot.config.tag_forward_m,
            robot.config.tag_left_m,
        )
        normalized = normalize_point(point)
        if normalized is None:
            return Observation(None, timestamp, False, "invalid projected fix")
        return Observation(normalized, timestamp, True)

    def _refresh_obstacles(self) -> bool:
        for robot in self.robots:
            robot.observation = self._sample(robot)
        all_trusted = all(robot.observation.trusted for robot in self.robots)
        for robot in self.robots:
            entries = []
            for keepout in self.config.keepouts:
                entries.append((
                    keepout.center[0], keepout.center[1],
                    robot.config.keepout_radius_m + keepout.radius_m,
                    {"type": "virtual", "label": keepout.label, "cooperative": False},
                ))
            for peer in self.robots:
                if peer is robot or not peer.observation.trusted:
                    continue
                px, py = peer.observation.position
                entries.append((
                    px, py,
                    robot.config.keepout_radius_m + peer.config.keepout_radius_m,
                    {"type": "robot", "label": peer.config.name, "cooperative": True},
                ))
            robot.obstacles = tuple(entries)
        return all_trusted

    def _stop_all(self) -> None:
        for robot in self.robots:
            try:
                robot.motion.stop()
            except Exception:
                pass

    def run(self) -> int:
        armed = []
        uwb_started = False
        try:
            self.uwb.start()
            uwb_started = True
            for robot in self.robots:
                robot.motion.connect()
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and not self.stop_event.is_set():
                self._refresh_obstacles()
                errors = [robot.readiness_error() for robot in self.robots]
                if not any(errors):
                    break
                time.sleep(0.10)
            if self.stop_event.is_set():
                return 0
            errors = {robot.config.name: robot.readiness_error() for robot in self.robots}
            errors = {name: error for name, error in errors.items() if error}
            if errors:
                raise RuntimeError(f"preflight failed: {errors}")
            for robot in self.robots:
                if not robot.motion.arm():
                    raise RuntimeError(f"{robot.config.name}: ARM failed: {robot.motion.last_error}")
                armed.append(robot)
            period = 1.0 / self.config.control_hz
            while not self.stop_event.is_set():
                started = time.monotonic()
                feed_healthy = self._refresh_obstacles()
                reports = []
                for robot in self.robots:
                    heading = robot.motion.heading_deg()
                    gate = robot.readiness_error()
                    if gate or not robot.motion.session_healthy:
                        intent = Intent(0.0, 0.0, "HOLD", gate or "guard session unhealthy")
                    else:
                        intent = robot.patrol.step(
                            started,
                            robot.observation.position,
                            heading,
                            obstacle_feed_healthy=feed_healthy,
                        )
                    robot.motion.command(intent.forward_mps, intent.turn_rad_s)
                    if intent.head_yaw_rad is not None:
                        robot.motion.head(0.0, intent.head_yaw_rad, speed=cfg.HEAD_SLEW_RAD_S)
                    reports.append({
                        "robot": robot.config.name,
                        "phase": intent.phase,
                        "reason": intent.reason,
                        "position": robot.observation.position,
                        "heading_deg": heading,
                        "forward_mps": round(intent.forward_mps, 4),
                        "turn_rad_s": round(intent.turn_rad_s, 4),
                    })
                print(json.dumps({"ts": time.time(), "robots": reports}, sort_keys=True), flush=True)
                self.stop_event.wait(max(0.0, period - (time.monotonic() - started)))
        finally:
            self.uwb.running = False
            self._stop_all()
            for robot in reversed(armed):
                try:
                    robot.motion.disarm()
                except Exception:
                    pass
            for robot in self.robots:
                try:
                    robot.motion.shutdown()
                except Exception:
                    pass
            if uwb_started and self.uwb.is_alive():
                self.uwb.join(timeout=1.5)
        return 0


def validate_live_gate(tracker: TrackerConfig, acknowledgement: str | None) -> None:
    if not tracker.live_motion_enabled:
        raise ConfigError("local configuration has live_motion_enabled=false")
    if not tracker.enabled_robots:
        raise ConfigError("no robots are enabled")
    if acknowledgement != LIVE_ACK:
        raise ConfigError(f"--acknowledge must equal {LIVE_ACK!r}")
    validate_enabled_safety_relationships(tracker)
    if tracker.uwb_port.upper() == "AUTO":
        raise ConfigError("live motion requires an explicit uwb.port; AUTO is not allowed")
    for robot in tracker.enabled_robots:
        for host in robot.panel_hosts:
            address = ipaddress.ip_address(host)
            if (
                address.version != 4
                or any(address in network for network in DOCUMENTATION_NETWORKS)
                or not any(address in network for network in PRIVATE_IPV4_NETWORKS)
                or address.is_loopback
                or address.is_link_local
                or address.is_multicast
                or address.is_unspecified
            ):
                raise ConfigError(
                    f"{robot.name} host {host} must be a site-local RFC1918 IPv4 address"
                )
        read_auth_token(tracker, robot)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="tracker.local.json path")
    parser.add_argument("--live", action="store_true", help="permit UWB/network/motion after all gates")
    parser.add_argument("--acknowledge", help="exact live-motion approval phrase")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        tracker = load_config(args.config)
        tracker.apply_navigation_defaults()
        if not args.live:
            print(json.dumps({
                "valid": True,
                "config": str(tracker.path),
                "enabled_robots": [robot.name for robot in tracker.enabled_robots],
                "live_motion": False,
                "note": "validation only; no serial, network, SDK, or robot command was used",
            }, indent=2))
            return 0
        validate_live_gate(tracker, args.acknowledge)
        fleet = LiveFleet(tracker)
        signal.signal(signal.SIGINT, lambda *_args: fleet.stop_event.set())
        signal.signal(signal.SIGTERM, lambda *_args: fleet.stop_event.set())
        return fleet.run()
    except (ConfigError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
