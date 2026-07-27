#!/usr/bin/env python3
"""Minimal tracked-arena patrol for the Booster K1.

The default mode is a fully offline simulation.  Live observation performs
GETs only.  Motor/head POSTs are possible only in ``execute`` mode with an
explicit confirmation phrase.

Coordinate convention used throughout this file:

* arena +X is heading 0 degrees;
* arena +Y is heading +90 degrees;
* positive logical yaw rate is counter-clockwise;
* ``sdk_yaw_sign`` is applied once, at the HTTP/SDK boundary.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Sequence


Point = tuple[float, float]
EXECUTE_CONFIRMATION = "I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT"
ROLE_RE = re.compile(r"([at])(\d+):(\d+)")


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_deg(value: float) -> float:
    """Wrap degrees to [-180, 180)."""
    return (value + 180.0) % 360.0 - 180.0


def point_distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _point(value: Any, name: str) -> Point:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"{name} must contain [x, y]")
    return (_finite(value[0], f"{name}[0]"), _finite(value[1], f"{name}[1]"))


@dataclass(frozen=True)
class ArenaConfig:
    arena: tuple[float, float, float, float]
    anchors: tuple[Point, ...]
    anchor_to_tag_vertical_offset_m: float
    tag_id: int
    tag_forward_offset_m: float
    perimeter_stop_m: float
    safe_margin_m: float
    waypoints: tuple[Point, ...]
    arrival_radius_m: float
    slow_radius_m: float
    max_forward_m_s: float
    max_turn_rad_s: float
    recovery_forward_m_s: float
    turn_kp: float
    forward_cone_deg: float
    control_hz: float
    uwb_stale_s: float
    nav_stale_s: float
    status_stale_s: float
    trusted_anchor_count: int
    max_uwb_residual_m: float
    calibration_distance_m: float
    calibration_speed_m_s: float
    calibration_timeout_s: float
    imu_yaw_sign: int
    sdk_yaw_sign: int

    @classmethod
    def load(cls, path: Path) -> "ArenaConfig":
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("configuration root must be a JSON object")
        arena_raw = data.get("arena_m")
        if not isinstance(arena_raw, list) or len(arena_raw) != 4:
            raise ValueError("arena_m must contain [x_min, y_min, x_max, y_max]")
        arena = tuple(_finite(v, f"arena_m[{i}]") for i, v in enumerate(arena_raw))
        anchors_raw = data.get("anchors_m")
        if not isinstance(anchors_raw, list):
            raise ValueError("anchors_m must be a list")
        anchors = tuple(_point(v, f"anchors_m[{i}]") for i, v in enumerate(anchors_raw))
        waypoints_raw = data.get("waypoints_m")
        if not isinstance(waypoints_raw, list):
            raise ValueError("waypoints_m must be a list")
        waypoints = tuple(
            _point(v, f"waypoints_m[{i}]") for i, v in enumerate(waypoints_raw)
        )

        tag_id = data.get("tag_id")
        trusted_anchor_count = data.get("trusted_anchor_count")
        imu_yaw_sign = data.get("imu_yaw_sign")
        sdk_yaw_sign = data.get("sdk_yaw_sign")
        for name, value in (
            ("tag_id", tag_id),
            ("trusted_anchor_count", trusted_anchor_count),
            ("imu_yaw_sign", imu_yaw_sign),
            ("sdk_yaw_sign", sdk_yaw_sign),
        ):
            if type(value) is not int:
                raise ValueError(f"{name} must be an integer")

        cfg = cls(
            arena=arena,  # type: ignore[arg-type]
            anchors=anchors,
            anchor_to_tag_vertical_offset_m=_finite(
                data.get("anchor_to_tag_vertical_offset_m"),
                "anchor_to_tag_vertical_offset_m",
            ),
            tag_id=tag_id,
            tag_forward_offset_m=_finite(
                data.get("tag_forward_offset_m"), "tag_forward_offset_m"
            ),
            perimeter_stop_m=_finite(
                data.get("perimeter_stop_m"), "perimeter_stop_m"
            ),
            safe_margin_m=_finite(data.get("safe_margin_m"), "safe_margin_m"),
            waypoints=waypoints,
            arrival_radius_m=_finite(
                data.get("arrival_radius_m"), "arrival_radius_m"
            ),
            slow_radius_m=_finite(data.get("slow_radius_m"), "slow_radius_m"),
            max_forward_m_s=_finite(
                data.get("max_forward_m_s"), "max_forward_m_s"
            ),
            max_turn_rad_s=_finite(
                data.get("max_turn_rad_s"), "max_turn_rad_s"
            ),
            recovery_forward_m_s=_finite(
                data.get("recovery_forward_m_s"), "recovery_forward_m_s"
            ),
            turn_kp=_finite(data.get("turn_kp"), "turn_kp"),
            forward_cone_deg=_finite(
                data.get("forward_cone_deg"), "forward_cone_deg"
            ),
            control_hz=_finite(data.get("control_hz"), "control_hz"),
            uwb_stale_s=_finite(data.get("uwb_stale_s"), "uwb_stale_s"),
            nav_stale_s=_finite(data.get("nav_stale_s"), "nav_stale_s"),
            status_stale_s=_finite(
                data.get("status_stale_s"), "status_stale_s"
            ),
            trusted_anchor_count=trusted_anchor_count,
            max_uwb_residual_m=_finite(
                data.get("max_uwb_residual_m"), "max_uwb_residual_m"
            ),
            calibration_distance_m=_finite(
                data.get("calibration_distance_m"), "calibration_distance_m"
            ),
            calibration_speed_m_s=_finite(
                data.get("calibration_speed_m_s"), "calibration_speed_m_s"
            ),
            calibration_timeout_s=_finite(
                data.get("calibration_timeout_s"), "calibration_timeout_s"
            ),
            imu_yaw_sign=imu_yaw_sign,
            sdk_yaw_sign=sdk_yaw_sign,
        )
        cfg.validate()
        return cfg

    def validate(self) -> None:
        x0, y0, x1, y1 = self.arena
        if not (x1 > x0 and y1 > y0):
            raise ValueError("arena dimensions must be positive")
        if len(self.anchors) < 3:
            raise ValueError("at least three anchors are required")
        if not self.waypoints:
            raise ValueError("at least one waypoint is required")
        if self.imu_yaw_sign not in (-1, 1):
            raise ValueError("imu_yaw_sign must be -1 or 1")
        if self.sdk_yaw_sign not in (-1, 1):
            raise ValueError("sdk_yaw_sign must be -1 or 1")
        if self.tag_id < 0:
            raise ValueError("tag_id cannot be negative")
        if not 3 <= self.trusted_anchor_count <= len(self.anchors):
            raise ValueError("trusted_anchor_count must be between 3 and anchor count")
        positive = {
            "safe_margin_m": self.safe_margin_m,
            "arrival_radius_m": self.arrival_radius_m,
            "slow_radius_m": self.slow_radius_m,
            "max_forward_m_s": self.max_forward_m_s,
            "max_turn_rad_s": self.max_turn_rad_s,
            "recovery_forward_m_s": self.recovery_forward_m_s,
            "turn_kp": self.turn_kp,
            "forward_cone_deg": self.forward_cone_deg,
            "control_hz": self.control_hz,
            "uwb_stale_s": self.uwb_stale_s,
            "nav_stale_s": self.nav_stale_s,
            "status_stale_s": self.status_stale_s,
            "calibration_distance_m": self.calibration_distance_m,
            "calibration_speed_m_s": self.calibration_speed_m_s,
            "calibration_timeout_s": self.calibration_timeout_s,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.perimeter_stop_m < 0.0:
            raise ValueError("perimeter_stop_m cannot be negative")
        if self.safe_margin_m <= self.perimeter_stop_m:
            raise ValueError("safe_margin_m must exceed perimeter_stop_m")
        if 2.0 * self.safe_margin_m >= min(x1 - x0, y1 - y0):
            raise ValueError("safe_margin_m leaves no navigable interior")
        if self.max_forward_m_s > 0.4 or self.max_turn_rad_s > 0.8:
            raise ValueError("configured speed exceeds the current onboard bridge limits")
        sx0, sy0, sx1, sy1 = self.safe_rect
        for index, (x, y) in enumerate(self.waypoints):
            if not (sx0 <= x <= sx1 and sy0 <= y <= sy1):
                raise ValueError(f"waypoint {index} is outside the safe interior")

    @property
    def safe_rect(self) -> tuple[float, float, float, float]:
        x0, y0, x1, y1 = self.arena
        m = self.safe_margin_m
        return (x0 + m, y0 + m, x1 - m, y1 - m)

    @property
    def center(self) -> Point:
        x0, y0, x1, y1 = self.arena
        return ((x0 + x1) / 2.0, (y0 + y1) / 2.0)


@dataclass(frozen=True)
class UwbFix:
    position: Point
    received_at: float
    anchors_used: int = 0
    residual_m: float = 0.0


@dataclass(frozen=True)
class NavSample:
    yaw_rad: float
    source_ts: float
    progressed_at: float
    rate_hz: float


@dataclass(frozen=True)
class PanelStatus:
    mode: str
    last_rc: Optional[int]
    last_move_ts: float
    last_move_error: str
    received_at: float


@dataclass(frozen=True)
class MotionCommand:
    forward_m_s: float = 0.0
    yaw_rad_s: float = 0.0

    @property
    def moving(self) -> bool:
        return abs(self.forward_m_s) > 1e-9 or abs(self.yaw_rad_s) > 1e-9


@dataclass(frozen=True)
class HeadCommand:
    pitch_rad: float
    yaw_rad: float
    duration_s: float = 0.8


@dataclass(frozen=True)
class ControlOutput:
    state: str
    reason: str
    motion: MotionCommand = MotionCommand()
    head: Optional[HeadCommand] = None
    position: Optional[Point] = None
    target: Optional[Point] = None
    heading_deg: Optional[float] = None


def parse_mc_ranges(parts: Sequence[str], anchor_count: int) -> list[tuple[int, float]]:
    ranges: list[tuple[int, float]] = []
    for index in range(anchor_count):
        field_index = 2 + index
        if field_index >= len(parts):
            break
        raw = parts[field_index]
        if len(raw) != 8 or raw.lower() == "ffffffff":
            continue
        try:
            millimetres = int(raw, 16)
        except ValueError:
            continue
        if millimetres > 0:
            ranges.append((index, millimetres / 1000.0))
    return ranges


def trilaterate_2d(
    anchors: Sequence[Point], ranges: Sequence[tuple[int, float]]
) -> Optional[Point]:
    if len(ranges) < 3:
        return None
    first_index, first_range = ranges[0]
    x0, y0 = anchors[first_index]
    sxx = sxy = syy = sxz = syz = 0.0
    rows = 0
    for index, measured_range in ranges[1:]:
        x, y = anchors[index]
        ax = 2.0 * (x - x0)
        ay = 2.0 * (y - y0)
        b = (
            x * x
            - x0 * x0
            + y * y
            - y0 * y0
            - (measured_range * measured_range - first_range * first_range)
        )
        sxx += ax * ax
        sxy += ax * ay
        syy += ay * ay
        sxz += ax * b
        syz += ay * b
        rows += 1
    determinant = sxx * syy - sxy * sxy
    if rows < 2 or abs(determinant) < 1e-9:
        return None
    return (
        (sxz * syy - sxy * syz) / determinant,
        (sxx * syz - sxy * sxz) / determinant,
    )


def range_residual(
    anchors: Sequence[Point],
    ranges: Sequence[tuple[int, float]],
    position: Point,
) -> float:
    if not ranges:
        return 0.0
    error_sq = 0.0
    for index, measured in ranges:
        anchor = anchors[index]
        predicted = point_distance(position, anchor)
        error_sq += (predicted - measured) ** 2
    return math.sqrt(error_sq / len(ranges))


def parse_uwb_line(line: str, cfg: ArenaConfig, received_at: float) -> Optional[UwbFix]:
    """Parse one Haorutech ULM3 line.

    ``LO=[x,y,z]`` is a device-solved position and therefore reports
    ``anchors_used=0``.  ``mc`` packets are solved locally and retain their
    anchor count and reprojection residual.
    """
    raw = line.strip()
    if not raw:
        return None
    role = ROLE_RE.search(raw)
    line_tag_id = int(role.group(2)) if role else cfg.tag_id
    if line_tag_id != cfg.tag_id:
        return None
    if "LO=[" in raw and "no solution" not in raw.lower():
        inside = raw.split("LO=[", 1)[1].split("]", 1)[0]
        try:
            fields = [float(token) for token in inside.split(",")]
            position = (fields[0], fields[1])
        except (ValueError, IndexError):
            return None
        if not all(math.isfinite(value) for value in position):
            return None
        return UwbFix(position=position, received_at=received_at)
    if raw.startswith("mc "):
        ranges = parse_mc_ranges(raw.split(), len(cfg.anchors))
        if cfg.anchor_to_tag_vertical_offset_m:
            dz = cfg.anchor_to_tag_vertical_offset_m
            ranges = [
                (index, math.sqrt(max(distance * distance - dz * dz, 0.0)))
                for index, distance in ranges
            ]
        position = trilaterate_2d(cfg.anchors, ranges)
        if position is None:
            return None
        return UwbFix(
            position=position,
            received_at=received_at,
            anchors_used=len(ranges),
            residual_m=range_residual(cfg.anchors, ranges, position),
        )
    return None


class SerialUwbReader(threading.Thread):
    """Small ULM3 reader with a median prefilter and jump rejection."""

    def __init__(
        self,
        port: str,
        baud: int,
        cfg: ArenaConfig,
        *,
        max_jump_m: float = 1.5,
    ):
        super().__init__(name="ulm3-uwb", daemon=True)
        self.port = port
        self.baud = baud
        self.cfg = cfg
        self.max_jump_m = max_jump_m
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._raw: deque[Point] = deque(maxlen=5)
        self._latest: Optional[UwbFix] = None
        self.last_error = ""

    def stop(self) -> None:
        self._stop_event.set()

    def latest(self) -> Optional[UwbFix]:
        with self._lock:
            return self._latest

    def run(self) -> None:
        try:
            import serial  # type: ignore
        except ImportError:
            self.last_error = "pyserial is required for live UWB input"
            return
        try:
            stream = serial.Serial(self.port, self.baud, timeout=0.5)
        except Exception as exc:
            self.last_error = f"cannot open {self.port}: {exc}"
            return
        try:
            while not self._stop_event.is_set():
                try:
                    line = stream.readline().decode("ascii", errors="replace")
                except Exception as exc:
                    self.last_error = f"UWB read failed: {exc}"
                    break
                fix = parse_uwb_line(line, self.cfg, time.monotonic())
                if fix is None:
                    continue
                self._raw.append(fix.position)
                filtered = (
                    statistics.median(point[0] for point in self._raw),
                    statistics.median(point[1] for point in self._raw),
                )
                with self._lock:
                    previous = self._latest
                    if (
                        previous is not None
                        and point_distance(previous.position, filtered) > self.max_jump_m
                    ):
                        continue
                    self._latest = UwbFix(
                        position=filtered,
                        received_at=fix.received_at,
                        anchors_used=fix.anchors_used,
                        residual_m=fix.residual_m,
                    )
        finally:
            try:
                stream.close()
            except Exception:
                pass


class HttpPanelBridge:
    """Client for the existing K1 onboard control-panel bridge.

    The bridge's ``POST /api/move`` response means the target was queued.  The
    runner separately watches ``/api/status.last_move_ts`` for progress from a
    successful robot-side ``B1LocoClient.Move`` call.
    """

    def __init__(
        self,
        base_url: str,
        cfg: ArenaConfig,
        *,
        commands_enabled: bool,
    ):
        try:
            import requests  # type: ignore
        except ImportError as exc:
            raise RuntimeError("requests is required for live panel access") from exc
        self._requests = requests
        self.base_url = base_url.rstrip("/")
        self.cfg = cfg
        self.commands_enabled = commands_enabled
        self._nav_session = requests.Session()
        self._status_session = requests.Session()
        self._move_session = requests.Session()
        self._head_session = requests.Session()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._nav: Optional[NavSample] = None
        self._status: Optional[PanelStatus] = None
        self._last_nav_source_ts: Optional[float] = None
        self.last_error = ""
        self.last_motion_payload: Optional[dict[str, float]] = None
        self.last_head_payload: Optional[dict[str, Any]] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._poll_loop, name="k1-panel-poll", daemon=True
        )
        self._thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        for session in (
            self._nav_session,
            self._status_session,
            self._move_session,
            self._head_session,
        ):
            session.close()

    def latest(self) -> tuple[Optional[NavSample], Optional[PanelStatus]]:
        with self._lock:
            return self._nav, self._status

    def _poll_loop(self) -> None:
        next_nav = next_status = time.monotonic()
        nav_period = 1.0 / self.cfg.control_hz
        status_period = 0.5
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= next_nav:
                self._poll_nav(now)
                next_nav = now + nav_period
            if now >= next_status:
                self._poll_status(now)
                next_status = now + status_period
            delay = max(0.005, min(next_nav, next_status) - time.monotonic())
            self._stop_event.wait(delay)

    def _poll_nav(self, now: float) -> None:
        try:
            response = self._nav_session.get(
                f"{self.base_url}/api/nav", timeout=0.35
            )
            response.raise_for_status()
            data = response.json()
            if not isinstance(data, dict):
                raise ValueError("nav response is not an object")
            yaw = _finite(data.get("yaw"), "nav.yaw")
            source_ts = _finite(data.get("ts"), "nav.ts")
            rate = _finite(data.get("rate_hz"), "nav.rate_hz")
            if source_ts <= 0.0:
                raise ValueError("nav.ts must advance from a positive value")
            with self._lock:
                source_changed = (
                    self._last_nav_source_ts is None
                    or source_ts > self._last_nav_source_ts
                    or source_ts < self._last_nav_source_ts
                )
                if source_changed:
                    self._nav = NavSample(yaw, source_ts, now, rate)
                    self._last_nav_source_ts = source_ts
                self.last_error = ""
        except Exception as exc:
            with self._lock:
                self.last_error = f"nav poll failed: {exc}"

    def _poll_status(self, now: float) -> None:
        try:
            response = self._status_session.get(
                f"{self.base_url}/api/status", timeout=0.35
            )
            response.raise_for_status()
            data = response.json()
            if not isinstance(data, dict):
                raise ValueError("status response is not an object")
            mode = data.get("mode")
            if mode not in ("damp", "prep", "walk", "custom", "unknown"):
                raise ValueError(f"invalid mode {mode!r}")
            raw_rc = data.get("last_rc")
            if raw_rc is not None and type(raw_rc) is not int:
                raise ValueError("last_rc must be an integer or null")
            move_error = data.get("last_move_error", "")
            if type(move_error) is not str:
                raise ValueError("last_move_error must be a string")
            status = PanelStatus(
                mode=mode,
                last_rc=raw_rc,
                last_move_ts=_finite(
                    data.get("last_move_ts", 0.0), "status.last_move_ts"
                ),
                last_move_error=move_error,
                received_at=now,
            )
            with self._lock:
                self._status = status
                self.last_error = ""
        except Exception as exc:
            with self._lock:
                self.last_error = f"status poll failed: {exc}"

    def motion_payload(self, command: MotionCommand) -> dict[str, float]:
        return {
            "vx": clamp(
                command.forward_m_s,
                -self.cfg.max_forward_m_s,
                self.cfg.max_forward_m_s,
            ),
            "vy": 0.0,
            "vyaw": clamp(
                self.cfg.sdk_yaw_sign * command.yaw_rad_s,
                -self.cfg.max_turn_rad_s,
                self.cfg.max_turn_rad_s,
            ),
        }

    def send_motion(self, command: MotionCommand) -> None:
        payload = self.motion_payload(command)
        self.last_motion_payload = payload
        if not self.commands_enabled:
            return
        response = self._move_session.post(
            f"{self.base_url}/api/move", json=payload, timeout=0.25
        )
        response.raise_for_status()
        data = response.json()
        if not isinstance(data, dict) or type(data.get("rc")) is not int:
            raise RuntimeError("move response did not contain an integer rc")
        if data["rc"] != 0:
            raise RuntimeError(f"move queue rejected command: rc={data['rc']}")

    def send_head(self, command: HeadCommand) -> None:
        payload: dict[str, Any] = {
            "mode": "ease",
            "pitch": clamp(command.pitch_rad, -0.30, 0.30),
            "yaw": clamp(command.yaw_rad, -1.10, 1.10),
            "duration": clamp(command.duration_s, 0.1, 10.0),
            "arc": 0.0,
            "arc_dir": 1,
        }
        self.last_head_payload = payload
        if not self.commands_enabled:
            return
        response = self._head_session.post(
            f"{self.base_url}/api/head", json=payload, timeout=0.25
        )
        response.raise_for_status()
        data = response.json()
        if not isinstance(data, dict) or data.get("rc") != 0:
            raise RuntimeError(f"head command rejected: {data!r}")

    def stop_burst(self, count: int = 3) -> None:
        if not self.commands_enabled:
            return
        for _ in range(max(1, count)):
            try:
                self.send_motion(MotionCommand())
            except Exception:
                pass
            time.sleep(0.05)


class ArenaNavigator:
    def __init__(self, cfg: ArenaConfig):
        self.cfg = cfg

    def command(
        self, position: Point, heading_deg: float, target: Point, *, recovery: bool
    ) -> MotionCommand:
        dx = target[0] - position[0]
        dy = target[1] - position[1]
        distance = math.hypot(dx, dy)
        desired_heading = math.degrees(math.atan2(dy, dx))
        error_deg = wrap_deg(desired_heading - heading_deg)
        turn = clamp(
            self.cfg.turn_kp * math.radians(error_deg),
            -self.cfg.max_turn_rad_s,
            self.cfg.max_turn_rad_s,
        )
        forward = 0.0
        if abs(error_deg) <= self.cfg.forward_cone_deg:
            speed_cap = (
                self.cfg.recovery_forward_m_s
                if recovery
                else self.cfg.max_forward_m_s
            )
            distance_scale = clamp(distance / self.cfg.slow_radius_m, 0.0, 1.0)
            alignment = max(0.35, math.cos(math.radians(error_deg)))
            forward = speed_cap * distance_scale * alignment
        return MotionCommand(forward, turn)


class ArenaPatrolController:
    DWELL_SEQUENCE: tuple[tuple[HeadCommand, float], ...] = (
        (HeadCommand(-0.03, -0.55, 0.8), 1.25),
        (HeadCommand(0.08, 0.55, 0.8), 1.25),
        (HeadCommand(0.03, 0.0, 0.7), 1.25),
        (HeadCommand(0.0, 0.0, 0.5), 0.75),
    )

    def __init__(
        self,
        cfg: ArenaConfig,
        *,
        heading_offset_deg: Optional[float],
        allow_calibration_motion: bool,
    ):
        self.cfg = cfg
        self.navigator = ArenaNavigator(cfg)
        self.heading_offset_deg = heading_offset_deg
        self.allow_calibration_motion = allow_calibration_motion
        self.state = "WAIT_INPUT"
        self.reason = "waiting for fresh UWB, nav, and status"
        self.fault = ""
        self._started_motion = False
        self._calibration_start: Optional[UwbFix] = None
        self._calibration_started_at: Optional[float] = None
        self._settle_until = 0.0
        self._waypoint_index = 0
        self._dwell_started_at = 0.0
        self._dwell_pose_sent = -1

    @property
    def current_waypoint(self) -> Point:
        return self.cfg.waypoints[self._waypoint_index]

    def trip_fault(self, reason: str) -> None:
        self.fault = reason
        self.state = "FAULT_STOP"
        self.reason = reason

    def _output(
        self,
        motion: MotionCommand = MotionCommand(),
        *,
        head: Optional[HeadCommand] = None,
        position: Optional[Point] = None,
        target: Optional[Point] = None,
        heading_deg: Optional[float] = None,
    ) -> ControlOutput:
        if motion.moving:
            self._started_motion = True
        return ControlOutput(
            state=self.state,
            reason=self.reason,
            motion=motion,
            head=head,
            position=position,
            target=target,
            heading_deg=heading_deg,
        )

    def _input_problem(
        self,
        now: float,
        fix: Optional[UwbFix],
        nav: Optional[NavSample],
        status: Optional[PanelStatus],
    ) -> Optional[str]:
        if fix is None:
            return "no UWB fix"
        if now - fix.received_at > self.cfg.uwb_stale_s:
            return "UWB fix is stale"
        if (
            fix.anchors_used > 0
            and (
                fix.anchors_used < self.cfg.trusted_anchor_count
                or fix.residual_m > self.cfg.max_uwb_residual_m
            )
        ):
            return (
                f"UWB fix untrusted ({fix.anchors_used} anchors, "
                f"{fix.residual_m:.3f} m residual)"
            )
        if nav is None:
            return "no IMU/nav sample"
        if now - nav.progressed_at > self.cfg.nav_stale_s:
            return "IMU/nav source timestamp is stale"
        if status is None:
            return "no robot status"
        if now - status.received_at > self.cfg.status_stale_s:
            return "robot status is stale"
        if status.mode != "walk":
            return f"robot must already be in WALK (reported {status.mode!r})"
        if status.last_move_error:
            return f"robot-side Move error: {status.last_move_error}"
        if status.last_rc not in (None, 0):
            return f"robot-side SDK rc={status.last_rc}"
        return None

    def _inside_hard_arena(self, position: Point) -> bool:
        x0, y0, x1, y1 = self.cfg.arena
        return x0 <= position[0] <= x1 and y0 <= position[1] <= y1

    def _edge_distance(self, position: Point) -> float:
        x0, y0, x1, y1 = self.cfg.arena
        return min(
            position[0] - x0,
            x1 - position[0],
            position[1] - y0,
            y1 - position[1],
        )

    def _inside_safe_interior(self, position: Point) -> bool:
        sx0, sy0, sx1, sy1 = self.cfg.safe_rect
        return sx0 <= position[0] <= sx1 and sy0 <= position[1] <= sy1

    def _heading(self, nav: NavSample) -> float:
        assert self.heading_offset_deg is not None
        return (
            self.cfg.imu_yaw_sign * math.degrees(nav.yaw_rad)
            + self.heading_offset_deg
        ) % 360.0

    def _robot_center(self, raw_tag_position: Point, heading_deg: float) -> Point:
        heading_rad = math.radians(heading_deg)
        offset = self.cfg.tag_forward_offset_m
        return (
            raw_tag_position[0] + offset * math.cos(heading_rad),
            raw_tag_position[1] + offset * math.sin(heading_rad),
        )

    def _calibration_step(
        self, now: float, fix: UwbFix, nav: NavSample
    ) -> ControlOutput:
        if self._calibration_start is None:
            if not self._inside_safe_interior(fix.position):
                self.state = "NEEDS_SAFE_CAL_START"
                self.reason = "place the tag inside the safe interior before calibration"
                return self._output(position=fix.position)
            self._calibration_start = fix
            self._calibration_started_at = now
            self.state = "CALIBRATE_FORWARD"
            self.reason = "walking straight to measure arena heading"
        assert self._calibration_started_at is not None
        if now - self._calibration_started_at > self.cfg.calibration_timeout_s:
            self.trip_fault("heading calibration timed out")
            return self._output(position=fix.position)
        if not self._inside_hard_arena(fix.position):
            self.trip_fault("UWB tag left the measured arena during calibration")
            return self._output(position=fix.position)
        if self._edge_distance(fix.position) <= self.cfg.perimeter_stop_m:
            self.trip_fault("calibration reached the perimeter stop band")
            return self._output(position=fix.position)
        travelled = point_distance(self._calibration_start.position, fix.position)
        if travelled >= self.cfg.calibration_distance_m:
            dx = fix.position[0] - self._calibration_start.position[0]
            dy = fix.position[1] - self._calibration_start.position[1]
            measured_bearing = math.degrees(math.atan2(dy, dx))
            self.heading_offset_deg = (
                measured_bearing
                - self.cfg.imu_yaw_sign * math.degrees(nav.yaw_rad)
            ) % 360.0
            self.state = "CALIBRATION_SETTLE"
            self.reason = f"heading offset measured: {self.heading_offset_deg:.2f} deg"
            self._settle_until = now + 1.0
            return self._output(
                head=HeadCommand(0.0, 0.0, 0.5),
                position=fix.position,
                heading_deg=self._heading(nav),
            )
        return self._output(
            MotionCommand(self.cfg.calibration_speed_m_s, 0.0),
            position=fix.position,
        )

    def _enter_dwell(
        self, now: float, position: Point, heading_deg: float
    ) -> ControlOutput:
        self.state = "DWELL_LOOK"
        self.reason = f"waypoint {self._waypoint_index + 1} reached; stopped and looking"
        self._dwell_started_at = now
        self._dwell_pose_sent = 0
        return self._output(
            head=self.DWELL_SEQUENCE[0][0],
            position=position,
            target=self.current_waypoint,
            heading_deg=heading_deg,
        )

    def _dwell_step(
        self, now: float, position: Point, heading_deg: float
    ) -> ControlOutput:
        elapsed = now - self._dwell_started_at
        cursor = 0.0
        pose_index: Optional[int] = None
        for index, (_, hold_s) in enumerate(self.DWELL_SEQUENCE):
            cursor += hold_s
            if elapsed < cursor:
                pose_index = index
                break
        if pose_index is None:
            self._waypoint_index = (self._waypoint_index + 1) % len(
                self.cfg.waypoints
            )
            self.state = "TRAVEL"
            self.reason = f"look complete; next waypoint {self._waypoint_index + 1}"
            self._dwell_pose_sent = -1
            return self._output(
                head=HeadCommand(0.0, 0.0, 0.5),
                position=position,
                target=self.current_waypoint,
                heading_deg=heading_deg,
            )
        head = None
        if pose_index != self._dwell_pose_sent:
            self._dwell_pose_sent = pose_index
            head = self.DWELL_SEQUENCE[pose_index][0]
        return self._output(
            head=head,
            position=position,
            target=self.current_waypoint,
            heading_deg=heading_deg,
        )

    def step(
        self,
        now: float,
        fix: Optional[UwbFix],
        nav: Optional[NavSample],
        status: Optional[PanelStatus],
    ) -> ControlOutput:
        if self.fault:
            return self._output()
        problem = self._input_problem(now, fix, nav, status)
        if problem is not None:
            if self._started_motion:
                self.trip_fault(problem)
            else:
                self.state = "WAIT_INPUT"
                self.reason = problem
            return self._output()
        assert fix is not None and nav is not None

        if self.heading_offset_deg is None:
            if not self.allow_calibration_motion:
                self.state = "NEEDS_HEADING"
                self.reason = (
                    "provide --heading-offset-deg or explicitly allow calibration"
                )
                return self._output(position=fix.position)
            return self._calibration_step(now, fix, nav)

        heading_deg = self._heading(nav)
        position = self._robot_center(fix.position, heading_deg)
        if self.state == "CALIBRATION_SETTLE":
            if now < self._settle_until:
                return self._output(position=position, heading_deg=heading_deg)
            self.state = "TRAVEL"
            self.reason = "calibration settled; beginning patrol"

        if not self._inside_hard_arena(position):
            self.trip_fault("projected robot center is outside the measured arena")
            return self._output(position=position, heading_deg=heading_deg)
        edge_distance = self._edge_distance(position)
        if edge_distance <= self.cfg.perimeter_stop_m:
            self.trip_fault("robot entered the perimeter stop band")
            return self._output(position=position, heading_deg=heading_deg)

        if self.state == "DWELL_LOOK":
            return self._dwell_step(now, position, heading_deg)

        if edge_distance < self.cfg.safe_margin_m:
            self.state = "RECOVER_CENTER"
            self.reason = "inside warning band; returning slowly toward arena center"
            command = self.navigator.command(
                position, heading_deg, self.cfg.center, recovery=True
            )
            return self._output(
                command,
                position=position,
                target=self.cfg.center,
                heading_deg=heading_deg,
            )

        if self.state == "RECOVER_CENTER":
            self.state = "TRAVEL"
            self.reason = "safe interior reacquired; resuming waypoint patrol"
        elif self.state not in ("TRAVEL",):
            self.state = "TRAVEL"
            self.reason = "beginning waypoint patrol"

        target = self.current_waypoint
        if point_distance(position, target) <= self.cfg.arrival_radius_m:
            return self._enter_dwell(now, position, heading_deg)
        command = self.navigator.command(
            position, heading_deg, target, recovery=False
        )
        self.reason = f"walking to waypoint {self._waypoint_index + 1}"
        return self._output(
            command,
            position=position,
            target=target,
            heading_deg=heading_deg,
        )


class MotionDeliveryGate:
    """Require robot-side successful-Move progress while a nonzero target is sent."""

    def __init__(self, timeout_s: float = 1.25):
        self.timeout_s = timeout_s
        self._moving_since: Optional[float] = None
        self._baseline_source_ts = 0.0
        self._last_source_ts = 0.0
        self._last_progress_at: Optional[float] = None
        self._confirmed = False

    def check(
        self, now: float, command: MotionCommand, status: Optional[PanelStatus]
    ) -> Optional[str]:
        if not command.moving:
            self._moving_since = None
            self._confirmed = False
            self._last_progress_at = None
            self._baseline_source_ts = (
                status.last_move_ts if status is not None else 0.0
            )
            self._last_source_ts = self._baseline_source_ts
            return None
        if self._moving_since is None:
            self._moving_since = now
            self._baseline_source_ts = (
                status.last_move_ts if status is not None else 0.0
            )
            self._last_source_ts = self._baseline_source_ts
        if status is not None and status.last_move_ts != self._last_source_ts:
            self._last_source_ts = status.last_move_ts
            if status.last_move_ts > self._baseline_source_ts:
                self._confirmed = True
                self._last_progress_at = now
        if not self._confirmed:
            if now - self._moving_since > self.timeout_s:
                return "no robot-side successful Move timestamp after nonzero command"
            return None
        assert self._last_progress_at is not None
        if now - self._last_progress_at > self.timeout_s:
            return "robot-side successful Move timestamp stopped advancing"
        return None


@dataclass
class SimWorld:
    position: Point = (2.20, 2.20)
    heading_deg: float = 15.0
    true_imu_offset_deg: float = 32.0

    def sensors(
        self, cfg: ArenaConfig, now: float
    ) -> tuple[UwbFix, NavSample, PanelStatus]:
        heading_rad = math.radians(self.heading_deg)
        tag = (
            self.position[0] - cfg.tag_forward_offset_m * math.cos(heading_rad),
            self.position[1] - cfg.tag_forward_offset_m * math.sin(heading_rad),
        )
        raw_yaw_deg = (
            self.heading_deg - self.true_imu_offset_deg
        ) / cfg.imu_yaw_sign
        return (
            UwbFix(tag, now, anchors_used=len(cfg.anchors), residual_m=0.01),
            NavSample(math.radians(raw_yaw_deg), now + 1.0, now, cfg.control_hz),
            PanelStatus("walk", 0, now + 1.0, "", now),
        )

    def advance(self, command: MotionCommand, dt: float) -> None:
        self.heading_deg = (
            self.heading_deg + math.degrees(command.yaw_rad_s * dt)
        ) % 360.0
        heading_rad = math.radians(self.heading_deg)
        self.position = (
            self.position[0] + command.forward_m_s * math.cos(heading_rad) * dt,
            self.position[1] + command.forward_m_s * math.sin(heading_rad) * dt,
        )


def run_simulation(cfg: ArenaConfig, seconds: float, *, quiet: bool = False) -> int:
    controller = ArenaPatrolController(
        cfg, heading_offset_deg=None, allow_calibration_motion=True
    )
    world = SimWorld()
    dt = 1.0 / cfg.control_hz
    now = 0.0
    last_state = ""
    arrivals = 0
    while now <= seconds:
        fix, nav, status = world.sensors(cfg, now)
        output = controller.step(now, fix, nav, status)
        if output.state != last_state:
            if output.state == "DWELL_LOOK":
                arrivals += 1
            if not quiet:
                print(
                    f"{now:6.2f}s  {output.state:20s}  {output.reason}"
                    f"  pos=({world.position[0]:.2f},{world.position[1]:.2f})"
                )
            last_state = output.state
        world.advance(output.motion, dt)
        now += dt
        if controller.fault:
            if not quiet:
                print(f"simulation fault: {controller.fault}")
            return 1
    if not quiet:
        offset = controller.heading_offset_deg
        print(
            f"simulation complete: {arrivals} waypoint arrivals, "
            f"heading offset={offset:.2f} deg, "
            f"final=({world.position[0]:.2f},{world.position[1]:.2f})"
        )
    return 0 if arrivals >= 2 else 2


def _live_log(output: ControlOutput, previous: str) -> str:
    if output.state != previous:
        position = (
            "unknown"
            if output.position is None
            else f"({output.position[0]:.2f},{output.position[1]:.2f})"
        )
        print(f"{output.state}: {output.reason}; position={position}")
    return output.state


def run_live(
    cfg: ArenaConfig,
    *,
    mode: str,
    robot_url: str,
    uwb_port: str,
    uwb_baud: int,
    heading_offset_deg: Optional[float],
    calibrate_heading: bool,
    seconds: float,
) -> int:
    commands_enabled = mode == "execute"
    panel = HttpPanelBridge(
        robot_url, cfg, commands_enabled=commands_enabled
    )
    uwb = SerialUwbReader(uwb_port, uwb_baud, cfg)
    controller = ArenaPatrolController(
        cfg,
        heading_offset_deg=heading_offset_deg,
        allow_calibration_motion=calibrate_heading,
    )
    delivery = MotionDeliveryGate()
    panel.start()
    uwb.start()
    started_at = time.monotonic()
    deadline = started_at + seconds if seconds > 0.0 else None
    next_tick = started_at
    prior_state = ""
    print(
        "LIVE EXECUTION ENABLED" if commands_enabled else "OBSERVE ONLY: no POSTs"
    )
    try:
        while deadline is None or time.monotonic() < deadline:
            now = time.monotonic()
            fix = uwb.latest()
            nav, status = panel.latest()
            output = controller.step(now, fix, nav, status)
            if commands_enabled:
                delivery_problem = delivery.check(now, output.motion, status)
                if delivery_problem:
                    controller.trip_fault(delivery_problem)
                    output = controller.step(now, fix, nav, status)
                try:
                    panel.send_motion(output.motion)
                    if output.head is not None:
                        panel.send_head(output.head)
                except Exception as exc:
                    controller.trip_fault(f"panel command failed: {exc}")
                    output = controller.step(now, fix, nav, status)
            prior_state = _live_log(output, prior_state)
            if controller.fault:
                return 1
            next_tick += 1.0 / cfg.control_hz
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("operator interrupted patrol")
    finally:
        if commands_enabled:
            panel.stop_burst()
        uwb.stop()
        uwb.join(timeout=1.0)
        panel.close()
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    default_config = Path(__file__).with_name("arena_config.json")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("simulate", "observe", "execute"),
        default="simulate",
        help="simulate is offline; observe performs GET/serial only; execute can POST",
    )
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument("--robot-url", help="for example http://ROBOT_IP:8080")
    parser.add_argument("--uwb-port", help="ULM3 serial port, for example COM_PORT")
    parser.add_argument("--uwb-baud", type=int, default=115200)
    parser.add_argument("--heading-offset-deg", type=float)
    parser.add_argument(
        "--calibrate-heading",
        action="store_true",
        help="allow the controller to walk straight 0.8 m to measure heading",
    )
    parser.add_argument(
        "--confirm-live-motion",
        help=f"execute mode requires exactly: {EXECUTE_CONFIRMATION}",
    )
    parser.add_argument(
        "--sdk-yaw-sign",
        type=int,
        choices=(-1, 1),
        help="override the config only after a lifted sign check",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    cfg = ArenaConfig.load(args.config)
    if args.sdk_yaw_sign is not None:
        cfg = ArenaConfig(**{**cfg.__dict__, "sdk_yaw_sign": args.sdk_yaw_sign})
        cfg.validate()
    if args.seconds < 0.0:
        raise SystemExit("--seconds cannot be negative")
    if args.mode == "simulate":
        return run_simulation(cfg, args.seconds)
    if not args.robot_url or not args.uwb_port:
        raise SystemExit("observe/execute require --robot-url and --uwb-port")
    if args.heading_offset_deg is not None and args.calibrate_heading:
        raise SystemExit(
            "choose either --heading-offset-deg or --calibrate-heading, not both"
        )
    if args.mode == "execute":
        if args.confirm_live_motion != EXECUTE_CONFIRMATION:
            raise SystemExit(
                "execute refused: pass the exact --confirm-live-motion phrase "
                "shown in --help"
            )
        if args.heading_offset_deg is None and not args.calibrate_heading:
            raise SystemExit(
                "execute refused: provide --heading-offset-deg or "
                "--calibrate-heading"
            )
    return run_live(
        cfg,
        mode=args.mode,
        robot_url=args.robot_url,
        uwb_port=args.uwb_port,
        uwb_baud=args.uwb_baud,
        heading_offset_deg=args.heading_offset_deg,
        calibrate_heading=args.calibrate_heading,
        seconds=args.seconds,
    )


if __name__ == "__main__":
    raise SystemExit(main())
