"""
UWB position tracker for TRACK_CONTROL
======================================
Self-contained reader for the Haorutech ULM3 (HR-RTLS1) serial stream. Lifted
from the proven uwb_viewer pipeline so TRACK_CONTROL has no dependency on the
viewer script:

  * reads a TAG port (uses its own LO=[x,y,z] when present), or
  * solves (x, y) from an ANCHOR's mc ranging packet via 2D trilateration.

A short median pre-filter + a constant-velocity Kalman filter (with jump
rejection) keep each tag's position steady even with floor-level anchors.
"""

from __future__ import annotations

import math
import re
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

try:
    import serial  # pyserial
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial not installed. Run: python -m pip install pyserial") from exc

try:
    import numpy as np
    _HAVE_NUMPY = True
except ImportError:
    _HAVE_NUMPY = False

ROLE_RE = re.compile(r"([at])(\d+):(\d+)")


@dataclass(frozen=True)
class UwbMeasurement:
    """One accepted fix and its quality metadata from the same lock epoch."""

    position: Optional[Tuple[float, float]]
    trail: Tuple[Tuple[float, float], ...]
    timestamp: float
    n_anchors: int
    residual_m: float
    source: str


def parse_position_line(
    raw: str,
    anchors: List[Tuple[float, float]],
    *,
    dz: float = 0.0,
):
    """Return ``(tag, position, anchors, residual, source)`` or ``None``.

    A role marker is mandatory.  Treating a malformed/unidentified line as tag
    zero can silently attach one robot's position to another robot, so missing
    role metadata is rejected rather than assigned a fallback identity.
    """
    if not isinstance(raw, str) or not raw:
        return None
    role = ROLE_RE.search(raw)
    if role is None:
        return None
    tag_id = int(role.group(2))

    if "LO=[" in raw and "no solution" not in raw.lower():
        if "]" not in raw.split("LO=[", 1)[1]:
            return None
        inside = raw.split("LO=[", 1)[1].split("]", 1)[0]
        try:
            values = [float(token) for token in inside.split(",")]
            position = (values[0], values[1])
        except (ValueError, OverflowError, IndexError):
            return None
        if not all(math.isfinite(value) for value in position):
            return None
        return tag_id, position, 0, 0.0, "lo"

    if not raw.startswith("mc "):
        return None
    parts = raw.split()
    if len(parts) < 2 + len(anchors):
        return None
    ranges = parse_mc_ranges(parts, len(anchors))
    if dz:
        ranges = [
            (index, math.sqrt(max(distance * distance - dz * dz, 0.0)))
            for index, distance in ranges
        ]
    if len(ranges) < 3:
        return None
    position = trilaterate_2d(anchors, ranges)
    if position is None or not all(math.isfinite(value) for value in position):
        return None
    residual = reprojection_residual(anchors, ranges, position)
    if not math.isfinite(residual):
        return None
    return tag_id, position, len(ranges), residual, "mc"


def autodetect_port() -> Optional[str]:
    """Return the first CH340 / USB-serial COM port, or None."""
    try:
        from serial.tools import list_ports
    except Exception:
        return None
    for p in list_ports.comports():
        desc = (p.description or "")
        if "CH340" in desc or "USB-SERIAL" in desc:
            return p.device
    return None


def parse_mc_ranges(parts: List[str], n_anchors: int) -> List[Tuple[int, float]]:
    out: List[Tuple[int, float]] = []
    for i in range(n_anchors):
        if 2 + i >= len(parts):
            break
        hexval = parts[2 + i]
        if len(hexval) != 8 or hexval.lower() == "ffffffff":
            continue
        try:
            mm = int(hexval, 16)
        except ValueError:
            continue
        if mm > 0:
            out.append((i, mm / 1000.0))
    return out


def trilaterate_2d(anchors: List[Tuple[float, float]],
                   ranges: List[Tuple[int, float]]) -> Optional[Tuple[float, float]]:
    """Least-squares 2D position from (anchor_index, distance) pairs."""
    if len(ranges) < 3:
        return None
    i0, r0 = ranges[0]
    x0, y0 = anchors[i0]
    Sxx = Sxy = Syy = Sxz = Syz = 0.0
    rows = 0
    for i, ri in ranges[1:]:
        xi, yi = anchors[i]
        ax = 2 * (xi - x0)
        ay = 2 * (yi - y0)
        b = (xi * xi - x0 * x0) + (yi * yi - y0 * y0) - (ri * ri - r0 * r0)
        Sxx += ax * ax
        Sxy += ax * ay
        Syy += ay * ay
        Sxz += ax * b
        Syz += ay * b
        rows += 1
    if rows < 2:
        return None
    det = Sxx * Syy - Sxy * Sxy
    if abs(det) < 1e-9:
        return None
    x = (Sxz * Syy - Sxy * Syz) / det
    y = (Sxx * Syz - Sxy * Sxz) / det
    return (x, y)


def reprojection_residual(anchors: List[Tuple[float, float]],
                          ranges: List[Tuple[int, float]],
                          pos: Tuple[float, float]) -> float:
    """RMS of |measured_range - distance(pos, anchor)| over the contributing
    anchors. With 4+ anchors this is meaningful redundancy: a high residual
    means the ranges disagree (a dead/multipath anchor) even though a position
    was still computed -- so the caller can distrust the fix."""
    if not ranges:
        return 0.0
    x, y = pos
    acc = 0.0
    for i, ri in ranges:
        ax, ay = anchors[i]
        d = math.hypot(x - ax, y - ay)
        acc += (d - ri) ** 2
    return math.sqrt(acc / len(ranges))


class Kalman2D:
    """Constant-velocity 2D Kalman filter. State = [x, y, vx, vy]."""

    def __init__(self, meas_noise: float = 0.15, accel_noise: float = 2.0):
        self.r = meas_noise
        self.qa = accel_noise
        self.x = None
        self.P = None

    def init(self, mx: float, my: float) -> None:
        self.x = np.array([mx, my, 0.0, 0.0], dtype=float)
        self.P = np.diag([self.r ** 2, self.r ** 2, 1.0, 1.0])

    def predict_pos(self, dt: float):
        if self.x is None:
            return None
        return (self.x[0] + self.x[2] * dt, self.x[1] + self.x[3] * dt)

    def step(self, mx: float, my: float, dt: float):
        if self.x is None:
            self.init(mx, my)
            return (self.x[0], self.x[1])
        dt = max(dt, 1e-3)
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        q = self.qa ** 2
        Q = q * np.array([[dt**4/4, 0, dt**3/2, 0],
                          [0, dt**4/4, 0, dt**3/2],
                          [dt**3/2, 0, dt**2, 0],
                          [0, dt**3/2, 0, dt**2]], dtype=float)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        R = (self.r ** 2) * np.eye(2)
        z = np.array([mx, my], dtype=float)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        return (self.x[0], self.x[1])


class TagState:
    def __init__(self):
        self.raw = deque(maxlen=5)
        self.pos: Optional[Tuple[float, float]] = None
        self.trail = deque(maxlen=80)
        self.last = 0.0
        self.kf: Optional[Kalman2D] = None
        self.rejects = 0
        self.n_anchors = 0      # anchors behind the latest accepted fix (0 = device-solved LO)
        self.residual = 0.0     # reprojection RMS of the latest accepted fix (m)
        self.source = "unknown"


class UwbTracker(threading.Thread):
    """Background serial reader that maintains a filtered (x, y) per tag."""

    def __init__(self, port: str, baud: int, anchors: List[Tuple[float, float]],
                 dz: float = 0.0, meas_noise: float = 0.15, accel_noise: float = 2.0,
                 max_jump: float = 1.5, use_kf: bool = True,
                 minimum_anchors: int | None = None,
                 maximum_residual_m: float | None = None,
                 allow_unverified_lo: bool = True):
        super().__init__(daemon=True)
        self.port, self.baud, self.anchors = port, baud, anchors
        self.dz = dz
        self.meas_noise = meas_noise
        self.accel_noise = accel_noise
        self.max_jump = max_jump
        self.use_kf = use_kf and _HAVE_NUMPY
        self.minimum_anchors = (
            None if minimum_anchors is None else int(minimum_anchors)
        )
        self.maximum_residual_m = (
            None
            if maximum_residual_m is None
            else float(maximum_residual_m)
        )
        self.allow_unverified_lo = bool(allow_unverified_lo)
        self.lock = threading.Lock()
        self.tags: Dict[int, TagState] = {}
        self.running = True
        self.status = "starting..."
        self._stamps = deque(maxlen=120)
        self.quality_rejects = 0

    def rate(self) -> int:
        now = time.time()
        while self._stamps and now - self._stamps[0] > 1.0:
            self._stamps.popleft()
        return len(self._stamps)

    def _update(self, tag_id: int, pos: Tuple[float, float],
                n_anchors: int = 0, residual: float = 0.0,
                source: str = "unknown") -> None:
        try:
            px, py = float(pos[0]), float(pos[1])
            n_anchors = int(n_anchors)
            residual = float(residual)
        except (TypeError, ValueError, OverflowError, IndexError):
            return
        if not all(math.isfinite(value) for value in (px, py, residual)):
            return
        if n_anchors < 0 or residual < 0:
            self.quality_rejects += 1
            return
        if not self._quality_allowed(source, n_anchors, residual):
            self.quality_rejects += 1
            return
        pos = (px, py)
        st = self.tags.get(tag_id)
        if st is None:
            st = TagState()
            if self.use_kf:
                st.kf = Kalman2D(self.meas_noise, self.accel_noise)
            self.tags[tag_id] = st
        now = time.time()
        st.raw.append(pos)
        mx = statistics.median(p[0] for p in st.raw)
        my = statistics.median(p[1] for p in st.raw)

        if st.kf is not None:
            dt = (now - st.last) if st.last else 0.05
            pred = st.kf.predict_pos(dt)
            if pred is not None and st.rejects < 8:
                if math.hypot(mx - pred[0], my - pred[1]) > self.max_jump:
                    # A rejected outlier is NOT a good fix: do not advance st.last
                    # (consumers use it as the freshness timestamp) or the rate
                    # stamps. A sustained reject burst then correctly reads as
                    # stale rather than masking a frozen position, and the speed
                    # estimate isn't inflated by rejected packets.
                    st.rejects += 1
                    return
            st.rejects = 0
            sm = st.kf.step(mx, my, dt)
        elif st.pos is None:
            sm = (mx, my)
        else:
            a = 0.4
            sm = (st.pos[0] * (1 - a) + mx * a, st.pos[1] * (1 - a) + my * a)

        st.pos = sm
        st.trail.append(sm)
        st.last = now
        st.n_anchors = n_anchors
        st.residual = float(residual)
        st.source = source if source in ("lo", "mc") else "unknown"
        self._stamps.append(now)

    def _quality_allowed(self, source: str, n_anchors: int, residual: float) -> bool:
        if source == "lo":
            return self.allow_unverified_lo
        if source == "mc":
            return bool(
                (
                    self.minimum_anchors is None
                    or n_anchors >= self.minimum_anchors
                )
                and (
                    self.maximum_residual_m is None
                    or residual <= self.maximum_residual_m
                )
            )
        # Preserve the legacy direct-update helper only for callers that did
        # not configure a live quality policy. Serial parsing always supplies
        # explicit provenance.
        return bool(
            self.minimum_anchors is None
            and self.maximum_residual_m is None
        )

    def run(self) -> None:
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
        except serial.SerialException as e:
            self.status = f"CANNOT OPEN {self.port}: {e}"
            return
        self.status = f"reading {self.port} @ {self.baud}"
        read_errors = 0
        while self.running:
            try:
                raw = ser.readline().decode("ascii", errors="replace").strip()
            except Exception as e:
                # A persistent read error means the USB-serial link is gone. Don't
                # tight-spin on `continue`; after a short streak, set a CANNOT-OPEN
                # status and exit so the backend's _ensure_uwb_tracker respawns
                # (and re-autodetects the port) instead of looping on a dead port.
                read_errors += 1
                if read_errors > 20:
                    self.status = f"CANNOT OPEN {self.port}: read error ({e})"
                    break
                time.sleep(0.05)
                continue
            read_errors = 0
            if not raw:
                continue
            parsed = parse_position_line(raw, self.anchors, dz=self.dz)
            if parsed is not None:
                tag_id, pos, n_anchors_used, residual_used, source = parsed
                with self.lock:
                    self._update(
                        tag_id,
                        pos,
                        n_anchors_used,
                        residual_used,
                        source,
                    )
        try:
            ser.close()
        except Exception:
            pass

    def get(self, tag_id: int):
        """Return (pos, trail_list, last_ts) for one tag, or (None, [], 0)."""
        measurement = self.measurement(tag_id)
        return measurement.position, list(measurement.trail), measurement.timestamp

    def quality(self, tag_id: int) -> Tuple[int, float]:
        """Return (n_anchors, residual_m) of the latest accepted fix for a tag.
        n_anchors == 0 means a device-solved LO position (anchor health is not
        observable from the wire). Used to reject degraded trilateration fixes."""
        measurement = self.measurement(tag_id)
        return measurement.n_anchors, measurement.residual_m

    def measurement(self, tag_id: int) -> UwbMeasurement:
        """Return position, time, provenance, and quality atomically."""
        with self.lock:
            st = self.tags.get(tag_id)
            if st is None:
                return UwbMeasurement(None, (), 0.0, 0, 0.0, "none")
            position = (
                None
                if st.pos is None
                else (float(st.pos[0]), float(st.pos[1]))
            )
            trail = tuple((float(point[0]), float(point[1])) for point in st.trail)
            return UwbMeasurement(
                position=position,
                trail=trail,
                timestamp=float(st.last),
                n_anchors=int(st.n_anchors),
                residual_m=float(st.residual),
                source=str(st.source),
            )

    def snapshot(self):
        with self.lock:
            return ({tid: (st.pos, list(st.trail), st.last)
                     for tid, st in self.tags.items()}, self.status, self.rate())
