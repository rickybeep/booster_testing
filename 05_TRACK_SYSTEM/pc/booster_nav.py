"""
Navigator for BOOSTER_TRACK
===========================
Ported from REGULAR_HUMAN_LOVE (V10 lineage). Navigation is transport-agnostic:
the same UWB arena frame, layered safety zones, recovery behavior, anti-orbit
alignment, and face-angle helper work unchanged for the Booster K1.
"""

from __future__ import annotations

from contextlib import nullcontext
import math
from numbers import Real
import random
from typing import Dict, List, Optional, Tuple

import booster_config as cfg

Point = Tuple[float, float]


def normalize_real(value: object) -> Optional[float]:
    """Return a finite built-in float for any real scalar, else ``None``.

    UWB Kalman output contains NumPy real scalars.  Those are valid inputs, but
    booleans, complex values, textual numbers, infinities, and integers too
    large for a float must all fail closed.
    """
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def normalize_point(value: object) -> Optional[Point]:
    """Normalize a two-component real sequence to built-in finite floats."""
    if isinstance(value, (str, bytes, bytearray)):
        return None
    try:
        if len(value) != 2:  # type: ignore[arg-type]
            return None
        x = normalize_real(value[0])  # type: ignore[index]
        y = normalize_real(value[1])  # type: ignore[index]
    except (TypeError, ValueError, OverflowError, IndexError, KeyError):
        return None
    return (x, y) if x is not None and y is not None else None


def _finite_number(value: object) -> bool:
    return normalize_real(value) is not None


def _finite_point(value: object) -> bool:
    return normalize_point(value) is not None


def wrap_deg(a: object) -> Optional[float]:
    angle = normalize_real(a)
    if angle is None:
        return None
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return wrapped + 360.0 if wrapped <= -180.0 else wrapped


def arc_speed_scale(herr_deg: object) -> Optional[float]:
    """Forward-speed fraction for a given heading error (2026-07-29 arc entry).

    1.0 below ARC_FULL_SPEED_DEG, then a smooth half-cosine falloff to
    ARC_MIN_SPEED_FRAC at ARC_MAX_DEG, flat (creep) beyond. Replaces the raw
    cos(herr) alignment term so moderate misalignment produces a deliberate
    walking arc instead of a stop-and-spin, while large errors are reduced
    to a creep (the FORWARD_CONE_DEG gate still zeroes forward entirely
    past 80 deg).
    """
    error = normalize_real(herr_deg)
    if error is None:
        return None
    magnitude = abs(error)
    full = float(cfg.ARC_FULL_SPEED_DEG)
    edge = float(cfg.ARC_MAX_DEG)
    floor = float(cfg.ARC_MIN_SPEED_FRAC)
    if magnitude <= full:
        return 1.0
    if magnitude >= edge or edge <= full:
        return floor
    t = (magnitude - full) / (edge - full)
    return floor + (1.0 - floor) * 0.5 * (1.0 + math.cos(math.pi * t))


def bearing_deg(source: object, target: object) -> Optional[float]:
    start = normalize_point(source)
    end = normalize_point(target)
    if start is None or end is None:
        return None
    bearing = math.degrees(
        math.atan2(end[1] - start[1], end[0] - start[0])
    )
    return bearing if math.isfinite(bearing) else None


def _inflate(rect, b: float):
    x0, y0, x1, y1 = rect
    return (x0 - b, y0 - b, x1 + b, y1 + b)


def _nearest_on_rect(pos: Point, rect):
    x0, y0, x1, y1 = rect
    x, y = pos
    inside = (x0 <= x <= x1) and (y0 <= y <= y1)
    return min(max(x, x0), x1), min(max(y, y0), y1), inside


def safe_rect():
    """Arena inset by the full safe margin (red keep-out + yellow warn band).
    Targets and the robot must stay inside this rectangle."""
    x0, y0, x1, y1 = cfg.ARENA
    m = cfg.SAFE_MARGIN_M
    return (x0 + m, y0 + m, x1 - m, y1 - m)


def inside_safe(pos, extra: float = 0.0) -> bool:
    """True if pos is inside the safe interior. `extra` shrinks the rectangle
    further (used for recovery hysteresis)."""
    point = normalize_point(pos)
    margin = normalize_real(extra)
    if point is None or margin is None:
        return False
    sx0, sy0, sx1, sy1 = safe_rect()
    x, y = point
    return (
        (sx0 + margin) <= x <= (sx1 - margin)
        and (sy0 + margin) <= y <= (sy1 - margin)
    )


def normalize_obstacle(entry: object) -> Optional[Tuple[Point, float, dict]]:
    """Normalize an obstacle to ``((x, y), pair_clear_m, metadata)``.

    2026-07-30 per-robot keepout bubbles: fleet peer entries carry a third
    element, the pairwise waypoint-clearance distance for THIS pair (the
    sum of both robots' keepout bubble radii). Bare ``(x, y)`` entries
    (single-robot tests, legacy suppliers) fall back to the global
    ``ROBOT_AVOID_CLEAR_M``, as does an invalid or non-positive third
    element -- avoidance must never weaken because a supplier sent garbage.

    Fleet-internal entries may carry a fourth metadata dict used for stable
    right-of-way and diagnostics. Geometry-only callers can keep using
    :func:`normalize_peer`, which deliberately strips that metadata.
    """
    if isinstance(entry, (str, bytes, bytearray)):
        return None
    try:
        length = len(entry)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if length not in (2, 3, 4):
        return None
    try:
        point = normalize_point((entry[0], entry[1]))  # type: ignore[index]
        raw_clear = entry[2] if length >= 3 else None  # type: ignore[index]
        raw_metadata = entry[3] if length == 4 else {}  # type: ignore[index]
    except (TypeError, ValueError, OverflowError, IndexError, KeyError):
        return None
    if point is None:
        return None
    clear = normalize_real(raw_clear) if raw_clear is not None else None
    if clear is None or clear <= 0.0:
        clear = float(cfg.ROBOT_AVOID_CLEAR_M)
    metadata = dict(raw_metadata) if isinstance(raw_metadata, dict) else {}
    return point, clear, metadata


def normalize_peer(entry: object) -> Optional[Tuple[Point, float]]:
    """Geometry-only view of :func:`normalize_obstacle`."""
    normalized = normalize_obstacle(entry)
    if normalized is None:
        return None
    point, clear, _metadata = normalized
    return point, clear


def peer_pair_distances(pair_clear_m: float) -> Tuple[float, float, float, float]:
    """``(clear, margin, hold, hyst)`` for one robot pair.

    The global constants define the layer RATIOS; ``pair_clear_m`` (sum of
    the pair's keepout radii) sets the overall size. Uniform scaling keeps
    the layers ordered exactly as the globals are (repulsion margin >
    waypoint clear > hold ring), so asymmetric per-robot bubbles can never
    invert the avoidance layering. The all-default pair reproduces the
    global constants unchanged.

    2026-07-30 near-collision fix: the SAFETY rings (margin, hold, hyst)
    never scale below the physical floor ROBOT_HOLD_FLOOR_M on the hold
    ring -- small operator bubbles used to shrink the hard stop under the
    robots' own body size + stopping distance. The waypoint ``clear``
    distance still scales freely (spacing stays an operator choice); only
    the last-resort layers are floored, and margin/hyst keep their global
    ratios to the floored hold so repulsion always starts outside it.
    """
    base_clear = float(cfg.ROBOT_AVOID_CLEAR_M)
    scale = (float(pair_clear_m) / base_clear) if base_clear > 0.0 else 1.0
    hold_base = float(cfg.ROBOT_HOLD_DIST_M)
    safety_scale = scale
    if hold_base > 0.0:
        safety_scale = max(scale, float(cfg.ROBOT_HOLD_FLOOR_M) / hold_base)
    return (
        float(pair_clear_m),
        float(cfg.ROBOT_AVOID_MARGIN_M) * safety_scale,
        hold_base * safety_scale,
        float(cfg.ROBOT_HOLD_HYST_M) * safety_scale,
    )


def peer_hold_active(pos, peers, *, currently_holding) -> bool:
    """Hard forward-hold ring against other robots, with release hysteresis.

    Enters when any peer is inside its pair's hold ring (the global
    ROBOT_HOLD_DIST_M scaled by that pair's keepout sum -- see
    peer_pair_distances); once holding, releases only when ALL peers are
    beyond their ring + hysteresis. An invalid ``pos`` means the caller has
    no trusted fix and its own staleness gates already hold the robot, so
    the current hold state is returned unchanged. Invalid peer entries are
    dropped.
    """
    holding = bool(currently_holding)
    point = normalize_point(pos)
    if point is None:
        return holding
    try:
        entries = [normalize_peer(peer) for peer in peers]
    except Exception:
        return holding
    x, y = point
    for entry in entries:
        if entry is None:
            continue
        (px, py), pair_clear = entry
        _, _, hold, hyst = peer_pair_distances(pair_clear)
        threshold = hold + (hyst if holding else 0.0)
        if math.hypot(x - px, y - py) <= threshold:
            return True
    return False


# ---- peer-bubble path planning (2026-07-30) ---------------------------------
# Pure tangent-detour geometry: when the straight segment to the target
# passes within a bubble's planning ring (pair clear + PLAN_DETOUR_MARGIN_M),
# route via waypoints on that ring instead of walking into the reactive
# repulsion/hold layers. Everything here is pure-function math on normalized
# floats; the Navigator owns the (tiny) side-memory state for hysteresis.


def segment_point_distance(a: object, b: object, c: object) -> Optional[float]:
    """Minimum distance from segment ``a-b`` to point ``c``; None on garbage."""
    p = normalize_point(a)
    q = normalize_point(b)
    point = normalize_point(c)
    if p is None or q is None or point is None:
        return None
    vx, vy = q[0] - p[0], q[1] - p[1]
    wx, wy = point[0] - p[0], point[1] - p[1]
    seg_len_sq = vx * vx + vy * vy
    if seg_len_sq <= 0.0:
        return math.hypot(wx, wy)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len_sq))
    return math.hypot(wx - t * vx, wy - t * vy)


def segment_clears_circle(a: object, b: object, center: object,
                          radius: object) -> bool:
    """True if segment ``a-b`` stays at least ``radius`` from ``center``.
    Unknown geometry fails CLOSED (counts as not clearing)."""
    distance = segment_point_distance(a, b, center)
    r = normalize_real(radius)
    if distance is None or r is None:
        return False
    return distance >= r


def _arc_points(p: Point, q: Point, center: Point, radius: float,
                orient: int, chord_clear: float) -> Optional[List[Point]]:
    """Waypoints on the ``radius`` circle from the tangent point of ``p`` to
    the tangent point of ``q``, traversed in ``orient`` direction (+1 CCW).
    Interior points are spaced so every chord stays outside ``chord_clear``
    (the actual bubble). None when either endpoint is inside the circle."""
    dp = math.hypot(p[0] - center[0], p[1] - center[1])
    dq = math.hypot(q[0] - center[0], q[1] - center[1])
    if dp <= radius or dq <= radius:
        return None
    angle_p = math.atan2(p[1] - center[1], p[0] - center[0])
    angle_q = math.atan2(q[1] - center[1], q[0] - center[0])
    gamma_p = math.acos(max(-1.0, min(1.0, radius / dp)))
    gamma_q = math.acos(max(-1.0, min(1.0, radius / dq)))
    theta_p = angle_p + orient * gamma_p
    theta_q = angle_q - orient * gamma_q
    two_pi = 2.0 * math.pi
    delta = theta_q - theta_p
    if orient > 0:
        delta %= two_pi
    else:
        delta = -((-delta) % two_pi)
    if 0.0 < chord_clear < radius:
        phi_max = 2.0 * math.acos(max(-1.0, min(1.0, chord_clear / radius)))
    else:
        phi_max = math.pi / 6.0
    if phi_max <= 1e-6:
        phi_max = math.pi / 6.0
    # Per-obstacle step cap: a forced long-way arc may want many chords;
    # the reactive layers still protect if spacing gets coarse.
    steps = max(1, min(4, math.ceil(abs(delta) / phi_max)))
    return [
        (
            center[0] + radius * math.cos(theta_p + delta * (i / steps)),
            center[1] + radius * math.sin(theta_p + delta * (i / steps)),
        )
        for i in range(steps + 1)
    ]


def _detour_waypoints(p: Point, q: Point, center: Point, clear: float,
                      ring: float, line_side: int) -> Optional[List[Point]]:
    """Tangent detour waypoints around one bubble, on ``line_side`` of the
    segment ``p-q`` (+1 = left of travel). Waypoints are clamped into the
    safe rectangle; the side FAILS (None) if clamping would drag a point
    inside the bubble itself, so the caller can try the other side. Also
    None when either endpoint is inside the physical clear circle. Start-inside
    planning-ring cases are handled earlier by the explicit escape planner."""
    dp = math.hypot(p[0] - center[0], p[1] - center[1])
    dq = math.hypot(q[0] - center[0], q[1] - center[1])
    if dp <= clear + 1e-6 or dq <= clear + 1e-6:
        return None
    # Endpoints inside the planning ring (e.g. a target picked in the margin
    # band) shrink the ring toward the bubble so tangents stay constructible.
    radius = min(ring, dp - 1e-3, dq - 1e-3)
    if radius <= clear + 1e-6:
        return None
    rect = safe_rect()
    for orient in (1, -1):
        points = _arc_points(p, q, center, radius, orient, clear)
        if not points:
            continue
        mid = points[len(points) // 2]
        cross = (
            (q[0] - p[0]) * (mid[1] - p[1])
            - (q[1] - p[1]) * (mid[0] - p[0])
        )
        if cross != 0.0 and (cross > 0.0) != (line_side > 0):
            continue
        clamped: List[Point] = []
        usable = True
        for wx, wy in points:
            cx = min(max(wx, rect[0]), rect[2])
            cy = min(max(wy, rect[1]), rect[3])
            if math.hypot(cx - center[0], cy - center[1]) < clear - 1e-9:
                usable = False
                break
            clamped.append((cx, cy))
        if not usable:
            continue
        result: List[Point] = []
        for pt in clamped:
            if math.hypot(pt[0] - p[0], pt[1] - p[1]) < 0.02:
                continue
            if math.hypot(pt[0] - q[0], pt[1] - q[1]) < 0.02:
                continue
            if result and math.hypot(
                pt[0] - result[-1][0], pt[1] - result[-1][1]
            ) < 0.02:
                continue
            result.append(pt)
        return result
    return None


def _escape_waypoint(start: Point, center: Point, ring: float) -> Optional[Point]:
    """Shortest safe-rectangle point just outside ``ring`` that moves
    monotonically away from ``center``.

    The radial exit is preferred. Near a wall, exact intersections between
    the exit circle and safe rectangle provide deterministic tangent-ish
    alternatives. ``None`` means the obstacle covers the entire safe arena.
    """
    rect = safe_rect()
    exit_radius = ring + max(float(cfg.PLAN_CLEAR_HYST_M), 1e-3)
    radial = (start[0] - center[0], start[1] - center[1])
    start_dist = math.hypot(radial[0], radial[1])

    def feasible(point: Point) -> bool:
        if not (
            rect[0] - 1e-9 <= point[0] <= rect[2] + 1e-9
            and rect[1] - 1e-9 <= point[1] <= rect[3] + 1e-9
        ):
            return False
        if math.hypot(point[0] - center[0], point[1] - center[1]) < (
            exit_radius - 1e-7
        ):
            return False
        if start_dist > 1e-9:
            step = (point[0] - start[0], point[1] - start[1])
            if radial[0] * step[0] + radial[1] * step[1] < -1e-9:
                return False
        return True

    if start_dist > 1e-9:
        direct = (
            center[0] + exit_radius * radial[0] / start_dist,
            center[1] + exit_radius * radial[1] / start_dist,
        )
        if feasible(direct):
            return direct

    candidates: List[Point] = [
        (center[0] + exit_radius, center[1]),
        (center[0], center[1] + exit_radius),
        (center[0] - exit_radius, center[1]),
        (center[0], center[1] - exit_radius),
    ]
    for x in (rect[0], rect[2]):
        dx = x - center[0]
        if abs(dx) <= exit_radius:
            dy = math.sqrt(max(0.0, exit_radius * exit_radius - dx * dx))
            candidates.extend(((x, center[1] + dy), (x, center[1] - dy)))
    for y in (rect[1], rect[3]):
        dy = y - center[1]
        if abs(dy) <= exit_radius:
            dx = math.sqrt(max(0.0, exit_radius * exit_radius - dy * dy))
            candidates.extend(((center[0] + dx, y), (center[0] - dx, y)))

    usable = [point for point in candidates if feasible(point)]
    if not usable:
        return None
    return min(
        usable,
        key=lambda point: (
            math.hypot(point[0] - start[0], point[1] - start[1]),
            point[0],
            point[1],
        ),
    )


def plan_detour_route(start: object, goal: object, obstacles,
                      *, memory=()) -> Tuple[List[Point], List[dict]]:
    """Plan ``[start, w1, ..., goal]`` around circular bubbles.

    ``obstacles`` are ``((x, y), clear_m)`` entries exactly as
    ``Navigator._peers()`` yields them (peer robots and virtual test
    bubbles look identical here). Each bubble whose planning ring
        (``clear + PLAN_DETOUR_MARGIN_M``) the current route crosses gets a
    tangent detour on the shorter side; new segments are re-checked against
    the remaining bubbles (each bubble detours at most once -- plenty for
    this fleet size).

    ``memory`` is the previous plan's side memory (``{"center", "side"}``
    dicts). A remembered side wins over the geometric shorter side while
    the bubble still blocks (or sits within PLAN_CLEAR_HYST_M of blocking)
    so a symmetric approach can never flip sides tick to tick; a remembered
    side that has become unusable (wall) re-derives. Returns
    ``(route, new_memory)``.

    A start inside a planning ring is special: tangent construction is
    undefined there, so an explicit outward escape waypoint is prepended.
    Garbage inputs, a goal inside a bubble, or an unsatisfiable maze retain
    the legacy direct/reactive fallback.
    """
    p0 = normalize_point(start)
    p1 = normalize_point(goal)
    if p0 is None or p1 is None:
        return ([pt for pt in (p0, p1) if pt is not None], [])
    margin = float(cfg.PLAN_DETOUR_MARGIN_M)
    hyst = float(cfg.PLAN_CLEAR_HYST_M)
    match_m = float(cfg.PLAN_MATCH_M)
    obs: List[Tuple[Point, float, float]] = []
    try:
        entries = list(obstacles or ())
    except TypeError:
        entries = []
    for entry in entries:
        try:
            center = normalize_point(entry[0])
            clear = normalize_real(entry[1])
        except (TypeError, ValueError, IndexError, KeyError):
            continue
        if center is None or clear is None or clear <= 0.0:
            continue
        obs.append((center, clear, clear + margin))

    def remembered_side(center: Point) -> int:
        for item in memory or ():
            try:
                mem_center = normalize_point(item.get("center"))
                mem_side = item.get("side")
            except (TypeError, ValueError, AttributeError):
                continue
            if mem_center is None or mem_side not in (1, -1):
                continue
            if math.hypot(
                mem_center[0] - center[0], mem_center[1] - center[1]
            ) <= match_m:
                return mem_side
        return 0

    prefix: List[Point] = []
    route_start = p0
    containing = [
        (math.hypot(p0[0] - center[0], p0[1] - center[1]), center, ring)
        for center, _clear, ring in obs
        if math.hypot(p0[0] - center[0], p0[1] - center[1]) < ring
    ]
    if containing:
        # Deepest normalized intrusion first. Stable center coordinates make
        # coincident/multi-obstacle input deterministic.
        _distance, center, ring = min(
            containing,
            key=lambda item: (
                item[0] / max(item[2], 1e-9),
                item[1][0],
                item[1][1],
            ),
        )
        escape = _escape_waypoint(p0, center, ring)
        if escape is None:
            return [p0], []
        prefix = [p0]
        route_start = escape

    route: List[Point] = [route_start, p1]
    handled: set = set()
    used_sides: Dict[int, int] = {}
    for _ in range(len(obs) + 1):
        block = None
        for seg_i in range(len(route) - 1):
            for oi, (center, _clear, ring) in enumerate(obs):
                if oi in handled:
                    continue
                d = segment_point_distance(
                    route[seg_i], route[seg_i + 1], center
                )
                if d is not None and d < ring:
                    block = (seg_i, oi)
                    break
            if block is not None:
                break
        if block is None:
            break
        seg_i, oi = block
        center, clear, ring = obs[oi]
        handled.add(oi)
        a, b = route[seg_i], route[seg_i + 1]
        cross = (
            (b[0] - a[0]) * (center[1] - a[1])
            - (b[1] - a[1]) * (center[0] - a[0])
        )
        # Pass on the side OPPOSITE the bubble center's offset from the
        # line (the shorter deviation); a remembered side wins (hysteresis).
        geometric = -1 if cross > 0.0 else 1
        side = remembered_side(center) or geometric
        points = _detour_waypoints(a, b, center, clear, ring, side)
        if points is None:
            side = -side
            points = _detour_waypoints(a, b, center, clear, ring, side)
        if points is None:
            continue        # endpoints inside / both sides walled off:
                            # direct through -- reactive layers own it
        route[seg_i + 1:seg_i + 1] = points
        used_sides[oi] = side
    if len(route) - 2 > int(cfg.PLAN_MAX_WAYPOINTS):
        route = [route_start, p1]
        used_sides = {}
    new_memory: List[dict] = []
    for oi, (center, _clear, ring) in enumerate(obs):
        side = used_sides.get(oi, 0)
        if not side:
            side = remembered_side(center)
            if not side:
                continue
            d0 = segment_point_distance(p0, p1, center)
            if d0 is None or d0 >= ring + hyst:
                continue    # comfortably clear: forget the side
        new_memory.append({"center": center, "side": side})
    if prefix:
        route = prefix + route
    return route, new_memory


class Navigator:
    def __init__(
        self,
        peer_supplier=None,
        reservation_supplier=None,
        target_lock=None,
    ):
        self.target: Optional[Point] = None
        # Destination provenance is policy, not presentation: only arbitrary
        # AUTO roam targets may be discarded when a direct route is blocked.
        self.target_source: Optional[str] = None
        self.turn_sign = int(cfg.TURN_SIGN)
        self._keepout_inf = _inflate(cfg.KEEPOUT, cfg.KEEPOUT_BUFFER) if cfg.KEEPOUT else None
        self._recovering = False   # True while driving back from the warn/keep-out zone
        self._aligning = False     # near-target turn-in-place (anti-orbit)
        # Zero-arg callable returning freshness-filtered peer robot entries:
        # (x, y) or (x, y, pair_clear_m) with the pairwise keepout-sum clear
        # distance; None keeps the single-robot behavior unchanged.
        self._peer_supplier = peer_supplier
        # Fleet-only final-destination reservations. These are separate from
        # physical peer obstacles: they reject nearby target endpoints but
        # never create fake repulsion/hold geometry or route detours.
        self._reservation_supplier = reservation_supplier
        # Every fleet Navigator shares one RLock. Selection + assignment is
        # therefore atomic: once one robot claims a final target, another
        # selector sees it before accepting its own candidate.
        self._target_lock = target_lock
        # 2026-07-30 peer-bubble detour planning: the latest planned route
        # [w1, ..., target] (intermediate detour points then the true
        # target; [target] when direct), refreshed by plan_path() every nav
        # tick and read by the state payload for the dashboard polyline.
        self.planned_path: List[Point] = []
        self._plan_memory: List[dict] = []
        self.escape_center: Optional[Point] = None
        self.escape_label = ""
        self.escape_blocked = False
        self.target_blocked_reason = ""
        self._obstacle_error = ""
        self._auto_retry_at = -1e9
        self._auto_wait_obstacle = None
        self._auto_event = None
        self._auto_event_sent = False

    # ---- peers ----------------------------------------------------------
    def _peers(self) -> List[Tuple[Point, float]]:
        """``((x, y), pair_clear_m)`` peer entries from the injected
        supplier, normalized fail-safe: a broken supplier or malformed
        entry must never crash a nav tick (the backend's own hold/staleness
        layers still protect). Bare (x, y) entries get the global clear
        distance (see normalize_peer)."""
        if self._peer_supplier is None:
            return []
        try:
            peers = []
            for entry in self._peer_supplier():
                normalized = normalize_peer(entry)
                if normalized is not None:
                    peers.append(normalized)
            return peers
        except Exception:
            return []

    def _obstacles(self) -> List[Tuple[Point, float, dict]]:
        if self._peer_supplier is None:
            self._obstacle_error = ""
            return []
        try:
            obstacles = []
            for entry in self._peer_supplier():
                normalized = normalize_obstacle(entry)
                if normalized is None:
                    raise ValueError("malformed obstacle entry")
                obstacles.append(normalized)
            self._obstacle_error = ""
            return obstacles
        except Exception as exc:
            self._obstacle_error = f"obstacle feed unavailable: {type(exc).__name__}"
            return []

    def _reservations(self) -> List[Tuple[Point, float, dict]]:
        if self._reservation_supplier is None:
            return []
        try:
            reservations = []
            for entry in self._reservation_supplier():
                normalized = normalize_obstacle(entry)
                if normalized is not None:
                    reservations.append(normalized)
            return reservations
        except Exception:
            return []

    def _target_guard(self):
        return self._target_lock if self._target_lock is not None else nullcontext()

    @staticmethod
    def _reservation_conflict(
        candidate: Point,
        reservations: List[Tuple[Point, float, dict]],
    ):
        """Return the closest final-target reservation containing candidate.

        Fleet reservation entries carry the pair keepout sum as ``clear``.
        Adding the same planning margin used by physical target selection
        gives one coherent endpoint reservation radius.
        """
        conflicts = []
        for center, clear, metadata in reservations:
            required = clear + float(cfg.PLAN_DETOUR_MARGIN_M)
            distance = math.hypot(
                candidate[0] - center[0], candidate[1] - center[1]
            )
            if distance < required:
                conflicts.append(
                    (
                        distance / max(required, 1e-9),
                        center[0],
                        center[1],
                        {
                            "center": center,
                            "clear": clear,
                            "metadata": metadata,
                            "label": str(
                                metadata.get("label")
                                or metadata.get("name")
                                or "PEER"
                            ),
                        },
                    )
                )
        return (
            min(conflicts, key=lambda item: item[:3])[-1]
            if conflicts
            else None
        )

    # ---- target selection ---------------------------------------------------
    def _candidate_ok(self, point: Point, tx: float, ty: float,
                      peers: List[Tuple[Point, float]]) -> bool:
        """Shared waypoint constraints: keepout, minimum step, per-pair
        peer clear ring (arena margins are enforced by the sampling
        bounds)."""
        inf = self._keepout_inf
        if inf and (inf[0] <= tx <= inf[2] and inf[1] <= ty <= inf[3]):
            return False
        if math.hypot(tx - point[0], ty - point[1]) < cfg.MIN_TARGET_STEP:
            return False
        if any(
            math.hypot(tx - px, ty - py) < clear
            for (px, py), clear in peers
        ):
            return False
        return True

    def _auto_candidate_ok(
        self,
        point: Point,
        candidate: Point,
        obstacles: List[Tuple[Point, float, dict]],
        reservations: Optional[List[Tuple[Point, float, dict]]] = None,
    ) -> bool:
        """AUTO candidates must be valid endpoints with a direct open route.

        A robot already inside a planning ring is allowed one strictly
        outward segment to escape it; every other obstacle must be cleared by
        the full pair distance plus planning margin.
        """
        if not inside_safe(candidate):
            return False
        peers = [
            (center, clear + float(cfg.PLAN_DETOUR_MARGIN_M))
            for center, clear, _metadata in obstacles
        ]
        if not self._candidate_ok(
            point, candidate[0], candidate[1], peers
        ):
            return False
        if self._reservation_conflict(
            candidate,
            self._reservations() if reservations is None else reservations,
        ) is not None:
            return False
        for center, clear, _metadata in obstacles:
            ring = clear + float(cfg.PLAN_DETOUR_MARGIN_M)
            start_dist = math.hypot(
                point[0] - center[0], point[1] - center[1]
            )
            if start_dist < ring:
                end_dist = math.hypot(
                    candidate[0] - center[0], candidate[1] - center[1]
                )
                if end_dist <= start_dist + 1e-6:
                    return False
                continue
            if not segment_clears_circle(point, candidate, center, ring):
                return False
        return True

    def _pick_open_auto_target(
        self,
        point: Point,
        obstacles: List[Tuple[Point, float, dict]],
        away_from: object = None,
    ) -> Optional[Point]:
        """Boundedly sample a direct/open AUTO destination.

        When replacing a blocked target, first use the configured away cone
        from the most intrusive obstacle, then relax to its away half-plane.
        Exhaustion returns ``None``; callers hold and retry instead of falling
        back to a known-blocked center point.
        """
        origin = normalize_point(point)
        away = normalize_point(away_from)
        if origin is None:
            return None
        away_unit = None
        if away is not None:
            vx = origin[0] - away[0]
            vy = origin[1] - away[1]
            length = math.hypot(vx, vy)
            if length >= 1e-9:
                away_unit = (vx / length, vy / length)
        xmin, ymin, xmax, ymax = safe_rect()
        reservations = self._reservations()
        attempts = max(1, int(cfg.AUTO_TARGET_PICK_ATTEMPTS))
        cone = math.cos(math.radians(float(cfg.PEER_DISENGAGE_CONE_DEG)))
        stages = (cone, 0.0) if away_unit is not None else (-1.0,)
        for cos_min in stages:
            for _ in range(attempts):
                candidate = (
                    random.uniform(xmin, xmax),
                    random.uniform(ymin, ymax),
                )
                if not self._auto_candidate_ok(
                    origin, candidate, obstacles, reservations
                ):
                    continue
                if away_unit is not None:
                    dx = candidate[0] - origin[0]
                    dy = candidate[1] - origin[1]
                    distance = math.hypot(dx, dy) or 1.0
                    alignment = (
                        dx * away_unit[0] + dy * away_unit[1]
                    ) / distance
                    if alignment < cos_min:
                        continue
                return candidate
        # Randomness gives roaming variety, but selection must still be
        # deterministic and fail closed when RNG is patched or pathological.
        base_angle = (
            math.atan2(away_unit[1], away_unit[0])
            if away_unit is not None
            else 0.0
        )
        offsets = (
            (0, 30, -30, 60, -60, 90, -90)
            if away_unit is not None
            else (0, 45, -45, 90, -90, 135, -135, 180)
        )
        for distance in (
            float(cfg.MIN_TARGET_STEP) + 0.05,
            1.5,
            2.0,
        ):
            for offset in offsets:
                angle = base_angle + math.radians(offset)
                candidate = (
                    origin[0] + distance * math.cos(angle),
                    origin[1] + distance * math.sin(angle),
                )
                if not self._auto_candidate_ok(
                    origin, candidate, obstacles, reservations
                ):
                    continue
                if away_unit is not None:
                    dx = candidate[0] - origin[0]
                    dy = candidate[1] - origin[1]
                    alignment = (
                        dx * away_unit[0] + dy * away_unit[1]
                    ) / (math.hypot(dx, dy) or 1.0)
                    if alignment < 0.0:
                        continue
                return candidate
        return None

    @staticmethod
    def _obstacle_label(metadata: dict) -> str:
        return str(
            metadata.get("label")
            or metadata.get("name")
            or "BUBBLE"
        )

    def _direct_obstruction(
        self,
        point: Point,
        target: Point,
        obstacles: List[Tuple[Point, float, dict]],
    ):
        """Return the most intrusive meaningful direct-route obstruction."""
        blocked = []
        for center, clear, metadata in obstacles:
            ring = clear + float(cfg.PLAN_DETOUR_MARGIN_M)
            start_dist = math.hypot(
                point[0] - center[0], point[1] - center[1]
            )
            target_dist = math.hypot(
                target[0] - center[0], target[1] - center[1]
            )
            distance = segment_point_distance(point, target, center)
            # Starting inside belongs to the explicit outward escape policy,
            # unless the destination itself is also trapped in the ring.
            if target_dist >= ring and start_dist < ring:
                continue
            if distance is None or distance < ring:
                blocked.append(
                    (
                        (distance if distance is not None else 0.0)
                        / max(ring, 1e-9),
                        center[0],
                        center[1],
                        {
                            "center": center,
                            "clear": clear,
                            "metadata": metadata,
                            "label": self._obstacle_label(metadata),
                        },
                    )
                )
        return min(blocked, default=None, key=lambda item: item[:3])[-1] if blocked else None

    def _assign_target(self, target: Point, source: str) -> None:
        self.target = self._clamp_safe(target)
        self.target_source = source
        self._aligning = False
        self.planned_path = [self.target]
        self._plan_memory = []
        self.target_blocked_reason = ""

    def prepare_auto_target(self, pos: object, now: float) -> dict:
        with self._target_guard():
            return self._prepare_auto_target_locked(pos, now)

    def _prepare_auto_target_locked(self, pos: object, now: float) -> dict:
        """Keep AUTO on a direct/open target without per-tick repick churn."""
        if self.target_source != "auto":
            return {"status": self.target_source or "none"}
        point = normalize_point(pos)
        timestamp = normalize_real(now)
        if point is None or timestamp is None:
            return {"status": "waiting", "reason": "waiting for open route"}
        obstacles = self._obstacles()
        obstruction = None
        target = normalize_point(self.target) if self.target is not None else None
        if target is not None:
            obstruction = self._direct_obstruction(
                point, target, obstacles
            )
            if obstruction is None:
                obstruction = self._reservation_conflict(
                    target, self._reservations()
                )
            if obstruction is None:
                self.planned_path = [target]
                self._plan_memory = []
                return {"status": "open", "target": target}
            self.target = None
            self.planned_path = []
            self._plan_memory = []
            self._auto_wait_obstacle = obstruction
            self._auto_event_sent = False
        else:
            obstruction = self._auto_wait_obstacle
        if timestamp < self._auto_retry_at:
            return {"status": "waiting", "reason": "waiting for open route"}
        candidate = self._pick_open_auto_target(
            point,
            obstacles,
            away_from=(
                obstruction.get("center")
                if isinstance(obstruction, dict)
                else None
            ),
        )
        label = (
            obstruction.get("label", "BUBBLE")
            if isinstance(obstruction, dict)
            else "BUBBLE"
        )
        if candidate is None:
            self._auto_retry_at = timestamp + float(cfg.AUTO_TARGET_RETRY_S)
            if not self._auto_event_sent:
                self._auto_event = {
                    "obstruction": label,
                    "target": None,
                    "status": "waiting",
                }
                self._auto_event_sent = True
            return {
                "status": "waiting",
                "reason": "waiting for open route",
                "obstruction": label,
            }
        self._assign_target(candidate, "auto")
        self._auto_wait_obstacle = None
        self._auto_retry_at = (
            timestamp + float(cfg.AUTO_TARGET_REPICK_COOLDOWN_S)
        )
        if not self._auto_event_sent:
            self._auto_event = {
                "obstruction": label,
                "target": candidate,
                "status": "repicked",
            }
            self._auto_event_sent = True
        return {
            "status": "repicked",
            "obstruction": label,
            "target": candidate,
        }

    def request_auto_target(self, pos: object, now: float) -> dict:
        """Begin a fresh bounded AUTO selection cycle.

        Used at the final dwell-preview transition: no reservation exists
        during observation, then selection and assignment happen atomically
        against the latest physical and target-reservation snapshots.
        """
        with self._target_guard():
            self.target = None
            self.target_source = "auto"
            self.planned_path = []
            self._plan_memory = []
            self._auto_wait_obstacle = None
            self._auto_retry_at = -1e9
            self._auto_event_sent = False
            return self._prepare_auto_target_locked(pos, now)

    def consume_auto_event(self):
        event = self._auto_event
        self._auto_event = None
        return event

    def pick_target(self, pos: Point) -> Point:
        point = normalize_point(pos)
        if point is None:
            xmin, ymin, xmax, ymax = cfg.ARENA
            return ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0)
        xmin, ymin, xmax, ymax = cfg.ARENA
        m = cfg.ARENA_MARGIN
        peers = self._peers()
        for _ in range(200):
            tx = random.uniform(xmin + m, xmax - m)
            ty = random.uniform(ymin + m, ymax - m)
            if not self._candidate_ok(point, tx, ty, peers):
                continue
            return (tx, ty)
        return ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0)

    def pick_target_away(self, pos: Point, away_from: object) -> Point:
        """Waypoint biased AWAY from ``away_from`` (peer disengage).

        Prefers candidates within PEER_DISENGAGE_CONE_DEG of the direction
        pointing from the peer through the robot, relaxing to the away
        half-plane when the cone is unreachable (robot backed against a
        wall). Every pick_target constraint still applies. Invalid or
        coincident inputs fall back to the plain picker.
        """
        point = normalize_point(pos)
        peer = normalize_point(away_from)
        if point is None or peer is None:
            return self.pick_target(pos)
        away_x = point[0] - peer[0]
        away_y = point[1] - peer[1]
        norm = math.hypot(away_x, away_y)
        if norm < 1e-9:
            # Coincident fixes give no away direction.
            return self.pick_target(pos)
        away_x /= norm
        away_y /= norm
        xmin, ymin, xmax, ymax = cfg.ARENA
        m = cfg.ARENA_MARGIN
        peers = self._peers()
        cone = math.cos(math.radians(float(cfg.PEER_DISENGAGE_CONE_DEG)))
        for cos_min in (cone, 0.0):
            for _ in range(200):
                tx = random.uniform(xmin + m, xmax - m)
                ty = random.uniform(ymin + m, ymax - m)
                if not self._candidate_ok(point, tx, ty, peers):
                    continue
                vx = tx - point[0]
                vy = ty - point[1]
                dist = math.hypot(vx, vy) or 1.0
                if (vx * away_x + vy * away_y) / dist >= cos_min:
                    return (tx, ty)
        return self.pick_target(pos)

    def new_target(self, pos: Optional[Point]) -> None:
        with self._target_guard():
            point = normalize_point(pos) if pos is not None else None
            if point is None:
                xmin, ymin, xmax, ymax = cfg.ARENA
                point = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0)
            candidate = self._pick_open_auto_target(
                point, self._obstacles()
            )
            self.target_source = "auto"
            self._auto_wait_obstacle = None
            self._auto_retry_at = -1e9
            self._auto_event_sent = False
            if candidate is None:
                self.target = None
                self.planned_path = []
                self._plan_memory = []
                return
            self._assign_target(candidate, "auto")

    def new_target_away(self, pos: Optional[Point], away_from: object) -> None:
        with self._target_guard():
            target = self.pick_target_away(
                normalize_point(pos) if pos is not None else (0.0, 0.0),
                away_from,
            )
            self._assign_target(target, "escape")

    def set_target(self, target: Point, source: str = "manual") -> None:
        with self._target_guard():
            point = normalize_point(target)
            if point is None:
                raise ValueError("target must contain two finite real numbers")
            if source not in ("auto", "manual", "escape"):
                raise ValueError(
                    "target source must be auto, manual, or escape"
                )
            point = self._clamp_safe(point)
            if source == "manual":
                conflict = self._reservation_conflict(
                    point, self._reservations()
                )
                if conflict is not None:
                    raise ValueError(
                        f"target conflicts with "
                        f"{conflict['label']} reserved target"
                    )
            self._assign_target(point, source)
            self._auto_wait_obstacle = None
            self._auto_retry_at = -1e9
            self._auto_event_sent = False

    def clear_target(self) -> None:
        """Forget the current target. With auto roaming on, update() will
        re-pick a roam target next tick; the backend pairs this with auto=False
        when the operator removes a hand-placed target."""
        with self._target_guard():
            self.target = None
            self.target_source = None
            self._aligning = False
            self.planned_path = []
            self._plan_memory = []
            self.escape_center = None
            self.escape_label = ""
            self.escape_blocked = False
            self.target_blocked_reason = ""
            self._auto_wait_obstacle = None
            self._auto_event = None
            self._auto_event_sent = False

    # ---- peer-bubble detour planning (2026-07-30) -----------------------
    def plan_path(self, pos: object) -> List[Point]:
        """Replan the route from ``pos`` to the current target around every
        peer/virtual bubble; returns and stores ``[w1, ..., target]``
        (just ``[target]`` when the direct segment is clear). A start inside a
        ring receives an outward waypoint; impossible escape geometry returns
        no carrot so update() holds instead of falling through direct."""
        target = (
            normalize_point(self.target) if self.target is not None else None
        )
        if target is None:
            self.planned_path = []
            self._plan_memory = []
            self.escape_center = None
            self.escape_label = ""
            self.escape_blocked = False
            return []
        point = normalize_point(pos)
        if point is None:
            self.planned_path = []
            self.escape_blocked = True
            self.target_blocked_reason = "invalid planner position"
            return []
        try:
            obstacles = self._obstacles()
            self.target_blocked_reason = ""
            if self._obstacle_error:
                self.planned_path = []
                self.escape_blocked = True
                self.target_blocked_reason = self._obstacle_error
                return []
            if self.target_source == "manual":
                containing_target = [
                    (
                        math.hypot(
                            target[0] - center[0],
                            target[1] - center[1],
                        )
                        / max(
                            clear + float(cfg.PLAN_DETOUR_MARGIN_M),
                            1e-9,
                        ),
                        center,
                        metadata,
                    )
                    for center, clear, metadata in obstacles
                    if math.hypot(
                        target[0] - center[0],
                        target[1] - center[1],
                    ) < clear + float(cfg.PLAN_DETOUR_MARGIN_M)
                ]
                if containing_target:
                    _intrusion, _center, metadata = min(
                        containing_target,
                        key=lambda item: (
                            item[0], item[1][0], item[1][1]
                        ),
                    )
                    label = self._obstacle_label(metadata)
                    self.target_blocked_reason = (
                        f"manual target inside {label}"
                    )
                    self.planned_path = []
                    self._plan_memory = []
                    self.escape_center = None
                    self.escape_label = ""
                    self.escape_blocked = False
                    return []
            containing = [
                (
                    math.hypot(
                        point[0] - center[0], point[1] - center[1]
                    ) / max(clear + float(cfg.PLAN_DETOUR_MARGIN_M), 1e-9),
                    center,
                    metadata,
                )
                for center, clear, metadata in obstacles
                if math.hypot(
                    point[0] - center[0], point[1] - center[1]
                ) < clear + float(cfg.PLAN_DETOUR_MARGIN_M)
            ]
            if containing:
                _intrusion, self.escape_center, metadata = min(
                    containing,
                    key=lambda item: (
                        item[0], item[1][0], item[1][1]
                    ),
                )
                self.escape_label = str(
                    metadata.get("label") or metadata.get("name") or "bubble"
                )
            else:
                self.escape_center = None
                self.escape_label = ""
            route, self._plan_memory = plan_detour_route(
                point,
                target,
                [(center, clear) for center, clear, _metadata in obstacles],
                memory=self._plan_memory,
            )
            self.escape_blocked = bool(
                self.escape_center is not None and len(route) < 2
            )
            path = route[1:] if len(route) >= 2 else []
        except Exception as exc:
            # Never silently degrade to a direct route through unknown
            # geometry when obstacle collection or planning fails.
            path = []
            self.escape_center = None
            self.escape_label = ""
            self.escape_blocked = True
            self.target_blocked_reason = (
                f"planner unavailable: {type(exc).__name__}"
            )
        self.planned_path = path
        return list(path)

    def steer_point(self, pos: object) -> Optional[Point]:
        """The immediate steering carrot: first waypoint of a fresh replan,
        or the true target when the path is direct. Never raises."""
        path = self.plan_path(pos)
        if path:
            return path[0]
        if self.escape_blocked:
            return None
        return normalize_point(self.target) if self.target is not None else None

    def flip_turn_sign(self) -> None:
        self.turn_sign *= -1

    # ---- potential field ----------------------------------------------------
    def _repulsion(self, pos: Point) -> Tuple[float, float]:
        x, y = pos
        rx = ry = 0.0
        if self._keepout_inf is not None:
            nx, ny, inside = _nearest_on_rect(pos, self._keepout_inf)
            if inside:
                cx = (self._keepout_inf[0] + self._keepout_inf[2]) / 2.0
                cy = (self._keepout_inf[1] + self._keepout_inf[3]) / 2.0
                vx, vy = x - cx, y - cy
                n = math.hypot(vx, vy) or 1.0
                rx += cfg.KEEPOUT_GAIN * vx / n
                ry += cfg.KEEPOUT_GAIN * vy / n
            else:
                d = math.hypot(x - nx, y - ny)
                if d < cfg.KEEPOUT_AVOID:
                    vx, vy = x - nx, y - ny
                    n = math.hypot(vx, vy) or 1.0
                    w = cfg.KEEPOUT_GAIN * (1.0 - d / cfg.KEEPOUT_AVOID)
                    rx += w * vx / n
                    ry += w * vy / n
        xmin, ymin, xmax, ymax = cfg.ARENA
        m = cfg.REPULSE_MARGIN_M
        if m > 0:
            if x - xmin < m:
                rx += (1.0 - (x - xmin) / m)
            if xmax - x < m:
                rx -= (1.0 - (xmax - x) / m)
            if y - ymin < m:
                ry += (1.0 - (y - ymin) / m)
            if ymax - y < m:
                ry -= (1.0 - (ymax - y) / m)
        gain = float(cfg.ROBOT_AVOID_GAIN)
        for (px, py), pair_clear in self._peers():
            # Per-pair repulsion margin: the global margin scaled by this
            # pair's keepout sum (see peer_pair_distances).
            margin = peer_pair_distances(pair_clear)[1]
            if margin <= 0:
                continue
            d = math.hypot(x - px, y - py)
            if d >= margin:
                continue
            if d == 0.0:
                # Coincident fixes give no direction: deterministic
                # full-gain push in +x.
                rx += gain
                continue
            w = gain * (1.0 - d / margin)
            rx += w * (x - px) / d
            ry += w * (y - py) / d
        return rx, ry

    # ---- turn-in-place to an absolute heading -------------------------------
    def face(self, heading_deg: float, desired_deg: float) -> Tuple[float, bool]:
        heading = normalize_real(heading_deg)
        desired = normalize_real(desired_deg)
        if heading is None or desired is None:
            return 0.0, False
        herr = wrap_deg(desired - heading)
        turn = self._turn_for_error(herr)
        if herr is None or turn is None:
            return 0.0, False
        return turn, abs(herr) <= cfg.FACE_TOL_DEG

    def _turn_for_error(self, error_deg: object) -> Optional[float]:
        error = normalize_real(error_deg)
        if error is None:
            return None
        turn = self.turn_sign * (-cfg.TURN_KP) * math.radians(error)
        if not math.isfinite(turn):
            return None
        return max(-cfg.MAX_TURN_HARD, min(cfg.MAX_TURN_HARD, turn))

    # ---- clamp any target into the safe interior ----------------------------
    def _clamp_safe(self, pt: Point) -> Point:
        point = normalize_point(pt)
        if point is None:
            raise ValueError("target must contain two finite real numbers")
        sx0, sy0, sx1, sy1 = safe_rect()
        return (
            min(max(point[0], sx0), sx1),
            min(max(point[1], sy0), sy1),
        )

    # ---- recovery: turn around and walk back toward the arena centre --------
    def _recover(self, pos: Point, heading_deg: float) -> Tuple[float, float, Dict]:
        point = normalize_point(pos)
        heading = normalize_real(heading_deg)
        if point is None or heading is None:
            return 0.0, 0.0, {
                "mode": "invalid-input",
                "target": self.target,
            }
        cx = (cfg.ARENA[0] + cfg.ARENA[2]) / 2.0
        cy = (cfg.ARENA[1] + cfg.ARENA[3]) / 2.0
        x, y = point
        desired = math.degrees(math.atan2(cy - y, cx - x))
        herr = wrap_deg(desired - heading)
        turn = self._turn_for_error(herr)
        if herr is None or turn is None:
            return 0.0, 0.0, {
                "mode": "invalid-input",
                "target": self.target,
            }
        # turn in place until roughly facing the centre, then walk back in
        forward = cfg.RECOVER_FORWARD_FRAC if abs(herr) <= cfg.HEADING_TOL_DEG else 0.0
        return forward, turn, {"mode": "recover", "target": self.target,
                               "dist": math.hypot(cx - x, cy - y), "herr": herr}

    # ---- main step ----------------------------------------------------------
    def update(self, pos: Optional[Point], heading_deg: Optional[float],
               target: Optional[Point] = None, auto_repick: bool = True
               ) -> Tuple[float, float, Dict]:
        if pos is None or heading_deg is None:
            return 0.0, 0.0, {
                "mode": "no-fix",
                "target": target if target is not None else self.target,
            }
        point = normalize_point(pos)
        heading = normalize_real(heading_deg)
        supplied_target = normalize_point(target) if target is not None else None
        current_target = (
            normalize_point(self.target) if self.target is not None else None
        )
        if (
            point is None
            or heading is None
            or (target is not None and supplied_target is None)
            or (self.target is not None and current_target is None)
        ):
            return 0.0, 0.0, {
                "mode": "invalid-input",
                "target": self.target,
            }
        pos = point
        heading_deg = heading
        if current_target is not None:
            self.target = current_target

        # ---- HARD SAFETY LAYER ------------------------------------------
        # If the robot is in the yellow warn band (or red / outside the arena),
        # ignore the target and drive straight back toward the arena centre.
        # Hysteresis (RECOVER_EXIT_M) keeps it recovering until comfortably back
        # inside the safe interior so it can't chatter on the boundary.
        if self._recovering:
            if inside_safe(pos, extra=cfg.RECOVER_EXIT_M):
                self._recovering = False
            else:
                return self._recover(pos, heading_deg)
        elif not inside_safe(pos):
            self._recovering = True
            if auto_repick:
                self.new_target(pos)   # relocate roam target to safety
            return self._recover(pos, heading_deg)

        if target is not None:
            self.set_target(supplied_target, source="manual")
        x, y = pos
        if self.target is None:
            self.new_target(pos)
        if self.target is None:
            return 0.0, 0.0, {
                "mode": "waiting_open_route",
                "reason": "waiting for open route",
                "target": None,
            }
        tx, ty = self.target
        dist = math.hypot(tx - x, ty - y)

        if dist < cfg.WAYPOINT_RADIUS:
            if auto_repick:
                self.new_target(pos)
                if self.target is None:
                    return 0.0, 0.0, {
                        "mode": "waiting_open_route",
                        "reason": "waiting for open route",
                        "target": None,
                    }
                tx, ty = self.target
                dist = math.hypot(tx - x, ty - y)
            else:
                self.planned_path = [self.target]
                return 0.0, 0.0, {"mode": "arrived", "target": self.target,
                                  "dist": dist, "herr": 0.0}

        # 2026-07-30 peer-bubble detour: steer toward the first planned
        # waypoint instead of straight through a bubble. Arrival, braking,
        # and the anti-orbit gates keep using the TRUE target -- detour
        # points are steering carrots, never destinations (no arrival
        # choreography at them; continuous replanning slides them around
        # the bubble as the robot advances).
        path = self.plan_path(pos)
        if self.target_blocked_reason:
            return 0.0, 0.0, {
                "mode": "manual_target_blocked",
                "reason": self.target_blocked_reason,
                "target": self.target,
                "dist": dist,
                "herr": 0.0,
            }
        if self.escape_blocked:
            return 0.0, 0.0, {
                "mode": "escape_blocked",
                "reason": f"no safe escape from {self.escape_label}",
                "target": self.target,
                "dist": dist,
                "herr": 0.0,
            }
        sx, sy = path[0] if path else (tx, ty)
        steer_dist = math.hypot(sx - x, sy - y)
        ax = (sx - x) / (steer_dist or 1.0)
        ay = (sy - y) / (steer_dist or 1.0)
        rpx, rpy = self._repulsion(pos)
        dx, dy = ax + rpx, ay + rpy
        if math.hypot(dx, dy) < 1e-6:
            dx, dy = ax, ay
        desired_deg = bearing_deg((0.0, 0.0), (dx, dy))
        if desired_deg is None:
            return 0.0, 0.0, {
                "mode": "invalid-input",
                "target": self.target,
            }
        herr = wrap_deg(desired_deg - heading_deg)
        turn = self._turn_for_error(herr)
        if herr is None or turn is None:
            return 0.0, 0.0, {
                "mode": "invalid-input",
                "target": self.target,
            }

        # ANTI-ORBIT: near the target the bearing changes faster than the
        # blended controller can re-aim while still moving (it floors forward at
        # ARRIVE_MIN_FRAC), so the robot circles the point. When close and
        # meaningfully misaligned, stop and turn in place until nearly facing
        # the target, then walk straight in. Hysteresis avoids walk/align chatter.
        near = dist < cfg.SLOW_RADIUS
        if self._aligning:
            if not near or abs(herr) <= cfg.NEAR_ALIGN_EXIT_DEG:
                self._aligning = False
        elif near and abs(herr) > cfg.NEAR_ALIGN_ENTER_DEG:
            self._aligning = True
        if self._aligning:
            info = {"mode": "align", "target": self.target,
                    "dist": dist, "herr": herr, "desired": desired_deg}
            if self.escape_center is not None:
                info["mode"] = "escaping_bubble"
                info["reason"] = f"escaping {self.escape_label}"
            return 0.0, turn, info

        # Blended control: don't wait to be perfectly aligned -- start walking
        # forward as soon as we're within the cone, scaling speed by how well we
        # face the target, while still turning to correct. 2026-07-29: the
        # alignment term is the graduated arc falloff (full speed <= 20 deg,
        # smooth falloff to a creep at ARC_MAX_DEG) instead of raw cos(herr),
        # so moderate misalignment produces a deliberate walking arc.
        if abs(herr) >= cfg.FORWARD_CONE_DEG:
            forward = 0.0
            mode = "turn"
        else:
            align = arc_speed_scale(herr)
            if align is None:
                return 0.0, 0.0, {
                    "mode": "invalid-input",
                    "target": self.target,
                }
            # 2026-07-29 arrival braking: taper toward the WAYPOINT_RADIUS
            # (not toward dist 0, which the old dist/SLOW_RADIUS scale plus
            # the 0.35 floor never reached -- the robot carried >= 62% of
            # walking speed into the hard-zero arrival). Full speed at
            # SLOW_RADIUS, easing to the ARRIVE_MIN_FRAC creep at the
            # radius, so the instant stop at arrival is imperceptible.
            brake_zone = max(cfg.SLOW_RADIUS - cfg.WAYPOINT_RADIUS, 1e-6)
            scale = max(
                cfg.ARRIVE_MIN_FRAC,
                min(1.0, (dist - cfg.WAYPOINT_RADIUS) / brake_zone),
            )
            forward = scale * align    # 0..1 fraction; caller scales by speed cap
            mode = "walk" if forward > 0.02 else "turn"

        if self.escape_center is not None and forward > 0.0:
            radial = (
                pos[0] - self.escape_center[0],
                pos[1] - self.escape_center[1],
            )
            if math.hypot(radial[0], radial[1]) < 1e-9:
                radial = (sx - pos[0], sy - pos[1])
            heading_rad = math.radians(heading_deg)
            separation_rate = (
                math.cos(heading_rad) * radial[0]
                + math.sin(heading_rad) * radial[1]
            )
            if separation_rate <= 1e-9:
                forward = 0.0
                mode = "turn"
        info = {"mode": mode, "target": self.target,
                "dist": dist, "herr": herr, "desired": desired_deg}
        if self.escape_center is not None:
            info["mode"] = "escaping_bubble"
            info["reason"] = f"escaping {self.escape_label}"
        return forward, turn, info
