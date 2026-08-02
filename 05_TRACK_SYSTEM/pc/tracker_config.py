"""Strict loader for the shareable tracker configuration schema."""

from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import json
import math
import os
from pathlib import Path
import re
from typing import Any

import booster_config as cfg


AUTH_TOKEN_PATTERN = re.compile(r"[A-Za-z0-9._~+/=-]{32,512}\Z")


class ConfigError(ValueError):
    """The local tracker configuration is missing, unsafe, or malformed."""


def _object(value: Any, where: str) -> dict:
    if not isinstance(value, dict):
        raise ConfigError(f"{where} must be an object")
    return value


def _keys(value: dict, allowed: set[str], required: set[str], where: str) -> None:
    missing = required - set(value)
    unknown = set(value) - allowed
    if missing:
        raise ConfigError(f"{where} missing: {', '.join(sorted(missing))}")
    if unknown:
        raise ConfigError(f"{where} unknown: {', '.join(sorted(unknown))}")


def _number(value: Any, where: str, *, minimum=None, maximum=None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{where} must be a JSON number")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{where} must be finite")
    if minimum is not None and result < minimum:
        raise ConfigError(f"{where} must be >= {minimum}")
    if maximum is not None and result > maximum:
        raise ConfigError(f"{where} must be <= {maximum}")
    return result


def _integer(value: Any, where: str, *, minimum=None, maximum=None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{where} must be a JSON integer")
    if minimum is not None and value < minimum:
        raise ConfigError(f"{where} must be >= {minimum}")
    if maximum is not None and value > maximum:
        raise ConfigError(f"{where} must be <= {maximum}")
    return value


def _point(value: Any, where: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ConfigError(f"{where} must be [x, y]")
    return (_number(value[0], f"{where}[0]"), _number(value[1], f"{where}[1]"))


@dataclass(frozen=True)
class Keepout:
    label: str
    center: tuple[float, float]
    radius_m: float


@dataclass(frozen=True)
class RobotConfig:
    name: str
    enabled: bool
    hardware_variant: str
    panel_hosts: tuple[str, ...]
    panel_port: int
    auth_token_file: str
    uwb_tag: int
    keepout_radius_m: float
    nav_yaw_stale_s: float
    imu_yaw_sign: int
    imu_yaw_offset_deg: float
    tag_forward_m: float
    tag_left_m: float
    waypoints: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class TrackerConfig:
    path: Path
    live_motion_enabled: bool
    uwb_port: str
    uwb_baud: int
    allow_unverified_lo: bool
    anchors: tuple[tuple[float, float], ...]
    anchor_to_tag_height_m: float
    position_stale_s: float
    minimum_anchors: int
    maximum_residual_m: float
    arena: tuple[float, float, float, float]
    safe_margin_m: float
    keepouts: tuple[Keepout, ...]
    control_hz: int
    dwell_s: float
    waypoint_radius_m: float
    max_forward_mps: float
    max_turn_rad_s: float
    head_scan_yaw_rad: tuple[float, ...]
    head_scan_interval_s: float
    robots: tuple[RobotConfig, ...]

    @property
    def enabled_robots(self) -> tuple[RobotConfig, ...]:
        return tuple(robot for robot in self.robots if robot.enabled)

    def apply_navigation_defaults(self) -> None:
        """Apply only validated geometry/rate values used by vendored code."""
        cfg.ARENA = self.arena
        cfg.SAFE_MARGIN_M = self.safe_margin_m
        cfg.ARENA_MARGIN = self.safe_margin_m
        cfg.REPULSE_MARGIN_M = self.safe_margin_m + 0.40
        cfg.CONTROL_HZ = self.control_hz
        cfg.NAV_STATE_HZ = self.control_hz
        cfg.WAYPOINT_RADIUS = self.waypoint_radius_m
        cfg.MAX_FORWARD = self.max_forward_mps
        cfg.MAX_TURN = self.max_turn_rad_s


def validate_enabled_safety_relationships(tracker: TrackerConfig) -> None:
    """Validate coupled limits that independent range checks cannot prove."""
    if tracker.position_stale_s > float(cfg.DEADMAN_SECONDS):
        raise ConfigError(
            "uwb.position_stale_s must not exceed the PC motion deadman "
            f"({cfg.DEADMAN_SECONDS:.2f} s)"
        )
    width = tracker.arena[2] - tracker.arena[0]
    height = tracker.arena[3] - tracker.arena[1]
    recovery_margin = tracker.safe_margin_m + float(cfg.RECOVER_EXIT_M)
    if 2.0 * recovery_margin >= min(width, height):
        raise ConfigError(
            "arena safe margin plus recovery hysteresis leaves no usable interior"
        )
    for robot in tracker.enabled_robots:
        required_wall_clearance = (
            robot.keepout_radius_m
            + tracker.max_forward_mps * float(cfg.DEADMAN_SECONDS)
            + tracker.maximum_residual_m
        )
        if tracker.safe_margin_m < required_wall_clearance:
            raise ConfigError(
                f"{robot.name} arena.safe_margin_m must cover keepout_radius_m, "
                "one motion-deadman interval at max_forward_mps, and "
                f"maximum_residual_m (requires >= {required_wall_clearance:.3f} m)"
            )
        if robot.nav_yaw_stale_s > float(cfg.MODE_STALE_S):
            raise ConfigError(
                f"{robot.name} nav_yaw_stale_s must not exceed the mode "
                f"freshness limit ({cfg.MODE_STALE_S:.2f} s)"
            )


def resolve_auth_token_path(
    tracker: TrackerConfig,
    robot: RobotConfig,
) -> Path:
    source = Path(robot.auth_token_file).expanduser()
    if not source.is_absolute():
        source = tracker.path.parent / source
    # Normalize the spelling without dereferencing the final path component;
    # read_auth_token() must inspect and reject a symlink itself.
    return Path(os.path.abspath(source))


def read_auth_token(tracker: TrackerConfig, robot: RobotConfig) -> str:
    """Read one bounded local bearer token without including it in errors."""
    source = resolve_auth_token_path(tracker, robot)
    try:
        if source.is_symlink():
            raise ConfigError(f"{robot.name} auth token file must not be a symbolic link")
        if not source.is_file():
            raise ConfigError(
                f"{robot.name} auth token path is not a regular file: {source}"
            )
        if source.stat().st_size > 4096:
            raise ConfigError(f"{robot.name} auth token file is too large")
        raw = source.read_text(encoding="ascii")
    except ConfigError:
        raise
    except (OSError, UnicodeError) as exc:
        raise ConfigError(
            f"{robot.name} auth token file is unreadable: {source} ({exc})"
        ) from None
    lines = raw.splitlines()
    if len(lines) != 1 or AUTH_TOKEN_PATTERN.fullmatch(lines[0]) is None:
        raise ConfigError(
            f"{robot.name} auth token file must contain one 32-512 character "
            "ASCII bearer token with no whitespace"
        )
    return lines[0]


def _parse_keepout(value: Any, index: int) -> Keepout:
    where = f"arena.static_keepouts[{index}]"
    item = _object(value, where)
    _keys(item, {"label", "center_m", "radius_m"}, {"label", "center_m", "radius_m"}, where)
    label = item["label"]
    if not isinstance(label, str) or not label.strip():
        raise ConfigError(f"{where}.label must be a nonempty string")
    return Keepout(
        label=label.strip(),
        center=_point(item["center_m"], f"{where}.center_m"),
        radius_m=_number(item["radius_m"], f"{where}.radius_m", minimum=0.05, maximum=10.0),
    )


def _parse_robot(value: Any, index: int) -> RobotConfig:
    where = f"robots[{index}]"
    item = _object(value, where)
    allowed = {
        "name", "enabled", "hardware_variant", "panel_hosts", "panel_port",
        "auth_token_file",
        "uwb_tag", "keepout_radius_m", "nav_yaw_stale_s", "imu_yaw_sign",
        "imu_yaw_offset_deg", "tag_offset_m", "waypoints_m",
    }
    _keys(item, allowed, allowed, where)
    name = item["name"]
    if not isinstance(name, str) or not name.strip() or len(name) > 64:
        raise ConfigError(f"{where}.name must be 1..64 characters")
    if not isinstance(item["enabled"], bool):
        raise ConfigError(f"{where}.enabled must be a JSON boolean")
    variant = item["hardware_variant"]
    if variant not in {"education_jetson", "geek_qualcomm_qrb5165"}:
        raise ConfigError(f"{where}.hardware_variant is unsupported")
    hosts = item["panel_hosts"]
    if not isinstance(hosts, list) or not hosts:
        raise ConfigError(f"{where}.panel_hosts must be a nonempty list")
    parsed_hosts = []
    for host_index, host in enumerate(hosts):
        if not isinstance(host, str):
            raise ConfigError(f"{where}.panel_hosts[{host_index}] must be an IP string")
        try:
            parsed_hosts.append(str(ipaddress.ip_address(host.strip())))
        except ValueError as exc:
            raise ConfigError(f"{where}.panel_hosts[{host_index}] is not an IP address") from exc
    offset = _object(item["tag_offset_m"], f"{where}.tag_offset_m")
    _keys(offset, {"forward", "left"}, {"forward", "left"}, f"{where}.tag_offset_m")
    points = item["waypoints_m"]
    if not isinstance(points, list) or not points:
        raise ConfigError(f"{where}.waypoints_m must be a nonempty list")
    sign = _integer(item["imu_yaw_sign"], f"{where}.imu_yaw_sign")
    if sign not in (-1, 1):
        raise ConfigError(f"{where}.imu_yaw_sign must be -1 or 1")
    auth_token_file = item["auth_token_file"]
    if not isinstance(auth_token_file, str) or not auth_token_file.strip():
        raise ConfigError(f"{where}.auth_token_file must be a nonempty path")
    return RobotConfig(
        name=name.strip(),
        enabled=item["enabled"],
        hardware_variant=variant,
        panel_hosts=tuple(parsed_hosts),
        panel_port=_integer(item["panel_port"], f"{where}.panel_port", minimum=1, maximum=65535),
        auth_token_file=auth_token_file.strip(),
        uwb_tag=_integer(item["uwb_tag"], f"{where}.uwb_tag", minimum=0),
        keepout_radius_m=_number(item["keepout_radius_m"], f"{where}.keepout_radius_m", minimum=0.30, maximum=5.0),
        nav_yaw_stale_s=_number(item["nav_yaw_stale_s"], f"{where}.nav_yaw_stale_s", minimum=0.30, maximum=3.0),
        imu_yaw_sign=sign,
        imu_yaw_offset_deg=_number(item["imu_yaw_offset_deg"], f"{where}.imu_yaw_offset_deg", minimum=-360.0, maximum=360.0),
        tag_forward_m=_number(offset["forward"], f"{where}.tag_offset_m.forward", minimum=-2.0, maximum=2.0),
        tag_left_m=_number(offset["left"], f"{where}.tag_offset_m.left", minimum=-2.0, maximum=2.0),
        waypoints=tuple(_point(point, f"{where}.waypoints_m[{point_index}]") for point_index, point in enumerate(points)),
    )


def parse_config(data: Any, *, path: Path = Path("<memory>")) -> TrackerConfig:
    root = _object(data, "root")
    _keys(root, {"schema_version", "live_motion_enabled", "uwb", "arena", "navigation", "robots"}, {"schema_version", "live_motion_enabled", "uwb", "arena", "navigation", "robots"}, "root")
    if root["schema_version"] != 1:
        raise ConfigError("schema_version must be 1")
    if not isinstance(root["live_motion_enabled"], bool):
        raise ConfigError("live_motion_enabled must be a JSON boolean")

    uwb = _object(root["uwb"], "uwb")
    uwb_keys = {"port", "baud", "allow_unverified_lo", "anchors_m", "anchor_to_tag_height_m", "position_stale_s", "minimum_anchors", "maximum_residual_m"}
    _keys(uwb, uwb_keys, uwb_keys, "uwb")
    if not isinstance(uwb["port"], str) or not uwb["port"].strip():
        raise ConfigError("uwb.port must be a nonempty string")
    if not isinstance(uwb["allow_unverified_lo"], bool):
        raise ConfigError("uwb.allow_unverified_lo must be a JSON boolean")
    anchors_raw = uwb["anchors_m"]
    if not isinstance(anchors_raw, list) or len(anchors_raw) < 3:
        raise ConfigError("uwb.anchors_m must contain at least three points")
    anchors = tuple(_point(value, f"uwb.anchors_m[{index}]") for index, value in enumerate(anchors_raw))

    arena = _object(root["arena"], "arena")
    _keys(arena, {"bounds_m", "safe_margin_m", "static_keepouts"}, {"bounds_m", "safe_margin_m", "static_keepouts"}, "arena")
    bounds_raw = arena["bounds_m"]
    if not isinstance(bounds_raw, list) or len(bounds_raw) != 4:
        raise ConfigError("arena.bounds_m must be [min_x, min_y, max_x, max_y]")
    bounds = tuple(_number(value, f"arena.bounds_m[{index}]") for index, value in enumerate(bounds_raw))
    if not bounds[0] < bounds[2] or not bounds[1] < bounds[3]:
        raise ConfigError("arena bounds must have positive width and height")
    safe_margin = _number(arena["safe_margin_m"], "arena.safe_margin_m", minimum=0.10)
    if 2.0 * safe_margin >= min(bounds[2] - bounds[0], bounds[3] - bounds[1]):
        raise ConfigError("arena.safe_margin_m leaves no safe interior")
    keepouts_raw = arena["static_keepouts"]
    if not isinstance(keepouts_raw, list):
        raise ConfigError("arena.static_keepouts must be a list")
    keepouts = tuple(_parse_keepout(value, index) for index, value in enumerate(keepouts_raw))

    nav = _object(root["navigation"], "navigation")
    nav_keys = {"control_hz", "dwell_s", "waypoint_radius_m", "max_forward_mps", "max_turn_rad_s", "head_scan_yaw_rad", "head_scan_interval_s"}
    _keys(nav, nav_keys, nav_keys, "navigation")
    scan = nav["head_scan_yaw_rad"]
    if not isinstance(scan, list) or not scan:
        raise ConfigError("navigation.head_scan_yaw_rad must be a nonempty list")
    scan_values = tuple(_number(value, f"navigation.head_scan_yaw_rad[{index}]", minimum=-1.1, maximum=1.1) for index, value in enumerate(scan))

    robots_raw = root["robots"]
    if not isinstance(robots_raw, list) or not robots_raw:
        raise ConfigError("robots must be a nonempty list")
    robots = tuple(_parse_robot(value, index) for index, value in enumerate(robots_raw))
    names = [robot.name.casefold() for robot in robots]
    tags = [robot.uwb_tag for robot in robots]
    if len(names) != len(set(names)):
        raise ConfigError("robot names must be unique (case-insensitive)")
    if len(tags) != len(set(tags)):
        raise ConfigError("robot UWB tags must be unique")

    parsed = TrackerConfig(
        path=path,
        live_motion_enabled=root["live_motion_enabled"],
        uwb_port=uwb["port"].strip(),
        uwb_baud=_integer(uwb["baud"], "uwb.baud", minimum=1200, maximum=4000000),
        allow_unverified_lo=uwb["allow_unverified_lo"],
        anchors=anchors,
        anchor_to_tag_height_m=_number(uwb["anchor_to_tag_height_m"], "uwb.anchor_to_tag_height_m", minimum=0.0, maximum=5.0),
        position_stale_s=_number(uwb["position_stale_s"], "uwb.position_stale_s", minimum=0.10, maximum=5.0),
        minimum_anchors=_integer(uwb["minimum_anchors"], "uwb.minimum_anchors", minimum=3, maximum=len(anchors)),
        maximum_residual_m=_number(uwb["maximum_residual_m"], "uwb.maximum_residual_m", minimum=0.01, maximum=5.0),
        arena=(bounds[0], bounds[1], bounds[2], bounds[3]),
        safe_margin_m=safe_margin,
        keepouts=keepouts,
        control_hz=_integer(nav["control_hz"], "navigation.control_hz", minimum=5, maximum=50),
        dwell_s=_number(nav["dwell_s"], "navigation.dwell_s", minimum=1.0, maximum=120.0),
        waypoint_radius_m=_number(nav["waypoint_radius_m"], "navigation.waypoint_radius_m", minimum=0.10, maximum=2.0),
        max_forward_mps=_number(nav["max_forward_mps"], "navigation.max_forward_mps", minimum=0.0, maximum=cfg.MAX_FORWARD_HARD),
        max_turn_rad_s=_number(nav["max_turn_rad_s"], "navigation.max_turn_rad_s", minimum=0.0, maximum=cfg.MAX_TURN_HARD),
        head_scan_yaw_rad=scan_values,
        head_scan_interval_s=_number(nav["head_scan_interval_s"], "navigation.head_scan_interval_s", minimum=0.20, maximum=30.0),
        robots=robots,
    )

    validate_enabled_safety_relationships(parsed)

    sx0, sy0 = parsed.arena[0] + parsed.safe_margin_m, parsed.arena[1] + parsed.safe_margin_m
    sx1, sy1 = parsed.arena[2] - parsed.safe_margin_m, parsed.arena[3] - parsed.safe_margin_m
    for robot in parsed.robots:
        for point in robot.waypoints:
            if not (sx0 <= point[0] <= sx1 and sy0 <= point[1] <= sy1):
                raise ConfigError(f"{robot.name} waypoint {point} is outside the safe interior")
            for keepout in parsed.keepouts:
                required = robot.keepout_radius_m + keepout.radius_m + cfg.PLAN_DETOUR_MARGIN_M
                if math.dist(point, keepout.center) < required:
                    raise ConfigError(f"{robot.name} waypoint {point} overlaps keepout {keepout.label!r}")
    return parsed


def load_config(path: str | Path) -> TrackerConfig:
    source = Path(path).resolve()
    try:
        data = json.loads(source.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise ConfigError(f"configuration not found: {source}") from None
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"configuration unreadable: {source} ({exc})") from None
    return parse_config(data, path=source)
