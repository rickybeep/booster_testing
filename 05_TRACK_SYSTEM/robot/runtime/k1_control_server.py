#!/usr/bin/env python3
"""Minimal K1 HTTP bridge for the partner tracker package.

Locomotion is never sent directly by this process. ``/api/move`` is a thin
bearer-authenticated proxy to the local Unix-socket motion guard. The only SDK
actuator retained here is ``RotateHead``. This server intentionally has no
mode-change, DAMP, camera, audio, hand, cloud, or UI endpoints.
"""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import stat
import threading
import time
from urllib.parse import urlsplit

try:
    from .k1_guard_protocol import (
        GuardClient,
        GuardProtocolError,
        require_sdk_module_path,
        require_sdk_success,
    )
except ImportError:
    from k1_guard_protocol import (  # type: ignore
        GuardClient,
        GuardProtocolError,
        require_sdk_module_path,
        require_sdk_success,
    )


PORT = int(os.environ.get("K1_TRACK_PORT", "8080"))
BIND = os.environ.get("K1_TRACK_BIND", "0.0.0.0")
MAX_BODY_BYTES = 16 * 1024
HEAD_PITCH_LIMIT = 0.60
HEAD_YAW_LIMIT = 1.20
HEAD_SPEED_LIMIT = 1.50
HEAD_PERIOD_S = 0.05
HEAD_RETRY_BASE_S = 0.25
HEAD_RETRY_MAX_S = 5.0
HEAD_MAX_FAILURES = 5
HEAD_FEEDBACK_LIMIT_TOLERANCE = 0.05
MODE_FRESH_S = 1.0
HEAD_TELEMETRY_FRESH_S = 0.75
RUNTIME_DIR = Path(
    os.environ.get("K1_TRACK_RUNTIME_DIR", "/run/booster-track-v10")
)
MODE_FILE = RUNTIME_DIR / "mode.json"
NAV_FILE = RUNTIME_DIR / "nav.json"
TELEMETRY_FILE = RUNTIME_DIR / "telemetry.json"
TOKEN_FILE = Path(
    os.environ.get(
        "K1_TRACK_TOKEN_FILE",
        "/etc/booster-track-v10/control.token",
    )
)
TOKEN_PATTERN = re.compile(r"[A-Za-z0-9._~+/=-]{32,512}\Z")
LOOPBACK_PUBLIC_GETS = frozenset({"/api/health"})

guard_client = GuardClient()
head_client = None
control_token = None
head_lock = threading.Lock()
head_state = {
    "target_pitch": None,
    "target_yaw": None,
    "current_pitch": None,
    "current_yaw": None,
    "current_source": "unknown",
    "speed": 0.5,
    "pending": False,
    "last_sdk_call_accepted": False,
    "consecutive_failures": 0,
    "_command_id": 0,
    "_feedback_ts": None,
    "_last_feedback_ts_used": None,
    "_retry_at": 0.0,
    "last_rc": None,
    "last_error": "",
}


class ModeGateError(RuntimeError):
    """The requested actuator operation is unsafe in the observed mode."""


class HeadBaselineError(RuntimeError):
    """Fresh physical head feedback is unavailable or malformed."""


def load_control_token(path: Path = TOKEN_FILE) -> str:
    """Load one nontrivial bearer token without placing it in process args."""
    path = Path(path)
    try:
        if path.is_symlink():
            raise RuntimeError("control token file must not be a symbolic link")
        info = path.stat()
        if not stat.S_ISREG(info.st_mode):
            raise RuntimeError("control token path must be a regular file")
        if os.name == "posix" and info.st_mode & 0o027:
            raise RuntimeError(
                "control token file must not be group-writable or world-accessible"
            )
        raw = path.read_text(encoding="ascii")
    except (OSError, UnicodeError) as exc:
        raise RuntimeError(f"cannot read control token file {path}") from exc
    lines = raw.splitlines()
    if len(lines) != 1 or TOKEN_PATTERN.fullmatch(lines[0]) is None:
        raise RuntimeError(
            "control token file must contain one 32-512 character bearer token"
        )
    return lines[0]


def _source_age(timestamp, *, now: float | None = None) -> float | None:
    if type(timestamp) not in (int, float):
        return None
    timestamp = float(timestamp)
    if not math.isfinite(timestamp):
        return None
    age = (time.time() if now is None else float(now)) - timestamp
    if not math.isfinite(age) or age < 0.0:
        return None
    return age


def _json_file(path: Path, fallback: dict) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else dict(fallback)
    except (OSError, ValueError):
        return dict(fallback)


def read_nav() -> dict:
    return _json_file(
        NAV_FILE,
        {"ts": 0.0, "yaw": None, "gyro_z": 0.0, "odom": {"x": 0.0, "y": 0.0, "theta": 0.0}, "rate_hz": 0.0},
    )


def read_telemetry() -> dict:
    return _json_file(
        TELEMETRY_FILE,
        {"ts": 0.0, "rate_hz": 0.0, "imu": {"rpy": [0.0, 0.0, 0.0], "gyro": [0.0, 0.0, 0.0], "acc": [0.0, 0.0, 0.0]}, "joints": [], "odom": {"x": 0.0, "y": 0.0, "theta": 0.0}},
    )


def read_head_baseline() -> tuple[float, float, float, float]:
    """Return fresh measured (pitch, yaw, timestamp, age) from LowState."""
    telemetry = read_telemetry()
    timestamp = telemetry.get("ts")
    age = _source_age(timestamp)
    if age is None or age > HEAD_TELEMETRY_FRESH_S:
        raise HeadBaselineError("fresh head telemetry is unavailable")
    joints = telemetry.get("joints")
    if not isinstance(joints, list):
        raise HeadBaselineError("head telemetry joints are unavailable")
    found = {}
    for joint in joints:
        if not isinstance(joint, dict):
            continue
        name = joint.get("name")
        if name not in ("HeadPitch", "HeadYaw"):
            continue
        if name in found:
            raise HeadBaselineError(f"duplicate {name} telemetry")
        if joint.get("lost") is not False:
            raise HeadBaselineError(f"{name} telemetry is marked lost")
        position = joint.get("q")
        if type(position) not in (int, float) or not math.isfinite(float(position)):
            raise HeadBaselineError(f"{name} telemetry is non-finite")
        found[name] = float(position)
    if set(found) != {"HeadPitch", "HeadYaw"}:
        raise HeadBaselineError("both HeadPitch and HeadYaw telemetry are required")
    if abs(found["HeadPitch"]) > HEAD_PITCH_LIMIT + HEAD_FEEDBACK_LIMIT_TOLERANCE:
        raise HeadBaselineError("HeadPitch telemetry is outside the physical envelope")
    if abs(found["HeadYaw"]) > HEAD_YAW_LIMIT + HEAD_FEEDBACK_LIMIT_TOLERANCE:
        raise HeadBaselineError("HeadYaw telemetry is outside the physical envelope")
    return found["HeadPitch"], found["HeadYaw"], float(timestamp), age


def read_mode() -> tuple[str, float | None]:
    value = _json_file(MODE_FILE, {})
    raw = value.get("mode")
    age = _source_age(value.get("ts"))
    if age is None:
        return "unknown", None
    if age > MODE_FRESH_S:
        return "unknown", age
    try:
        number = int(raw)
    except (TypeError, ValueError):
        return "unknown", age
    return MODE_NAMES.get(number, "unknown"), age


def require_fresh_walk() -> tuple[str, float]:
    mode, age = read_mode()
    if mode != "walk" or age is None or age > MODE_FRESH_S:
        age_text = "unknown" if age is None else f"{age:.3f}s"
        raise ModeGateError(
            f"fresh WALK mode required (mode={mode!r}, age={age_text})"
        )
    return mode, age


def motion_values(request: dict) -> tuple[float, float, float]:
    """Validate enough of a move to distinguish an exact zero safely."""
    return tuple(
        finite_number(request.get(name), name, -limit, limit)
        for name, limit in (
            ("vx", 0.4),
            ("vy", 0.2),
            ("vyaw", 0.8),
        )
    )


def guard_status() -> dict:
    try:
        return guard_client.status()
    except Exception as exc:
        return {
            "protocol": 1,
            "state": "UNREACHABLE",
            "ready": False,
            "error": str(exc) or type(exc).__name__,
        }


def finite_number(value, name, minimum, maximum) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be a JSON number")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{name} must be finite and within [{minimum}, {maximum}]")
    return result


def head_status(*, now: float | None = None) -> dict:
    now = time.monotonic() if now is None else float(now)
    with head_lock:
        result = {
            key: value for key, value in head_state.items() if not key.startswith("_")
        }
        retry_at = float(head_state["_retry_at"])
        feedback_ts = head_state["_feedback_ts"]
    feedback_age = _source_age(feedback_ts)
    if feedback_age is None or feedback_age > HEAD_TELEMETRY_FRESH_S:
        result.update(
            current_pitch=None,
            current_yaw=None,
            current_source="unknown",
            current_age_s=None if feedback_age is None else feedback_age,
        )
    else:
        result["current_source"] = "telemetry"
        result["current_age_s"] = feedback_age
    result["retry_in_s"] = max(0.0, retry_at - now)
    return result


def head_sender_step(now: float, dt: float) -> str:
    """Perform at most one guarded SDK head call for deterministic testing."""
    now = float(now)
    dt = max(0.001, min(0.20, float(dt)))
    with head_lock:
        if not head_state["pending"]:
            return "idle"
        if now < head_state["_retry_at"]:
            return "backoff"
        command_id = head_state["_command_id"]
        target_pitch = head_state["target_pitch"]
        target_yaw = head_state["target_yaw"]
        step = head_state["speed"] * dt

    if target_pitch is None or target_yaw is None:
        with head_lock:
            if head_state["_command_id"] != command_id:
                return "superseded"
            head_state.update(
                pending=False,
                last_sdk_call_accepted=False,
                last_rc=None,
                last_error="head target is unavailable",
            )
        return "invalid"

    try:
        current_pitch, current_yaw, feedback_ts, _age = read_head_baseline()
    except HeadBaselineError as exc:
        with head_lock:
            if head_state["_command_id"] == command_id:
                head_state.update(
                    current_pitch=None,
                    current_yaw=None,
                    current_source="unknown",
                    pending=False,
                    last_sdk_call_accepted=False,
                    last_rc=None,
                    last_error=str(exc),
                )
        return "feedback_blocked"

    with head_lock:
        if head_state["_command_id"] != command_id:
            return "superseded"
        head_state.update(
            current_pitch=current_pitch,
            current_yaw=current_yaw,
            current_source="telemetry",
            _feedback_ts=feedback_ts,
        )
        if head_state["_last_feedback_ts_used"] is not None and (
            feedback_ts <= head_state["_last_feedback_ts_used"]
        ):
            return "awaiting_feedback"

    if (
        abs(target_pitch - current_pitch) <= 1e-3
        and abs(target_yaw - current_yaw) <= 1e-3
    ):
        with head_lock:
            if head_state["_command_id"] == command_id:
                head_state.update(
                    pending=False,
                    last_error="",
                )
        return "complete"

    next_pitch = current_pitch + max(
        -step, min(step, target_pitch - current_pitch)
    )
    next_yaw = current_yaw + max(
        -step, min(step, target_yaw - current_yaw)
    )

    try:
        require_fresh_walk()
    except ModeGateError as exc:
        # Discard the queued target. A transition back to WALK must not resume
        # an old head command without a new authenticated POST.
        with head_lock:
            if head_state["_command_id"] != command_id:
                return "superseded"
            head_state.update(
                target_pitch=current_pitch,
                target_yaw=current_yaw,
                pending=False,
                last_sdk_call_accepted=False,
                _retry_at=0.0,
                last_rc=None,
                last_error=str(exc),
            )
        return "mode_blocked"

    rc = None
    try:
        rc = head_client.RotateHead(next_pitch, next_yaw)
        if rc is None:
            rc = 0
        if type(rc) is not int or rc != 0:
            raise RuntimeError(f"RotateHead returned rc={rc!r}")
    except Exception as exc:
        with head_lock:
            if head_state["_command_id"] != command_id:
                return "superseded"
            failures = int(head_state["consecutive_failures"]) + 1
            delay = min(
                HEAD_RETRY_MAX_S,
                HEAD_RETRY_BASE_S * (2 ** min(failures - 1, 16)),
            )
            abandoned = failures >= HEAD_MAX_FAILURES
            head_state.update(
                pending=False if abandoned else head_state["pending"],
                last_sdk_call_accepted=False,
                consecutive_failures=failures,
                _retry_at=now + delay,
                last_rc=rc if type(rc) is int else None,
                last_error=(
                    (str(exc) or type(exc).__name__)
                    + (
                        f"; command abandoned after {failures} failures"
                        if abandoned
                        else f"; retry in {delay:.2f}s"
                    )
                ),
            )
        return "abandoned" if abandoned else "failed"

    with head_lock:
        if head_state["_command_id"] != command_id:
            return "superseded"
        head_state.update(
            # rc=0 confirms only SDK call acceptance. Physical position remains
            # the latest telemetry sample above until a newer sample arrives.
            last_sdk_call_accepted=True,
            consecutive_failures=0,
            _last_feedback_ts_used=feedback_ts,
            _retry_at=0.0,
            last_rc=rc,
            last_error="",
        )
    return "sent"


def head_sender() -> None:
    last = time.monotonic()
    while True:
        now = time.monotonic()
        head_sender_step(now, now - last)
        last = now
        time.sleep(HEAD_PERIOD_S)


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):  # noqa: A003
        return

    def _json(
        self,
        value: dict,
        status: int = 200,
        extra_headers: dict[str, str] | None = None,
    ) -> None:
        body = json.dumps(value, allow_nan=False, sort_keys=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for name, value in (extra_headers or {}).items():
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(body)

    def _peer_is_loopback(self) -> bool:
        try:
            address = ipaddress.ip_address(self.client_address[0])
        except (ValueError, TypeError, IndexError):
            return False
        if isinstance(address, ipaddress.IPv6Address) and address.ipv4_mapped:
            address = address.ipv4_mapped
        return address.is_loopback

    def _has_valid_bearer(self) -> bool:
        if not isinstance(control_token, str):
            return False
        values = self.headers.get_all("Authorization", [])
        if len(values) != 1:
            return False
        scheme, separator, candidate = values[0].partition(" ")
        if (
            separator != " "
            or scheme.casefold() != "bearer"
            or not candidate
            or any(character.isspace() for character in candidate)
        ):
            return False
        return hmac.compare_digest(candidate, control_token)

    def _authorized(self, route: str, method: str) -> bool:
        if (
            method == "GET"
            and route in LOOPBACK_PUBLIC_GETS
            and self._peer_is_loopback()
        ):
            return True
        return self._has_valid_bearer()

    def _unauthorized(self) -> None:
        # The request body has not been consumed, so force a new connection.
        self.close_connection = True
        self._json(
            {"error": "bearer authorization required"},
            401,
            {
                "WWW-Authenticate": 'Bearer realm="booster-track-v10"',
                "Connection": "close",
            },
        )

    def _body(self) -> dict:
        if self.headers.get_all("Transfer-Encoding", []):
            raise ValueError("Transfer-Encoding is not supported")
        lengths = self.headers.get_all("Content-Length", [])
        if len(lengths) > 1:
            raise ValueError("duplicate Content-Length is not allowed")
        raw_length = lengths[0] if lengths else "0"
        try:
            length = int(raw_length)
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if not 0 <= length <= MAX_BODY_BYTES:
            raise ValueError("request body is too large")
        try:
            value = json.loads(self.rfile.read(length) or b"{}")
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("invalid JSON body") from exc
        if not isinstance(value, dict):
            raise ValueError("JSON body must be an object")
        return value

    def do_GET(self):
        route = urlsplit(self.path).path
        if not self._authorized(route, "GET"):
            return self._unauthorized()
        if route == "/api/guard/status":
            return self._json(guard_status())
        if route == "/api/nav":
            return self._json(read_nav())
        if route == "/api/telemetry":
            return self._json(read_telemetry())
        if route == "/api/status":
            mode, mode_age = read_mode()
            head = head_status()
            guard = guard_status()
            velocity = guard.get("ack_velocity")
            if not isinstance(velocity, list) or len(velocity) != 3:
                velocity = [0.0, 0.0, 0.0]
            return self._json({
                "mode": mode,
                "mode_age_s": mode_age,
                "gait": "unknown",
                "vx": velocity[0],
                "vy": velocity[1],
                "vyaw": velocity[2],
                "last_rc": guard.get("last_rc", 0),
                "head": head,
                "guard": guard,
                "capabilities": {"motion_guard": True, "motion_guard_protocol": 1, "head_slew": True},
            })
        if route == "/api/health":
            nav = read_nav()
            return self._json({
                "guard": guard_status(),
                "nav_source_age_s": _source_age(nav.get("ts")),
            })
        return self._json({"error": "not found"}, 404)

    def do_POST(self):
        route = urlsplit(self.path).path
        if not self._authorized(route, "POST"):
            return self._unauthorized()
        try:
            request = self._body()
            if route == "/api/guard/acquire":
                require_fresh_walk()
                result = guard_client.acquire(request.get("lease_ms", 400))
                return self._json(result)
            if route == "/api/guard/renew":
                require_fresh_walk()
                result = guard_client.renew(
                    generation=request.get("generation"),
                    session=request.get("session"),
                    seq=request.get("seq"),
                )
                return self._json(result)
            if route == "/api/guard/release":
                result = guard_client.release(
                    generation=request.get("generation"),
                    session=request.get("session"),
                    seq=request.get("seq"),
                )
                return self._json(result)
            if route == "/api/move":
                velocity = motion_values(request)
                if velocity != (0.0, 0.0, 0.0):
                    require_fresh_walk()
                result = guard_client.move(
                    generation=request.get("generation"),
                    session=request.get("session"),
                    seq=request.get("seq"),
                    vx=request.get("vx"),
                    vy=request.get("vy"),
                    vyaw=request.get("vyaw"),
                )
                return self._json({"rc": 0, "guard": result})
            if route == "/api/head":
                allowed = {"pitch", "yaw", "speed"}
                unknown = set(request) - allowed
                if unknown:
                    raise ValueError(f"unknown head field(s): {', '.join(sorted(unknown))}")
                pitch = finite_number(request.get("pitch", 0.0), "pitch", -HEAD_PITCH_LIMIT, HEAD_PITCH_LIMIT)
                yaw = finite_number(request.get("yaw", 0.0), "yaw", -HEAD_YAW_LIMIT, HEAD_YAW_LIMIT)
                speed = finite_number(request.get("speed", 0.5), "speed", 0.05, HEAD_SPEED_LIMIT)
                require_fresh_walk()
                current_pitch, current_yaw, feedback_ts, _age = (
                    read_head_baseline()
                )
                with head_lock:
                    head_state.update(
                        target_pitch=pitch,
                        target_yaw=yaw,
                        current_pitch=current_pitch,
                        current_yaw=current_yaw,
                        current_source="telemetry",
                        speed=speed,
                        pending=True,
                        last_sdk_call_accepted=False,
                        consecutive_failures=0,
                        _command_id=head_state["_command_id"] + 1,
                        _feedback_ts=feedback_ts,
                        _last_feedback_ts_used=None,
                        _retry_at=0.0,
                        last_rc=None,
                        last_error="",
                    )
                return self._json(
                    {
                        "rc": 0,
                        "accepted": True,
                        "queued": True,
                        "acceptance": "queued_only",
                        "sdk_call_completed": False,
                    }
                )
            return self._json({"error": "not found"}, 404)
        except ModeGateError as exc:
            return self._json({"error": str(exc)}, 409)
        except HeadBaselineError as exc:
            return self._json({"error": str(exc)}, 409)
        except ValueError as exc:
            return self._json({"error": str(exc)}, 400)
        except GuardProtocolError as exc:
            return self._json({"error": str(exc)}, 409)
        except Exception as exc:
            return self._json({"error": str(exc) or type(exc).__name__}, 503)


MODE_NAMES = {}


def main() -> None:
    global control_token, head_client
    import booster_robotics_sdk_python as sdk

    control_token = load_control_token()
    require_sdk_module_path(sdk)
    for member, label in (
        ("kDamping", "damp"),
        ("kPrepare", "prep"),
        ("kWalking", "walk"),
        ("kCustom", "custom"),
        ("kUnknown", "unknown"),
    ):
        value = getattr(sdk.RobotMode, member, None)
        if value is not None:
            MODE_NAMES[int(getattr(value, "value", value))] = label
    initial_guard = guard_status()
    if initial_guard.get("ready") is not True:
        raise RuntimeError(f"motion guard is not ready: {initial_guard.get('error', '')}")
    require_sdk_success(
        sdk.ChannelFactory.Instance().Init(0, "127.0.0.1"),
        "ChannelFactory.Init",
    )
    head_client = sdk.B1LocoClient()
    require_sdk_success(head_client.Init(), "B1LocoClient.Init")
    threading.Thread(target=head_sender, name="head-sender", daemon=True).start()
    print(f"minimal K1 tracker bridge listening on {BIND}:{PORT}", flush=True)
    Server((BIND, PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
