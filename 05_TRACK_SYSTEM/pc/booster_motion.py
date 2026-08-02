"""Minimal authenticated PC client for the partner K1 WALK bridge.

The public surface is intentionally narrow: connect/read feedback, acquire a
motion-guard lease, send WALK velocity, send a simple head target, stop, and
release. It contains no mode change, DAMP, gait, audio, camera, hand, upload,
or learned-policy methods.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Optional, Tuple

import requests

import booster_config as cfg


GUARD_LEASE_MS = 400
GUARD_RENEW_S = 0.10
GUARD_RENEW_TIMEOUT_S = 0.12
GUARD_RENEW_MIN_TIMEOUT_S = 0.05
GUARD_RENEW_RETRY_S = 0.02


def _finite_json_number(name: str, value: object) -> float:
    if type(value) not in (int, float):
        raise TypeError(f"{name} must be a finite JSON number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite JSON number")
    return number


def _required_finite(mapping: object, key: str, context: str) -> float:
    if not isinstance(mapping, dict) or key not in mapping:
        raise ValueError(f"{context}.{key} is required")
    return _finite_json_number(f"{context}.{key}", mapping[key])


def _require_http_200(response: requests.Response, context: str) -> None:
    response.raise_for_status()
    if response.status_code != 200:
        raise requests.HTTPError(
            f"{context} returned unexpected HTTP {response.status_code}",
            response=response,
        )


def _parse_nav_payload(data: object, *, fallback: bool):
    if not isinstance(data, dict):
        raise ValueError("nav payload must be an object")
    if fallback:
        imu = data.get("imu")
        if not isinstance(imu, dict):
            raise ValueError("nav.imu must be an object")
        rpy, gyro = imu.get("rpy"), imu.get("gyro")
        if not isinstance(rpy, list) or len(rpy) < 3:
            raise ValueError("nav.imu.rpy must contain three numbers")
        if not isinstance(gyro, list) or len(gyro) < 3:
            raise ValueError("nav.imu.gyro must contain three numbers")
        yaw = _finite_json_number("nav.imu.rpy[2]", rpy[2])
        gyro_z = _finite_json_number("nav.imu.gyro[2]", gyro[2])
    else:
        yaw = _required_finite(data, "yaw", "nav")
        gyro_z = _required_finite(data, "gyro_z", "nav")
    source_ts = _required_finite(data, "ts", "nav")
    rate = _required_finite(data, "rate_hz", "nav")
    if source_ts <= 0.0 or rate < 0.0:
        raise ValueError("nav timestamp/rate is invalid")
    odom_value = data.get("odom")
    if not isinstance(odom_value, dict):
        raise ValueError("nav.odom must be an object")
    odom = {
        key: _required_finite(odom_value, key, "nav.odom")
        for key in ("x", "y", "theta")
    }
    return yaw, gyro_z, odom, rate, source_ts


def _parse_status_payload(data: object):
    if not isinstance(data, dict):
        raise ValueError("status payload must be an object")
    mode = data.get("mode")
    if type(mode) is not str or mode not in (*cfg.K1_MODE_LABELS, "unknown"):
        raise ValueError("status.mode is invalid")
    mode_age = _required_finite(data, "mode_age_s", "status")
    if mode_age < 0.0 or mode_age > float(cfg.MODE_STALE_S):
        raise ValueError("status.mode_age_s is stale or invalid")
    last_rc = data.get("last_rc", 0)
    if last_rc is None:
        last_rc = 0
    if type(last_rc) is not int:
        raise ValueError("status.last_rc is invalid")
    return mode, last_rc, mode_age


def _guard_velocity_matches(
    status: object, expected: tuple[float, float, float]
) -> bool:
    if not isinstance(status, dict):
        return False
    for key in ("desired_velocity", "ack_velocity"):
        value = status.get(key)
        if not isinstance(value, list) or len(value) != 3:
            return False
        try:
            parsed = tuple(
                _finite_json_number(f"guard.{key}", item) for item in value
            )
        except (TypeError, ValueError):
            return False
        if any(
            abs(actual - wanted) > 1e-9
            for actual, wanted in zip(parsed, expected)
        ):
            return False
    return True


class MotionController:
    def __init__(
        self,
        hosts,
        panel_port=8080,
        robot_name="",
        nav_yaw_stale_s=None,
        auth_token=None,
    ):
        if isinstance(hosts, (str, bytes)):
            raise TypeError("hosts must be an iterable of host strings")
        self._hosts = tuple(hosts)
        if not self._hosts or any(
            type(host) is not str or not host for host in self._hosts
        ):
            raise ValueError("hosts must contain nonempty strings")
        if type(panel_port) is not int or not 0 < panel_port < 65536:
            raise ValueError("panel_port must be a valid integer TCP port")
        if type(robot_name) is not str:
            raise TypeError("robot_name must be a string")
        if nav_yaw_stale_s is None:
            nav_yaw_stale_s = cfg.NAV_YAW_STALE_S
        nav_yaw_stale_s = _finite_json_number(
            "nav_yaw_stale_s", nav_yaw_stale_s
        )
        if not 0.30 <= nav_yaw_stale_s <= 3.0:
            raise ValueError("nav_yaw_stale_s must be within [0.30, 3.0]")
        if auth_token is not None:
            if (
                type(auth_token) is not str
                or len(auth_token) < 32
                or any(character.isspace() for character in auth_token)
            ):
                raise ValueError(
                    "auth_token must be at least 32 non-whitespace characters"
                )

        self.robot_name = robot_name
        self._panel_port = panel_port
        self._nav_yaw_stale_s = nav_yaw_stale_s
        # Several fail-closed paths escalate while already holding state.
        # An RLock keeps that escalation atomic instead of deadlocking the
        # watchdog or caller during a transport loss.
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.Lock()
        self._move_io_lock = threading.RLock()
        self._poll_http_lock = threading.Lock()
        self._aux_http_lock = threading.Lock()
        self._clock = time.monotonic
        self._sleep = time.sleep

        self._move_session = requests.Session()
        self._poll_session = requests.Session()
        self._aux_session = requests.Session()
        for session in (
            self._move_session,
            self._poll_session,
            self._aux_session,
        ):
            session.trust_env = False
            if auth_token is not None:
                session.headers.update(
                    {"Authorization": f"Bearer {auth_token}"}
                )

        self._host: Optional[str] = None
        self._connected = False
        self._shutting_down = False
        self._sessions_closed = False
        self._phase = "offline"
        self._last_error = ""
        self._actual_mode = "unknown"
        self._mode_valid_mono: Optional[float] = None
        self._last_rc = 0

        self._yaw: Optional[float] = None
        self._gyro_z = 0.0
        self._odom = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self._imu_yaw_sign = cfg.IMU_YAW_SIGN
        self._imu_yaw_offset_deg = cfg.IMU_YAW_OFFSET_DEG
        self._yaw_source_ts: Optional[float] = None
        self._yaw_progress_mono: Optional[float] = None
        self._state_times = deque(maxlen=160)
        self._last_good_state_mono: Optional[float] = None
        self._nav_fallback = False

        self.max_forward = float(cfg.MAX_FORWARD)
        self.max_turn = float(cfg.MAX_TURN)
        self._armed = False
        self._guard_status: dict = {}
        self._guard_generation: Optional[str] = None
        self._guard_session: Optional[str] = None
        self._guard_seq = 0
        self._guard_epoch = 0
        self._guard_hard_fault = False
        self._lease_proof_mono: Optional[float] = None
        self._release_in_progress = False

        self._move_generation = 0
        self._last_cmd = (0.0, 0.0)
        self._last_cmd_ts = 0.0
        self._stop_delivered = False
        self._stop_request_seq = 0
        self._stop_ack_seq = 0
        self._last_stop_ack_mono: Optional[float] = None
        self._deadman_tripped = False

        self._stop_evt = threading.Event()
        self._poll_thread = None
        self._status_thread = None
        self._watchdog_thread = None

    @property
    def hosts(self) -> tuple[str, ...]:
        return self._hosts

    @property
    def panel_port(self) -> int:
        return self._panel_port

    def _url(self, path: str, host: Optional[str] = None) -> str:
        selected = host or self._host or self._hosts[0]
        return f"http://{selected}:{self._panel_port}{path}"

    def _resolve_host(self) -> Optional[str]:
        for host in self._hosts:
            try:
                with self._poll_http_lock:
                    response = self._poll_session.get(
                        self._url("/api/status", host),
                        timeout=1.0,
                        allow_redirects=False,
                    )
                _require_http_200(response, "status probe")
                payload = response.json()
                if not isinstance(payload, dict):
                    continue
                capabilities = payload.get("capabilities")
                if (
                    isinstance(capabilities, dict)
                    and capabilities.get("motion_guard") is True
                    and capabilities.get("motion_guard_protocol") == 1
                ):
                    return host
            except (requests.RequestException, ValueError):
                continue
        return None

    def _move_post(self, body: dict, timeout: float, *, host: str) -> dict:
        response = self._move_session.post(
            self._url("/api/move", host),
            json=body,
            timeout=float(timeout),
            allow_redirects=False,
        )
        _require_http_200(response, "move")
        payload = response.json()
        if not isinstance(payload, dict):
            raise ValueError("move response must be an object")
        return payload

    def _guard_control_post(
        self, path: str, body: dict, timeout: float
    ) -> dict:
        with self._move_io_lock:
            with self._lock:
                host = self._host
                unavailable = self._shutting_down or self._sessions_closed
            if host is None or unavailable:
                raise requests.RequestException(
                    "guard transport is unavailable"
                )
            response = self._move_session.post(
                self._url(path, host),
                json=body,
                timeout=float(timeout),
                allow_redirects=False,
            )
            _require_http_200(response, path)
            payload = response.json()
            if not isinstance(payload, dict):
                raise ValueError("guard response must be an object")
            return payload

    def _fetch_guard_status(self) -> dict:
        with self._poll_http_lock:
            with self._lock:
                host = self._host
            if host is None:
                raise requests.RequestException("no robot host is selected")
            response = self._poll_session.get(
                self._url("/api/guard/status", host),
                timeout=0.35,
                allow_redirects=False,
            )
            _require_http_200(response, "guard status")
            payload = response.json()
        if not isinstance(payload, dict):
            raise ValueError("guard status must be an object")
        return payload

    def _hard_guard_fault(self, reason: str) -> None:
        with self._lock:
            self._guard_hard_fault = True
            self._armed = False
            self._guard_session = None
            self._guard_generation = None
            self._guard_seq = 0
            self._guard_epoch += 1
            self._lease_proof_mono = None
            self._release_in_progress = False
            self._last_cmd = (0.0, 0.0)
            self._move_generation += 1
            self._phase = "guard fault"
            self._last_error = f"MOTION GUARD HARD FAULT: {reason}"

    def _observe_guard_status(
        self,
        status: object,
        *,
        epoch: int | None = None,
        ownership_proof: bool = False,
        expected_seq: int | None = None,
    ) -> bool:
        if not isinstance(status, dict):
            self._hard_guard_fault("guard status malformed")
            return False
        generation = status.get("generation")
        state = status.get("state")
        ready = status.get("ready") is True
        with self._lock:
            if epoch is not None and epoch != self._guard_epoch:
                return True
            owned_generation = self._guard_generation
            owned_session = self._guard_session
            releasing = self._release_in_progress
        if type(generation) is not str or not generation:
            self._hard_guard_fault("guard generation missing")
            return False
        if owned_generation is not None and generation != owned_generation:
            self._hard_guard_fault("guard generation changed")
            return False
        if owned_session is not None and not releasing:
            reported_session = status.get("session")
            if ownership_proof and reported_session != owned_session:
                self._hard_guard_fault(
                    "guard ownership proof missing or mismatched"
                )
                return False
            if (
                not ownership_proof
                and reported_session is not None
                and reported_session != owned_session
            ):
                self._hard_guard_fault("guard session mismatch")
                return False
            if ownership_proof and (
                type(expected_seq) is not int
                or status.get("seq") != expected_seq
            ):
                self._hard_guard_fault(
                    "guard acknowledgement sequence mismatch"
                )
                return False
            if state != "ACTIVE" or not ready:
                self._hard_guard_fault(
                    f"guard lease lost or degraded ({state})"
                )
                return False
        elif not releasing and (not ready or state == "DEGRADED"):
            self._hard_guard_fault(f"guard unavailable ({state})")
            return False
        with self._lock:
            self._guard_status = dict(status)
            if owned_session is not None and ownership_proof:
                self._lease_proof_mono = self._clock()
        return True

    def _lease_window_locked(self) -> float:
        lease_ms = self._guard_status.get("lease_ms")
        if type(lease_ms) is not int or not 300 <= lease_ms <= 500:
            lease_ms = GUARD_LEASE_MS
        return lease_ms / 1000.0

    def _next_guard_seq(self) -> tuple[str, str, int]:
        with self._lock:
            generation = self._guard_generation
            session = self._guard_session
            if generation is None or session is None:
                raise RuntimeError("motion guard has no active ownership")
            self._guard_seq += 1
            return generation, session, self._guard_seq

    def _release_guard(self) -> bool:
        with self._move_io_lock:
            with self._lock:
                if (
                    self._guard_generation is None
                    or self._guard_session is None
                ):
                    return True
                self._release_in_progress = True
                self._guard_epoch += 1
                generation, session = (
                    self._guard_generation,
                    self._guard_session,
                )
                self._guard_seq += 1
                seq = self._guard_seq
            try:
                status = self._guard_control_post(
                    "/api/guard/release",
                    {
                        "generation": generation,
                        "session": session,
                        "seq": seq,
                    },
                    timeout=0.6,
                )
                if (
                    status.get("state") != "RELEASED"
                    or status.get("ready") is not True
                    or status.get("generation") != generation
                    or status.get("seq") != seq
                ):
                    raise ValueError("guard release was not acknowledged")
            except Exception as exc:
                self._hard_guard_fault(
                    f"release ambiguous or rejected: {exc}"
                )
                return False
            with self._lock:
                self._guard_status = dict(status)
                self._guard_session = None
                self._guard_generation = None
                self._guard_seq = 0
                self._guard_epoch += 1
                self._release_in_progress = False
            return True

    def _record_nav_failure(self, exc: Exception) -> None:
        now = self._clock()
        with self._lock:
            if not self._guard_hard_fault:
                self._last_error = f"K1 nav invalid/unreachable: {exc}"
            age = (
                now - self._last_good_state_mono
                if self._last_good_state_mono is not None
                else float("inf")
            )
            if age > 2.0:
                self._connected = False
                self._host = None
                self._actual_mode = "unknown"
                self._mode_valid_mono = None
                self._phase = "panel offline"

    def _poll_nav(self) -> None:
        path = "/api/telemetry" if self._nav_fallback else "/api/nav"
        try:
            with self._lock:
                host = self._host
            if host is None:
                raise requests.RequestException("no robot host selected")
            with self._poll_http_lock:
                response = self._poll_session.get(
                    self._url(path, host),
                    timeout=cfg.PANEL_TIMEOUT_S,
                    allow_redirects=False,
                )
            if response.status_code == 404 and not self._nav_fallback:
                self._nav_fallback = True
                return
            _require_http_200(response, path)
            yaw, gyro_z, odom, _rate, source_ts = _parse_nav_payload(
                response.json(), fallback=self._nav_fallback
            )
        except (
            requests.RequestException,
            TypeError,
            ValueError,
            KeyError,
            IndexError,
        ) as exc:
            self._record_nav_failure(exc)
            return
        now = self._clock()
        with self._lock:
            if (
                self._yaw_source_ts is not None
                and source_ts < self._yaw_source_ts
            ):
                if not self._guard_hard_fault:
                    self._last_error = "K1 nav source timestamp regressed"
                return
            self._connected = True
            if not self._guard_hard_fault:
                self._phase = "connected"
                self._last_error = ""
            self._last_good_state_mono = now
            self._state_times.append(now)
            if self._yaw_source_ts is None:
                # Establish a baseline only. A frozen snapshot present before
                # connect must not count as fresh heading; a later advancing
                # source timestamp is the first freshness proof.
                self._yaw = yaw
                self._gyro_z = gyro_z
                self._odom = odom
                self._yaw_source_ts = source_ts
            elif source_ts > self._yaw_source_ts:
                self._yaw = yaw
                self._gyro_z = gyro_z
                self._odom = odom
                self._yaw_source_ts = source_ts
                self._yaw_progress_mono = now

    def _poll_status(self) -> None:
        try:
            with self._lock:
                host = self._host
                epoch = self._guard_epoch
            if host is None:
                raise requests.RequestException("no robot host selected")
            with self._poll_http_lock:
                response = self._poll_session.get(
                    self._url("/api/status", host),
                    timeout=cfg.PANEL_TIMEOUT_S,
                    allow_redirects=False,
                )
            _require_http_200(response, "status")
            data = response.json()
            mode, last_rc, _source_age = _parse_status_payload(data)
            if not self._observe_guard_status(
                data.get("guard"), epoch=epoch
            ):
                return
        except (requests.RequestException, TypeError, ValueError) as exc:
            with self._lock:
                self._actual_mode = "unknown"
                self._mode_valid_mono = None
                if not self._guard_hard_fault:
                    self._last_error = (
                        f"K1 status invalid/unreachable: {exc}"
                    )
            return
        with self._lock:
            self._actual_mode = mode
            self._mode_valid_mono = self._clock()
            self._last_rc = last_rc

    def _poll_loop(self, stop_event: threading.Event) -> None:
        period = 1.0 / max(1.0, float(cfg.NAV_STATE_HZ))
        while not stop_event.is_set():
            started = time.perf_counter()
            with self._lock:
                host = self._host
            if host is None:
                resolved = self._resolve_host()
                if resolved is not None:
                    with self._lock:
                        self._host = resolved
                        self._yaw = None
                        self._yaw_source_ts = None
                        self._yaw_progress_mono = None
            self._poll_nav()
            stop_event.wait(
                max(0.005, period - (time.perf_counter() - started))
            )

    def _status_loop(self, stop_event: threading.Event) -> None:
        period = 1.0 / max(0.5, float(cfg.STATUS_POLL_HZ))
        while not stop_event.wait(period):
            with self._lock:
                host = self._host
            if host is not None:
                self._poll_status()

    def _start_workers(self, stop_event: threading.Event) -> None:
        self._poll_thread = threading.Thread(
            target=self._poll_loop,
            args=(stop_event,),
            name="k1-nav-poll",
            daemon=True,
        )
        self._status_thread = threading.Thread(
            target=self._status_loop,
            args=(stop_event,),
            name="k1-status-poll",
            daemon=True,
        )
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            args=(stop_event,),
            name="k1-lease-renew",
            daemon=True,
        )
        for worker in (
            self._poll_thread,
            self._status_thread,
            self._watchdog_thread,
        ):
            worker.start()

    def _stop_workers(self) -> None:
        self._stop_evt.set()
        current = threading.current_thread()
        for worker in (
            self._poll_thread,
            self._status_thread,
            self._watchdog_thread,
        ):
            if worker is not None and worker is not current:
                worker.join(timeout=3.0)

    def connect(self) -> None:
        with self._lifecycle_lock:
            with self._lock:
                if self._shutting_down or self._sessions_closed:
                    raise RuntimeError("motion controller is shut down")
            self._stop_workers()
            stop_event = threading.Event()
            self._stop_evt = stop_event
            with self._lock:
                self._connected = False
                self._host = None
                self._phase = "connecting"
                self._guard_hard_fault = False
                self._guard_generation = None
                self._guard_session = None
                self._guard_seq = 0
                self._guard_epoch += 1
                self._lease_proof_mono = None
                self._yaw = None
                self._yaw_source_ts = None
                self._yaw_progress_mono = None
                epoch = self._guard_epoch
            host = self._resolve_host()
            if host is not None:
                with self._lock:
                    self._host = host
                try:
                    guard = self._fetch_guard_status()
                    self._observe_guard_status(guard, epoch=epoch)
                    self._poll_nav()
                    self._poll_status()
                except Exception as exc:
                    self._hard_guard_fault(
                        f"guard capability/readiness check failed: {exc}"
                    )
            else:
                with self._lock:
                    self._phase = "offline"
                    self._last_error = (
                        f"K1 not reachable on configured hosts {self._hosts}"
                    )
            self._start_workers(stop_event)

    def disconnect(self) -> None:
        with self._lifecycle_lock:
            self.disarm()
            self._stop_workers()
            with self._lock:
                self._connected = False
                self._host = None
                self._actual_mode = "unknown"
                self._mode_valid_mono = None
                self._phase = "offline"

    def shutdown(self) -> None:
        with self._lifecycle_lock:
            if self._sessions_closed:
                return
            self.disarm()
            self._stop_workers()
            with self._lock:
                self._shutting_down = True
                self._connected = False
                self._host = None
                self._actual_mode = "unknown"
                self._mode_valid_mono = None
                self._phase = "offline"
            for session in (
                self._move_session,
                self._poll_session,
                self._aux_session,
            ):
                session.close()
            with self._lock:
                self._sessions_closed = True

    def arm(self) -> bool:
        with self._lock:
            connected = self._connected
            actual_mode = self._actual_mode
            mode_fresh = (
                self._mode_valid_mono is not None
                and self._clock() - self._mode_valid_mono
                <= float(cfg.MODE_STALE_S)
            )
            guard = dict(self._guard_status)
            hard_fault = self._guard_hard_fault
            releasing = self._release_in_progress
            already_owned = (
                self._armed
                and self._guard_session is not None
                and guard.get("state") == "ACTIVE"
                and guard.get("ready") is True
            )
        if already_owned:
            return True
        if hard_fault or releasing:
            with self._lock:
                self._last_error = (
                    "ARM requires a fresh reconnect after a guard fault"
                    if hard_fault
                    else "ARM is blocked while guard release is in progress"
                )
            return False
        if (
            not connected
            or not mode_fresh
            or actual_mode != "walk"
            or self.heading_deg() is None
        ):
            with self._lock:
                self._last_error = (
                    "ARM requires connected, fresh WALK mode and fresh heading"
                )
            return False
        if guard.get("state") == "ACTIVE":
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    guard = self._fetch_guard_status()
                except Exception as exc:
                    with self._lock:
                        self._last_error = f"guard status failed: {exc}"
                    return False
                if guard.get("state") != "ACTIVE":
                    break
                time.sleep(0.05)
        if not (
            guard.get("ready") is True
            and guard.get("state") in ("READY_IDLE", "EXPIRED", "RELEASED")
        ):
            with self._lock:
                self._last_error = (
                    f"motion guard is not acquirable: {guard.get('state')!r}"
                )
            return False
        try:
            acquired = self._guard_control_post(
                "/api/guard/acquire",
                {"lease_ms": GUARD_LEASE_MS},
                timeout=0.5,
            )
            generation = acquired.get("generation")
            session = acquired.get("session")
            if (
                acquired.get("ready") is not True
                or acquired.get("state") != "ACTIVE"
                or type(generation) is not str
                or not generation
                or type(session) is not str
                or not session
                or acquired.get("seq") != 0
                or not _guard_velocity_matches(
                    acquired, (0.0, 0.0, 0.0)
                )
            ):
                raise ValueError("malformed or unproven guard acquisition")
        except Exception as exc:
            self._hard_guard_fault(f"ARM acquisition failed: {exc}")
            return False
        with self._lock:
            self._guard_generation = generation
            self._guard_session = session
            self._guard_seq = 0
            self._guard_epoch += 1
            self._lease_proof_mono = self._clock()
            self._guard_status = dict(acquired)
            self._guard_hard_fault = False
            self._armed = True
            self._deadman_tripped = False
            self._last_cmd_ts = self._clock()
            self._last_cmd = (0.0, 0.0)
            self._stop_delivered = True
        return True

    def set_caps(
        self, forward: Optional[float] = None, turn: Optional[float] = None
    ) -> None:
        with self._lock:
            if forward is not None:
                self.max_forward = max(
                    0.0,
                    min(
                        cfg.MAX_FORWARD_HARD,
                        _finite_json_number("forward cap", forward),
                    ),
                )
            if turn is not None:
                self.max_turn = max(
                    0.0,
                    min(
                        cfg.MAX_TURN_HARD,
                        _finite_json_number("turn cap", turn),
                    ),
                )

    def command(self, forward: float, turn: float) -> None:
        requested_forward = _finite_json_number("forward", forward)
        requested_turn = _finite_json_number("turn", turn)
        with self._lock:
            f = max(
                -self.max_forward,
                min(self.max_forward, requested_forward),
            )
            t = max(-self.max_turn, min(self.max_turn, requested_turn))
            self._last_cmd_ts = self._clock()
            already_still = (
                abs(f) <= 1e-12
                and abs(t) <= 1e-12
                and self._stop_delivered
                and self._guard_session is not None
            )
        if abs(f) <= 1e-12 and abs(t) <= 1e-12:
            if not already_still:
                self.stop()
            return
        sdk_turn = float(cfg.K1_TURN_CMD_SIGN) * t
        with self._move_io_lock:
            with self._lock:
                now = self._clock()
                mode_fresh = (
                    self._mode_valid_mono is not None
                    and now - self._mode_valid_mono
                    <= float(cfg.MODE_STALE_S)
                )
                heading_fresh = (
                    self._yaw_progress_mono is not None
                    and now - self._yaw_progress_mono
                    <= self._nav_yaw_stale_s
                )
                if (
                    not self._armed
                    or not self._connected
                    or self._host is None
                    or self._shutting_down
                    or self._guard_hard_fault
                    or self._actual_mode != "walk"
                    or not mode_fresh
                    or not heading_fresh
                ):
                    if self._guard_session is not None:
                        self._hard_guard_fault(
                            "motion requested while transport was unavailable"
                        )
                    return
                host = self._host
                move_generation = self._move_generation
            try:
                generation, session, seq = self._next_guard_seq()
                response = self._move_post(
                    {
                        "generation": generation,
                        "session": session,
                        "seq": seq,
                        "vx": f,
                        "vy": 0.0,
                        "vyaw": sdk_turn,
                    },
                    0.25,
                    host=host,
                )
                guard = response.get("guard")
                if (
                    type(response.get("rc")) is not int
                    or response.get("rc") != 0
                    or not self._observe_guard_status(
                        guard,
                        ownership_proof=True,
                        expected_seq=seq,
                    )
                    or not _guard_velocity_matches(
                        guard, (f, 0.0, sdk_turn)
                    )
                ):
                    raise ValueError(
                        "move was not positively acknowledged"
                    )
            except Exception as exc:
                self._hard_guard_fault(
                    f"move acknowledgement failed: {exc}"
                )
                return
            with self._lock:
                if move_generation == self._move_generation:
                    self._last_cmd = (f, t)
                    self._stop_delivered = False
                    self._last_error = ""

    def stop(self) -> bool:
        with self._lock:
            has_guard = (
                self._guard_generation is not None
                and self._guard_session is not None
            )
            self._last_cmd_ts = self._clock()
            self._stop_request_seq += 1
            request_seq = self._stop_request_seq
            self._stop_delivered = False
        if not has_guard:
            with self._lock:
                self._last_cmd = (0.0, 0.0)
            return False
        with self._move_io_lock:
            with self._lock:
                host = self._host
                unavailable = self._shutting_down or host is None
            if unavailable:
                self._hard_guard_fault("zero transport unavailable")
                return False
            try:
                generation, session, seq = self._next_guard_seq()
                response = self._move_post(
                    {
                        "generation": generation,
                        "session": session,
                        "seq": seq,
                        "vx": 0.0,
                        "vy": 0.0,
                        "vyaw": 0.0,
                    },
                    0.30,
                    host=host,
                )
                guard = response.get("guard")
                if (
                    type(response.get("rc")) is not int
                    or response.get("rc") != 0
                    or not self._observe_guard_status(
                        guard,
                        ownership_proof=True,
                        expected_seq=seq,
                    )
                    or not _guard_velocity_matches(
                        guard, (0.0, 0.0, 0.0)
                    )
                ):
                    raise ValueError(
                        "zero move was not positively acknowledged"
                    )
            except Exception as exc:
                self._hard_guard_fault(
                    f"zero acknowledgement failed: {exc}"
                )
                return False
            acknowledged_at = self._clock()
            with self._lock:
                self._last_cmd = (0.0, 0.0)
                self._stop_ack_seq = max(
                    self._stop_ack_seq, request_seq
                )
                self._last_stop_ack_mono = acknowledged_at
                if request_seq == self._stop_request_seq:
                    self._stop_delivered = True
            return True

    def disarm(self) -> bool:
        with self._lock:
            has_guard = self._guard_session is not None
        if not has_guard:
            with self._lock:
                self._armed = False
                self._last_cmd = (0.0, 0.0)
            return True
        delivered = self.stop()
        with self._lock:
            self._armed = False
            self._move_generation += 1
            self._last_cmd = (0.0, 0.0)
        released = self._release_guard() if delivered else False
        return bool(delivered and released)

    def head(
        self, pitch: float, yaw: float, speed: float | None = None
    ) -> bool:
        p = max(
            -cfg.HEAD_PITCH_LIM,
            min(
                cfg.HEAD_PITCH_LIM,
                _finite_json_number("pitch", pitch),
            ),
        )
        y = max(
            -cfg.HEAD_YAW_LIM,
            min(cfg.HEAD_YAW_LIM, _finite_json_number("yaw", yaw)),
        )
        speed_value = (
            cfg.HEAD_SLEW_RAD_S
            if speed is None
            else _finite_json_number("speed", speed)
        )
        with self._lock:
            host = self._host
            allowed = (
                self._connected
                and self._armed
                and self._guard_session is not None
                and not self._guard_hard_fault
                and self._actual_mode == "walk"
                and self._mode_valid_mono is not None
                and self._clock() - self._mode_valid_mono
                <= float(cfg.MODE_STALE_S)
                and not self._shutting_down
            )
        if not allowed or host is None:
            return False
        try:
            with self._aux_http_lock:
                response = self._aux_session.post(
                    self._url("/api/head", host),
                    json={"pitch": p, "yaw": y, "speed": speed_value},
                    timeout=0.25,
                    allow_redirects=False,
                )
            _require_http_200(response, "head")
            payload = response.json()
            return (
                isinstance(payload, dict)
                and type(payload.get("rc")) is int
                and payload.get("rc") == 0
            )
        except (requests.RequestException, ValueError):
            return False

    def _watchdog_loop(self, stop_event: threading.Event) -> None:
        while not stop_event.wait(GUARD_RENEW_S):
            with self._lock:
                armed = self._armed
                owned = self._guard_session is not None
                moving = any(abs(value) > 1e-4 for value in self._last_cmd)
                age = self._clock() - self._last_cmd_ts
                connected = self._connected
                mode = self._actual_mode
                mode_fresh = (
                    self._mode_valid_mono is not None
                    and self._clock() - self._mode_valid_mono
                    <= float(cfg.MODE_STALE_S)
                )
            if not armed:
                continue
            if not connected or not mode_fresh or mode != "walk":
                self._hard_guard_fault(
                    "feedback/mode gate lost while lease-owned"
                )
                continue
            if moving and age > float(cfg.DEADMAN_SECONDS):
                with self._lock:
                    self._deadman_tripped = True
                self.stop()
                continue
            if owned and not self._renew_lease():
                continue

    def _renew_lease(self) -> bool:
        last_error = "no renewal attempted"
        while True:
            with self._lock:
                proof = self._lease_proof_mono
                remaining = (
                    0.0
                    if proof is None
                    else self._lease_window_locked()
                    - (self._clock() - proof)
                )
            if remaining <= 0.0:
                self._hard_guard_fault(
                    f"lease renewal failed or ambiguous: {last_error}"
                )
                return False
            try:
                with self._move_io_lock:
                    generation, session, seq = self._next_guard_seq()
                    status = self._guard_control_post(
                        "/api/guard/renew",
                        {
                            "generation": generation,
                            "session": session,
                            "seq": seq,
                        },
                        timeout=max(
                            GUARD_RENEW_MIN_TIMEOUT_S,
                            min(GUARD_RENEW_TIMEOUT_S, remaining),
                        ),
                    )
                return self._observe_guard_status(
                    status,
                    ownership_proof=True,
                    expected_seq=seq,
                )
            except Exception as exc:
                last_error = str(exc)
                self._sleep(GUARD_RENEW_RETRY_S)

    def set_imu_calibration(self, *, sign=None, offset_deg=None) -> None:
        if sign is not None and sign not in (-1, 1):
            raise ValueError("IMU yaw sign must be -1 or 1")
        if offset_deg is not None:
            offset_deg = _finite_json_number(
                "IMU yaw offset", offset_deg
            )
        with self._lock:
            if sign is not None:
                self._imu_yaw_sign = sign
            if offset_deg is not None:
                self._imu_yaw_offset_deg = offset_deg

    def heading_deg(self):
        with self._lock:
            yaw = self._yaw
            progress = self._yaw_progress_mono
            sign = self._imu_yaw_sign
            offset = self._imu_yaw_offset_deg
        if (
            yaw is None
            or progress is None
            or not math.isfinite(yaw)
            or self._clock() - progress > self._nav_yaw_stale_s
        ):
            return None
        heading = float(sign) * math.degrees(yaw) + float(offset)
        return heading % 360.0 if math.isfinite(heading) else None

    @property
    def yaw_rate(self) -> float:
        with self._lock:
            return self._gyro_z

    @property
    def odom(self) -> dict:
        with self._lock:
            return dict(self._odom)

    @property
    def host(self) -> Optional[str]:
        with self._lock:
            return self._host

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def actual_mode(self) -> str:
        with self._lock:
            return self._actual_mode

    @property
    def mode_fresh(self) -> bool:
        with self._lock:
            timestamp = self._mode_valid_mono
            connected = self._connected
        return bool(
            connected
            and timestamp is not None
            and self._clock() - timestamp <= float(cfg.MODE_STALE_S)
        )

    @property
    def guard_status(self) -> dict:
        with self._lock:
            return dict(self._guard_status)

    @property
    def guard_acquirable(self) -> bool:
        with self._lock:
            return bool(
                self._guard_status.get("ready")
                and self._guard_status.get("state")
                in ("READY_IDLE", "EXPIRED", "RELEASED")
                and not self._release_in_progress
                and not self._guard_hard_fault
            )

    @property
    def session_healthy(self) -> bool:
        with self._lock:
            now = self._clock()
            lease_window = self._lease_window_locked()
            return bool(
                self._armed
                and self._guard_session
                and self._guard_generation
                and self._guard_status.get("ready")
                and self._guard_status.get("state") == "ACTIVE"
                and not self._guard_hard_fault
                and not self._release_in_progress
                and self._connected
                and self._actual_mode == "walk"
                and self._mode_valid_mono is not None
                and now - self._mode_valid_mono <= float(cfg.MODE_STALE_S)
                and self._yaw_progress_mono is not None
                and now - self._yaw_progress_mono <= self._nav_yaw_stale_s
                and self._lease_proof_mono is not None
                and now - self._lease_proof_mono <= lease_window
            )

    @property
    def hard_fault(self) -> bool:
        with self._lock:
            return self._guard_hard_fault

    @property
    def deadman_tripped(self) -> bool:
        with self._lock:
            return self._deadman_tripped

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    @property
    def last_command(self) -> Tuple[float, float]:
        with self._lock:
            return self._last_cmd

    @property
    def stop_delivered(self) -> bool:
        with self._lock:
            return bool(
                self._stop_delivered
                and self._stop_ack_seq == self._stop_request_seq
            )

    @property
    def stop_ack_age_s(self) -> Optional[float]:
        with self._lock:
            timestamp = self._last_stop_ack_mono
        return (
            None
            if timestamp is None
            else max(0.0, self._clock() - timestamp)
        )
