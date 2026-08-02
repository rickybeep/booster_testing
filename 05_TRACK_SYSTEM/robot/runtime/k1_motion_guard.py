#!/usr/bin/env python3
"""Independent, finite-output locomotion lease guard for BOOSTER TRACK V10."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import socket
import tempfile
import threading
import time
import uuid
from pathlib import Path

try:
    import fcntl
except ImportError:  # Imported on Windows for offline tests only.
    fcntl = None

try:
    from .k1_guard_protocol import (
        SDK_LOCK_PATH,
        SOCKET_PATH,
        STATUS_PATH,
        decode_message,
        encode_message,
        recv_line,
        require_sdk_module_path,
        require_sdk_success,
        validate_local_peer,
    )
except ImportError:
    from k1_guard_protocol import (  # type: ignore
        SDK_LOCK_PATH,
        SOCKET_PATH,
        STATUS_PATH,
        decode_message,
        encode_message,
        recv_line,
        require_sdk_module_path,
        require_sdk_success,
        validate_local_peer,
    )


VX_MAX, VY_MAX, VYAW_MAX = 0.4, 0.2, 0.8
DEFAULT_LEASE_MS = 400
MIN_LEASE_MS, MAX_LEASE_MS = 300, 500
WATCHDOG_TICK_S = 0.02
STOP_ATTEMPTS = 5
STOP_INTERVAL_S = 0.05
ZERO = (0.0, 0.0, 0.0)
CLIENT_READ_TIMEOUT_S = 0.05
SAFE_ZERO_REJECT_CODES = frozenset((100, 400))
SDK_ERROR_CODE_RE = re.compile(r"(?:code|rc)\s*=\s*(-?\d+)", re.IGNORECASE)


def _number(value, name, limit):
    if type(value) not in (int, float):
        raise TypeError(f"{name} must be a JSON number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    if abs(number) > limit:
        raise ValueError(f"{name} exceeds guard limit")
    return number


def _positive_sequence(value):
    if type(value) is not int:
        raise TypeError("seq must be a JSON integer")
    if value <= 0:
        raise ValueError("seq must be positive")
    return value


def _safe_zero_exception_code(exc):
    match = SDK_ERROR_CODE_RE.search(str(exc))
    if match is None:
        return None
    code = int(match.group(1))
    return code if code in SAFE_ZERO_REJECT_CODES else None


class MotionGuard:
    def __init__(
        self,
        move,
        *,
        clock=time.monotonic,
        sleep=time.sleep,
        generation=None,
        status_path=STATUS_PATH,
    ):
        self._move = move
        self._clock = clock
        self._sleep = sleep
        self.generation = generation or uuid.uuid4().hex
        self.status_path = None if status_path is None else Path(status_path)
        self._lock = threading.RLock()
        self._state = "STARTING"
        self._ready = False
        self._session = None
        self._lease_ms = DEFAULT_LEASE_MS
        self._lease_started = None
        self._lease_deadline = None
        self._seq = 0
        self._desired = ZERO
        # Unknown until the SDK positively accepts a command. In particular,
        # a non-WALK zero rejection is a safe stop indication, not delivery.
        self._ack = None
        self._last_rc = None
        self._error = ""
        self._stop_reason = ""
        self._stop_burst_count = 0
        self._last_zero_outcome = "not_attempted"
        self._publish()

    def _sdk_move(self, target, *, tolerate_zero_reject: bool = False):
        try:
            rc = self._move(*target)
        except Exception as exc:
            self._last_rc = None
            reject_code = _safe_zero_exception_code(exc)
            if tolerate_zero_reject and target == ZERO and reject_code is not None:
                self._last_zero_outcome = f"rejected_safe_mode_rc_{reject_code}"
                self._error = (
                    f"SDK Move zero rejected by non-walking mode rc={reject_code} "
                    "(accepted as stopped, not as delivered)"
                )
                self._publish()
                return
            self._error = f"SDK Move raised {exc!r}"
            self._state = "DEGRADED"
            self._ready = False
            self._publish()
            raise RuntimeError(self._error) from exc
        if rc is None:
            rc = 0
        self._last_rc = rc
        if type(rc) is not int or rc != 0:
            if (
                tolerate_zero_reject
                and target == ZERO
                and rc in SAFE_ZERO_REJECT_CODES
            ):
                self._last_zero_outcome = f"rejected_safe_mode_rc_{rc}"
                self._error = (
                    f"SDK Move zero rejected by non-walking mode rc={rc} "
                    "(accepted as stopped, not as delivered)"
                )
                self._publish()
                return
            self._error = f"SDK Move returned rc={rc}"
            self._state = "DEGRADED"
            self._ready = False
            self._publish()
            raise RuntimeError(self._error)
        self._ack = target
        if target == ZERO:
            self._last_zero_outcome = "delivered"
        self._error = ""

    def startup(self):
        with self._lock:
            if self._state != "STARTING":
                return self.status()
            self._sdk_move(ZERO, tolerate_zero_reject=True)
            self._desired = ZERO
            self._state = "READY_IDLE"
            self._ready = True
            self._publish()
            return self.status()

    def _validate_owner(self, request):
        if request.get("generation") != self.generation:
            raise ValueError("guard generation mismatch")
        if (
            self._state == "ACTIVE"
            and self._lease_deadline is not None
            and self._clock() >= self._lease_deadline
        ):
            self._stop_burst("lease expired before request", "EXPIRED")
            raise ValueError("guard lease expired")
        if self._state != "ACTIVE" or self._session is None:
            raise ValueError("fresh acquisition required")
        if request.get("session") != self._session:
            raise ValueError("guard session mismatch")
        seq = _positive_sequence(request.get("seq"))
        if seq <= self._seq:
            raise ValueError("sequence must strictly increase")
        return seq

    def _refresh(self, seq):
        now = self._clock()
        self._seq = seq
        self._lease_started = now
        self._lease_deadline = now + self._lease_ms / 1000.0

    def _acquire(self, request):
        lease_ms = request.get("lease_ms", DEFAULT_LEASE_MS)
        if type(lease_ms) is not int:
            raise TypeError("lease_ms must be a JSON integer")
        if not MIN_LEASE_MS <= lease_ms <= MAX_LEASE_MS:
            raise ValueError("lease_ms must be between 300 and 500")
        if self._state not in ("READY_IDLE", "EXPIRED", "RELEASED"):
            raise ValueError("guard is not available for acquisition")
        # The HTTP bridge admits acquisition only with fresh WALK feedback.
        # Establish a positively acknowledged zero before creating ownership;
        # a safe-mode rejection here contradicts that WALK precondition.
        self._sdk_move(ZERO)
        self._desired = ZERO
        self._session = uuid.uuid4().hex
        self._lease_ms = lease_ms
        self._seq = 0
        self._state = "ACTIVE"
        self._ready = True
        self._stop_reason = ""
        self._stop_burst_count = 0
        now = self._clock()
        self._lease_started = now
        self._lease_deadline = now + lease_ms / 1000.0
        self._publish()
        return self.status(include_session=True)

    def _move_request(self, request):
        seq = self._validate_owner(request)
        target = (
            _number(request.get("vx"), "vx", VX_MAX),
            _number(request.get("vy"), "vy", VY_MAX),
            _number(request.get("vyaw"), "vyaw", VYAW_MAX),
        )
        if target != self._desired or self._ack != target:
            try:
                self._sdk_move(target)
            except RuntimeError:
                # Never leave a prior nonzero command unsupervised in
                # DEGRADED. Revoke ownership and attempt the finite stop burst
                # immediately; no subsequent renew can resurrect this lease.
                self._stop_burst("SDK move failed", "DEGRADED")
                raise
            self._desired = target
        self._refresh(seq)
        self._publish()
        return self.status(include_session=True)

    def _renew(self, request):
        seq = self._validate_owner(request)
        self._refresh(seq)
        self._publish()
        return self.status(include_session=True)

    def _stop_burst(self, reason, terminal):
        self._desired = ZERO
        self._stop_reason = reason
        self._stop_burst_count = 0
        errors = []
        for index in range(STOP_ATTEMPTS):
            try:
                self._sdk_move(ZERO, tolerate_zero_reject=True)
            except RuntimeError as exc:
                errors.append(str(exc))
            self._stop_burst_count += 1
            self._publish()
            if index + 1 < STOP_ATTEMPTS:
                self._sleep(STOP_INTERVAL_S)
        self._session = None
        self._lease_started = None
        self._lease_deadline = None
        if errors:
            self._state = "DEGRADED"
            self._ready = False
            self._error = "; ".join(errors[-2:])
        else:
            self._state = terminal
            # Rejected zeros in DAMP still count as a completed stop intent.
            self._ready = terminal in ("EXPIRED", "RELEASED")
        self._publish()
        return self.status()

    def _release(self, request):
        seq = self._validate_owner(request)
        self._seq = seq
        return self._stop_burst("explicit release", "RELEASED")

    def tick(self):
        with self._lock:
            if (
                self._state == "ACTIVE"
                and self._lease_deadline is not None
                and self._clock() >= self._lease_deadline
            ):
                return self._stop_burst("lease expired", "EXPIRED")
            self._publish()
            return self.status()

    def handle(self, request):
        if not isinstance(request, dict):
            raise TypeError("request must be an object")
        op = request.get("op")
        with self._lock:
            if op == "status":
                return self.status()
            if op == "acquire":
                return self._acquire(request)
            if op == "move":
                return self._move_request(request)
            if op == "renew":
                return self._renew(request)
            if op in ("release", "quiesce"):
                return self._release(request)
            raise ValueError("unknown guard operation")

    def status(self, include_session=False):
        with self._lock:
            now = self._clock()
            if self._lease_started is None:
                age_ms = None
            else:
                age_ms = max(0, round((now - self._lease_started) * 1000))
            if self._lease_deadline is None:
                remaining_ms = None
            else:
                remaining_ms = max(
                    0, round((self._lease_deadline - now) * 1000)
                )
            result = {
                "protocol": 1,
                "state": self._state,
                "ready": self._ready,
                "generation": self.generation,
                "lease_ms": self._lease_ms,
                "lease_age_ms": age_ms,
                "lease_remaining_ms": remaining_ms,
                "seq": self._seq,
                "desired_velocity": list(self._desired),
                "ack_velocity": None if self._ack is None else list(self._ack),
                "last_rc": self._last_rc,
                "error": self._error,
                "last_stop_reason": self._stop_reason,
                "stop_burst_count": self._stop_burst_count,
                "last_zero_outcome": self._last_zero_outcome,
                "active_nonzero": (
                    self._state == "ACTIVE" and self._desired != ZERO
                ),
            }
            if include_session:
                result["session"] = self._session
            return result

    def _publish(self):
        if self.status_path is None:
            return
        self.status_path.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(
            self.status(include_session=False),
            sort_keys=True,
            allow_nan=False,
        )
        fd, temporary = tempfile.mkstemp(
            dir=str(self.status_path.parent),
            prefix=f".{self.status_path.name}.",
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as output:
                output.write(payload)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, self.status_path)
        finally:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


class GuardServer:
    def __init__(self, guard, socket_path=SOCKET_PATH, allowed_uids=None):
        self.guard = guard
        self.socket_path = Path(socket_path)
        self.allowed_uids = allowed_uids
        self._stop = threading.Event()

    def serve(self):
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
            listener.bind(str(self.socket_path))
            os.chmod(self.socket_path, 0o660)
            listener.listen(8)
            listener.settimeout(WATCHDOG_TICK_S)
            while not self._stop.is_set():
                self.guard.tick()
                try:
                    conn, _address = listener.accept()
                except socket.timeout:
                    continue
                with conn:
                    self.serve_connection(conn)

    def serve_connection(self, conn):
        """Handle one client. A dead peer must never kill the guard: losing
        the process means a restart, a new generation, and a forced PC hard
        fault -- exactly the outage the guard exists to prevent."""
        try:
            # Bound reads so a partial same-UID frame cannot prevent the main
            # serve loop from reaching the lease-expiry tick.
            conn.settimeout(CLIENT_READ_TIMEOUT_S)
            validate_local_peer(conn, self.allowed_uids)
            request = decode_message(recv_line(conn))
            result = self.guard.handle(request)
            response = {"ok": True, "result": result}
        except Exception as exc:
            response = {
                "ok": False,
                "error": str(exc) or type(exc).__name__,
            }
        try:
            conn.sendall(encode_message(response))
        except OSError:
            # Peer vanished mid-reply; the lease still expires on its own.
            pass


def create_sdk_move():
    import booster_robotics_sdk_python as sdk

    require_sdk_module_path(sdk)
    require_sdk_success(
        sdk.ChannelFactory.Instance().Init(0, "127.0.0.1"),
        "ChannelFactory.Init",
    )
    client = sdk.B1LocoClient()
    require_sdk_success(client.Init(), "B1LocoClient.Init")
    return client.Move


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default=SOCKET_PATH)
    parser.add_argument("--status", default=STATUS_PATH)
    args = parser.parse_args()
    if fcntl is None:
        raise RuntimeError("motion guard requires Linux flock support")
    Path(SDK_LOCK_PATH).parent.mkdir(parents=True, exist_ok=True)
    with open(SDK_LOCK_PATH, "a+b") as lock_file:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        guard = MotionGuard(create_sdk_move(), status_path=args.status)
        guard.startup()
        GuardServer(guard, socket_path=args.socket).serve()


if __name__ == "__main__":
    main()
