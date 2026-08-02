#!/usr/bin/env python3
"""Bounded local IPC protocol shared by the V10 guard and control server."""

from __future__ import annotations

import json
import os
import socket
import struct


SOCKET_PATH = "/run/booster-track-v10/motion-guard.sock"
STATUS_PATH = "/run/booster-track-v10/motion-guard-status.json"
SDK_LOCK_PATH = "/run/booster-track-v10/sdk-motion.lock"
MAX_MESSAGE_BYTES = 4096
PROTOCOL_VERSION = 1


class GuardProtocolError(RuntimeError):
    pass


def require_sdk_success(result, operation: str) -> None:
    """Accept the two success conventions observed in Booster Python calls."""
    if result is None or (type(result) is int and result == 0):
        return
    raise RuntimeError(f"{operation} returned unexpected result {result!r}")


def require_sdk_module_path(module) -> str:
    """Fail unless the imported SDK resolves under the operator-selected root."""
    expected = os.environ.get("K1_SDK_EXPECTED_ROOT", "")
    if not expected or not os.path.isabs(expected):
        raise RuntimeError("K1_SDK_EXPECTED_ROOT must be an absolute path")
    actual = getattr(module, "__file__", None)
    if not isinstance(actual, str) or not actual:
        raise RuntimeError("Booster SDK module does not expose __file__")
    expected_real = os.path.realpath(expected)
    actual_real = os.path.realpath(actual)
    try:
        contained = os.path.commonpath((expected_real, actual_real)) == expected_real
    except ValueError:
        contained = False
    if not contained:
        raise RuntimeError(
            f"Booster SDK resolved outside K1_SDK_EXPECTED_ROOT: {actual_real}"
        )
    return actual_real


def encode_message(payload: dict) -> bytes:
    if not isinstance(payload, dict):
        raise TypeError("guard message must be an object")
    raw = json.dumps(
        payload, separators=(",", ":"), allow_nan=False
    ).encode("utf-8") + b"\n"
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("guard message exceeds bounded size")
    return raw


def decode_message(raw: bytes) -> dict:
    if not isinstance(raw, bytes) or not raw.endswith(b"\n"):
        raise GuardProtocolError("guard message must be newline terminated")
    if len(raw) > MAX_MESSAGE_BYTES:
        raise GuardProtocolError("guard message exceeds bounded size")
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise GuardProtocolError("invalid guard JSON") from exc
    if not isinstance(value, dict):
        raise GuardProtocolError("guard message must be an object")
    return value


def recv_line(conn: socket.socket) -> bytes:
    data = bytearray()
    while len(data) < MAX_MESSAGE_BYTES:
        chunk = conn.recv(min(1024, MAX_MESSAGE_BYTES - len(data)))
        if not chunk:
            break
        data.extend(chunk)
        if b"\n" in chunk:
            break
    if b"\n" not in data:
        raise GuardProtocolError("incomplete or oversized guard message")
    end = data.index(b"\n") + 1
    if end != len(data):
        raise GuardProtocolError("one request per connection is required")
    return bytes(data)


def peer_credentials(conn: socket.socket) -> tuple[int, int, int]:
    if not hasattr(socket, "SO_PEERCRED"):
        raise GuardProtocolError("SO_PEERCRED unavailable")
    raw = conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12)
    return struct.unpack("3i", raw)


def validate_local_peer(conn: socket.socket, allowed_uids=None) -> None:
    _pid, uid, _gid = peer_credentials(conn)
    allowed = {os.getuid()} if allowed_uids is None else set(allowed_uids)
    if uid not in allowed:
        raise GuardProtocolError("guard peer uid is not authorized")


class GuardClient:
    def __init__(self, socket_path=SOCKET_PATH, timeout=0.2):
        self.socket_path = str(socket_path)
        self.timeout = float(timeout)

    def request(self, payload: dict) -> dict:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
            conn.settimeout(self.timeout)
            conn.connect(self.socket_path)
            conn.sendall(encode_message(payload))
            response = decode_message(recv_line(conn))
        if response.get("ok") is not True:
            raise GuardProtocolError(
                str(response.get("error") or "motion guard rejected request")
            )
        result = response.get("result")
        if not isinstance(result, dict):
            raise GuardProtocolError("motion guard returned malformed result")
        return result

    def status(self) -> dict:
        return self.request({"op": "status"})

    def acquire(self, lease_ms=400) -> dict:
        return self.request({"op": "acquire", "lease_ms": lease_ms})

    def move(self, *, generation, session, seq, vx, vy, vyaw) -> dict:
        return self.request(
            {
                "op": "move",
                "generation": generation,
                "session": session,
                "seq": seq,
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
            }
        )

    def renew(self, *, generation, session, seq) -> dict:
        return self.request(
            {
                "op": "renew",
                "generation": generation,
                "session": session,
                "seq": seq,
            }
        )

    def release(self, *, generation, session, seq) -> dict:
        return self.request(
            {
                "op": "release",
                "generation": generation,
                "session": session,
                "seq": seq,
            }
        )
