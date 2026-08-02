#!/usr/bin/env python3
"""Read-only health report for V10's independently supervised processes."""

from __future__ import annotations

import argparse
import json
import urllib.error
import urllib.request

try:
    from .k1_guard_protocol import GuardClient
except ImportError:
    from k1_guard_protocol import GuardClient  # type: ignore


def check_guard(client=None):
    client = client or GuardClient()
    try:
        status = client.status()
    except Exception as exc:
        return {
            "component": "motion_guard",
            "healthy": False,
            "guard_ready": False,
            "error": str(exc) or type(exc).__name__,
        }
    healthy = bool(
        status.get("ready")
        and status.get("state")
        in ("READY_IDLE", "ACTIVE", "EXPIRED", "RELEASED")
    )
    return {
        "component": "motion_guard",
        "healthy": healthy,
        "guard_ready": healthy,
        "status": status,
    }


def check_server(url="http://127.0.0.1:8080/api/health", opener=None):
    opener = opener or urllib.request.urlopen
    try:
        with opener(url, timeout=0.25) as response:
            payload = json.load(response)
        healthy = isinstance(payload, dict)
        return {
            "component": "control_server",
            "healthy": healthy,
            "status": payload if healthy else {},
        }
    except (OSError, ValueError, urllib.error.URLError) as exc:
        return {
            "component": "control_server",
            "healthy": False,
            "error": str(exc) or type(exc).__name__,
        }


def health_report():
    guard = check_guard()
    server = check_server()
    return {
        "healthy": guard["healthy"] and server["healthy"],
        # This is process readiness only. Motion authorization also requires
        # authenticated PC preflight, fresh WALK mode, and trusted navigation.
        "guard_ready": guard["guard_ready"],
        "motion_guard": guard,
        "control_server": server,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--require-ready", action="store_true")
    parser.add_argument("--require-guard", action="store_true")
    args = parser.parse_args()
    report = health_report()
    print(json.dumps(report, sort_keys=True))
    if args.require_guard:
        return 0 if report["motion_guard"]["healthy"] else 1
    return 0 if (report["healthy"] or not args.require_ready) else 1


if __name__ == "__main__":
    raise SystemExit(main())
