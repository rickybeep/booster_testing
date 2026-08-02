#!/usr/bin/env python3
"""Best-effort bounded zero burst after the guard has exited.

The nonblocking SDK lock is the authority boundary: if a guard still owns it,
this helper exits without creating a second locomotion client.
"""

import re
import time
from pathlib import Path

try:
    import fcntl
except ImportError:  # Imported on Windows for offline tests only.
    fcntl = None

try:
    from .k1_guard_protocol import (
        SDK_LOCK_PATH,
        require_sdk_module_path,
        require_sdk_success,
    )
except ImportError:
    from k1_guard_protocol import (  # type: ignore
        SDK_LOCK_PATH,
        require_sdk_module_path,
        require_sdk_success,
    )


SAFE_ZERO_REJECT_CODES = frozenset((100, 400))
SDK_ERROR_CODE_RE = re.compile(r"(?:code|rc)\s*=\s*(-?\d+)", re.IGNORECASE)
ZERO_ATTEMPTS = 5
ZERO_INTERVAL_S = 0.05


def _safe_zero_exception_code(exc) -> int | None:
    match = SDK_ERROR_CODE_RE.search(str(exc))
    if match is None:
        return None
    code = int(match.group(1))
    return code if code in SAFE_ZERO_REJECT_CODES else None


def deliver_zero_burst(client, *, sleep=time.sleep) -> int:
    """Attempt every zero in the finite burst and validate every result."""
    delivered = 0
    safe_rejected = 0
    errors = []
    for index in range(ZERO_ATTEMPTS):
        try:
            rc = client.Move(0.0, 0.0, 0.0)
        except Exception as exc:  # noqa: BLE001
            code = _safe_zero_exception_code(exc)
            if code is None:
                errors.append(f"attempt {index + 1}: {exc!r}")
            else:
                safe_rejected += 1
        else:
            if rc is None:
                rc = 0
            if type(rc) is int and rc == 0:
                delivered += 1
            elif type(rc) is int and rc in SAFE_ZERO_REJECT_CODES:
                safe_rejected += 1
            else:
                errors.append(f"attempt {index + 1}: unexpected rc={rc!r}")
        if index + 1 < ZERO_ATTEMPTS:
            sleep(ZERO_INTERVAL_S)

    print(
        "zero burst complete: "
        f"attempts={ZERO_ATTEMPTS} delivered={delivered} "
        f"safe_mode_rejections={safe_rejected} errors={len(errors)}"
    )
    for error in errors:
        print(f"zero delivery failed: {error}")
    return 0 if not errors else 1


def main():
    if fcntl is None:
        print("zero delivery failed: fcntl is unavailable")
        return 1
    import booster_robotics_sdk_python as sdk

    require_sdk_module_path(sdk)

    Path(SDK_LOCK_PATH).parent.mkdir(parents=True, exist_ok=True)
    with open(SDK_LOCK_PATH, "a+b") as lock_file:
        try:
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 0
        require_sdk_success(
            sdk.ChannelFactory.Instance().Init(0, "127.0.0.1"),
            "ChannelFactory.Init",
        )
        client = sdk.B1LocoClient()
        require_sdk_success(client.Init(), "B1LocoClient.Init")
        return deliver_zero_burst(client)


if __name__ == "__main__":
    raise SystemExit(main())
