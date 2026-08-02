# Offline Tests

The default suite is intended to run without a robot, UWB receiver, serial
device, ROS graph, SSH connection, or arena network. It uses fakes, synthetic
geometry, documentation addresses, and temporary state.

From the package root:

```powershell
py -3.10 -m pytest -q
```

Record the exact count and output only after all package edits are complete. A
previous pass is not evidence that the current files still pass.

## Recorded offline result

On 2026-07-31, after the PC and robot source contracts were frozen, this
package was checked on Windows with Python 3.10.9, NumPy 1.26.4, pyserial 3.5,
requests 2.32.3, and pytest 9.1.1:

```powershell
$env:PYTHONDONTWRITEBYTECODE = "1"
py -3.10 -m pytest -q -p no:cacheprovider
py -3.10 pc\arena_runner.py --config config\tracker.example.json
```

Observed results:

```text
115 passed
valid: true
enabled_robots: []
live_motion: false
validation note: no serial, network, SDK, or robot command was used
```

No hardware, SSH, arena-network, service, SDK, or live-motion action was part
of this run. Any later file change requires a new result.

## Current coverage

| Test file | Covered behavior |
|---|---|
| `test_bundle.py` | Exact release inventory and SHA-256 bytes, robot manifest equality, local Markdown links, excluded artifacts, and selected secret/address patterns |
| `test_config.py` | Disabled example, malformed/unsafe values, and live-gate address/acknowledgement checks |
| `test_navigation.py` | Tag projection, immediate stop smoothing, circular detour, hold hysteresis, dwell/hold behavior, and non-finite UWB rejection |
| `test_motion_guard.py` | Lease expiry, exact five-zero burst, recognized safe-mode zero rejections, degraded errors, and bounded partial local frames |
| `test_motion_client_contract.py` | Ownership proof, exact sequence/session/velocity checks, mode and heading gates, fault/reconnect behavior, and lease health |
| `test_pc_tracking_safety.py` | Atomic UWB samples/provenance, boundary hold, reversal-through-zero, cleanup on startup/preflight failure, coupled config constraints, explicit serial port, and local-token handoff |
| `test_robot_runtime_safety.py` | Token authorization, HTTP framing, mode freshness/gates, feedback-based head queue/retry/supersession behavior, SDK initialization, runtime health, zero helper, and candidate systemd fail-closed properties |
| `test_status_contract.py` | Required fresh source-derived mode age |

These tests establish only the named software behavior. They do not establish:

- successful installation or service supervision on either K1 variant;
- actual vendor SDK import path, ABI, return behavior, or DDS/RPC routing;
- HTTP server behavior through a real network stack;
- real UWB serial parsing quality, anchor placement, RF conditions, or site
  calibration;
- firmware consumption of a WALK/head command;
- physical stopping distance, balance, collision avoidance, or head position;
- CUSTOM/LowCmd discovery, matching, or 22-joint policy deployment; or
- safety certification.

## Fixture rules

- No default test may enable `--live`, open a real serial device, contact a
  robot address, change mode, or send body/head/joint commands.
- Hardware or live-motion tests, if ever added, must be separately marked and
  skipped by default.
- Use IANA documentation addresses in fixtures; do not copy active robot
  addresses or credentials.
- An exit code, HTTP 200, local SDK return, or LowState sample must never be
  presented as firmware-side or physical acceptance by itself.

Follow [TESTING_CHECKLIST.md](../TESTING_CHECKLIST.md) for evidence levels and
the separately authorized hardware workflow.
