# PC Runtime

This directory is the narrow Windows-side V10 WALK/head package. It imports no
Booster SDK; the vendor SDK remains on the robot.

## Files

| File | Responsibility |
|---|---|
| `arena_runner.py` | Validation-only entrypoint by default; optional fixed-waypoint fleet loop behind explicit live gates |
| `tracker_config.py` | Strict schema-1 loader and geometry/cap validation |
| `booster_config.py` | Sanitized constants required by the navigator and client |
| `booster_nav.py` | Circular detours, repulsion, hard hold, steering, braking, arrival, plus retained navigation helpers |
| `track_uwb.py` | Serial parsing, trilateration, filtering, and atomic position/time/quality/provenance snapshots |
| `booster_motion.py` | Minimal authenticated HTTP feedback, guard lease, WALK velocity, stop/release, and simple-head client |

All imports resolve within this directory or to dependencies listed in
`../requirements.txt`. Site-specific addresses, tags, calibration, and
waypoints belong in ignored `../config/tracker.local.json`.

## Offline setup and validation

From the package root on Windows with Python 3.10:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
.\.venv\Scripts\python -m pytest -q
.\.venv\Scripts\python pc\arena_runner.py --config config\tracker.example.json
```

The final command omits `--live`; it parses and validates the deliberately
disabled example and performs no serial, network, SDK, or robot operation.

## WALK behavior

Each enabled robot follows its configured waypoint list through TURN, WALK,
DWELL, and HOLD behavior. The navigator:

- keeps targets inside the safe arena interior;
- plans tangent detours around circular static/peer obstacles;
- retains reactive repulsion beneath the planned route;
- applies a hard proximity hold with release hysteresis;
- reduces forward speed for heading error and approach braking; and
- commands zero for stale pose, stale mode/heading, unhealthy shared obstacle
  data, blocked geometry, or guard-session loss.

The outer runner holds at zero outside the configured safe interior; it does
not invoke the vendored navigator's recovery motion there. Command smoothing
ramps increases, applies decreases/stops immediately, and passes a requested
direction reversal through a zero tick.

The outer fleet loop holds every enabled robot if any enabled robot's UWB pose
is untrusted. Close peers both stop; this package does not include the broader
dashboard backend's right-of-way/escape arbitration, so a close conflict can
deadlock. See [arena_runner.py](arena_runner.py#L109).

At a waypoint, body velocity remains zero while the runner cycles through the
configured simple head-yaw scan. Head acknowledgement is separate from body
lease evidence and is not physical joint feedback.

## Command ownership

`MotionController.connect()` performs authenticated GET-based discovery and
feedback polling.
`arm()` additionally requires connected state, fresh WALK mode, fresh heading,
and an acquirable robot guard before requesting a lease
([booster_motion.py](booster_motion.py#L714)). The runner never changes mode.

During startup, readiness accepts an acquirable idle guard. After acquisition,
the same gate accepts the client's own healthy `ACTIVE` session instead of
mistaking its lease for a foreign owner.

Body commands carry the guard generation, session, and increasing sequence.
The client requires matching desired and acknowledged velocity before updating
its local command record. A command is hard-faulted locally on ambiguous or
mismatched acknowledgement; lease expiry remains the robot-side fallback.

The PC only queues a head request while it owns a healthy guard session and has
fresh WALK feedback. The server also requires the bearer token and fresh WALK
mode, but the head endpoint does not carry or validate the guard session. Its
POST response proves queue acceptance only. The server establishes each slew
step from fresh, non-lost LowState `HeadPitch`/`HeadYaw` feedback and later
status separates measured position/age from `last_sdk_call_accepted` and error
state. LowState is robot feedback, not independent visual or causal proof.

The bridge uses bearer authentication over plaintext HTTP. It provides no TLS,
so the robot network still needs isolation and access control. Token paths live
in ignored local configuration; token contents must never be committed.

## Live path status

`--live` is an integration path, not a read-only preflight and not an
authorization mechanism. After its software gates pass, it opens UWB and robot
connections and can acquire a lease and send physical commands. Do not invoke
it without a separately reviewed physical-test plan and explicit approval.

There is no PC dashboard, iPad link, person detector, camera, audio, cloud
upload, mode-change, DAMP, gait, CUSTOM-policy, or deployment client in this
subtree.
