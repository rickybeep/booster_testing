# Booster K1 V10 Track System - Partner Package

> Status: isolated offline-review candidate. This folder has not been
> installed on a robot and has not passed an approved physical-motion test.

This is the narrow, shareable part of our V10 arena tracker. It shows how the
Windows arena PC reads UWB and robot feedback, follows fixed waypoints, stays
inside a configured safe interior, avoids circular keepouts and other tracked
robots, stops at each point, and requests a simple head scan while stopped.

It intentionally excludes the iPad workflow, person detection, camera and
streaming code, audio, cloud uploads, dashboards, private site configuration,
credentials, logs, captures, deployment utilities, and historical tracker
trees.

## Command path

The PC does not import the Booster SDK. The K1 runs the vendor SDK beside its
firmware services:

```text
UWB + K1 heading/mode
          |
          v
PC fixed-waypoint runner and V10 Navigator
          |
          v
Bearer-authenticated HTTP (high-level velocity only)
          |
          v
K1 Unix-socket lease guard
          |
          v
B1LocoClient.Move(vx, 0, vyaw) -> stock WALK controller
```

The guard is the sole persistent and sole nonzero `Move` owner in this package.
A lock-gated shutdown helper can attempt only a bounded zero burst after the
guard exits. The runner never changes mode and never enters PREP, DAMP, or
CUSTOM.

This WALK controller is separate from learned-policy deployment. It does not
publish `LowCmd`/`MotorCmd`, define the 22-joint CUSTOM contract, or load a
checkpoint.

## Included

- Strict schema-1 configuration with disabled Education/Jetson and
  Geek/Qualcomm examples.
- UWB role parsing, MC trilateration, atomic position/quality snapshots,
  pre-filter quality rejection, and explicit opt-in for unverified LO fixes.
- Heading and tag-to-center calibration.
- Fixed TURN -> WALK -> DWELL/head-scan behavior.
- V10 circular detour/repulsion logic, braking, peer hard-hold hysteresis, and
  fail-closed planner behavior.
- A runner-level HOLD whenever the robot center is outside the safe interior;
  this package does not command automatic boundary recovery.
- Bearer-authenticated PC/robot transport, fresh-WALK gates, strict command
  acknowledgement, a 400 ms lease, and bounded zero bursts.
- Minimal robot feedback/head bridge, protected runtime snapshots, ROS mode
  watcher, telemetry subscriber, candidate systemd units, and an exact runtime
  allowlist.
- Offline tests for configuration, geometry, navigation, UWB, transport,
  authentication, mode/head gates, lease behavior, cleanup, and package
  hygiene.

The human overview is [HOW_THE_TRACKER_WORKS.md](HOW_THE_TRACKER_WORKS.md).
The exact intended inventory is [PACKAGE_MANIFEST.md](PACKAGE_MANIFEST.md).

## Offline quick start

From this folder on Windows with Python 3.10:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
$env:PYTHONDONTWRITEBYTECODE = "1"
.\.venv\Scripts\python -m pytest -q -p no:cacheprovider
.\.venv\Scripts\python pc\arena_runner.py --config config\tracker.example.json
```

The final command is validation-only. It does not open a serial port, network
connection, SDK channel, or robot command path. The example uses IANA
documentation addresses, `uwb.port: "AUTO"`, `allow_unverified_lo: false`,
disabled robots, and `live_motion_enabled: false`.

Installing dependencies may contact a Python package index. The package pins
direct dependencies but does not contain a transitive lock, wheelhouse, vendor
SDK, ROS installation, or robot firmware.

## Live path is present but not authorized

The code's live path requires all of the following before it can start:

1. A separate ignored local configuration with `live_motion_enabled: true`.
2. At least one enabled robot and an explicit UWB serial port; `AUTO` is
   rejected.
3. Site-local RFC1918 IPv4 robot addresses.
4. A separate local bearer-token file for every enabled robot.
5. The `--live` flag and exact acknowledgement phrase enforced by the runner.
6. Trusted, non-future UWB data, fresh advancing heading data, fresh WALK mode,
   and a ready/acquirable guard.

Those are software gates, not permission. Any robot installation, service
start/stop, POST request, guard acquisition, head command, or body motion needs
a separate reviewed test plan and explicit user approval. Starting or stopping
the guard is itself command-producing because it attempts zero velocity.

## Evidence boundary

An HTTP response proves that an HTTP layer responded. A matching guard
generation/session/sequence and velocity proves the local guard processed that
operation; for a changed target it also shows the local SDK call returned
success. Advancing LowState/mode snapshots prove feedback is flowing.

None of those alone proves firmware acceptance or the intended physical
behavior. A clean process exit is not a hardware success criterion. See
[API_CONTRACT.md](API_CONTRACT.md) and
[TESTING_CHECKLIST.md](TESTING_CHECKLIST.md).

## Current limitations

- Neither K1 variant has been installed or physically tested from this copy.
- The exact robot interpreter, K1-aware SDK import root, ROS setup, DDS
  environment, and SDK return conventions must be verified per robot.
- HTTP bearer authentication has no TLS; the robot network must be isolated
  and access controlled.
- The Python vendor `Move` call has no in-process timeout. If it blocks, lease
  evaluation and zero attempts can be delayed.
- The SDK lock is cooperative; unrelated software can ignore it.
- Device-solved LO positions expose no anchor-count/residual evidence and are
  disabled by default.
- Symmetric peer hard holds can deadlock close robots. This reduced runner has
  no dashboard right-of-way/escape arbitration.
- Software geometry and attempted zeros are not safety-rated physical
  boundaries or stopping guarantees.

## Documents

- [BASELINE_WALK_AND_HEAD.md](BASELINE_WALK_AND_HEAD.md): scope, provenance,
  reported environment, and evidence status.
- [V10_SOURCE_NOTES.md](V10_SOURCE_NOTES.md): source snapshot and package-only
  adaptations.
- [SDK_AND_RUNTIME.md](SDK_AND_RUNTIME.md): PC/robot prerequisites and exact
  SDK-selection requirement.
- [API_CONTRACT.md](API_CONTRACT.md): authentication, payloads, guard replies,
  head semantics, and evidence levels.
- [CALIBRATION.md](CALIBRATION.md): arena, UWB, heading, offsets, waypoints, and
  limits.
- [OPERATIONS.md](OPERATIONS.md): approved offline preparation and proposed
  live-review boundary.
- [MOTION_GUARD_INTEGRATION.md](MOTION_GUARD_INTEGRATION.md): lease/zero
  contract and limitations.
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md): evidence-first fault isolation.
- [TESTING_CHECKLIST.md](TESTING_CHECKLIST.md): offline, read-only, and
  separately approved physical evidence gates.

The development source tree was not modified while preparing this isolated
package. Nothing in `05_TRACK_SYSTEM` has been staged, committed, pushed,
deployed, or copied to a robot.
