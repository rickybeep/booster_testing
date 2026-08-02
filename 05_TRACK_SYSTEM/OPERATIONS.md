# Preparation and Operations Review

> Only the offline workflow below is approved by this package. Robot
> installation, service changes, POST requests, mode changes, head movement,
> and body movement require a separate plan and explicit authorization.

## Control ownership

This package sends high-level velocity to the stock WALK controller through
`B1LocoClient.Move`. A learned policy in CUSTOM is a separate low-level joint
command path. Never run the WALK tracker and a CUSTOM policy as simultaneous
motion owners.

The runner never enters PREP, WALK, or CUSTOM and never calls DAMP. A physical
operator/vendor workflow must place the robot in WALK before the candidate can
arm. DAMP is not a normal software stop; it can make a standing K1 limp.

## Approved offline preparation

From `05_TRACK_SYSTEM` on Windows:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
.\.venv\Scripts\python -m pytest -q
.\.venv\Scripts\python pc\arena_runner.py --config config\tracker.example.json
Copy-Item config\tracker.example.json config\tracker.local.json
.\.venv\Scripts\python pc\arena_runner.py --config config\tracker.local.json
```

Before validating the local copy:

1. Keep `live_motion_enabled` false and every robot disabled.
2. Enter measured arena bounds, safe margin, anchors, UWB height correction,
   tag IDs, heading calibration, tag-to-center offsets, keepouts, and waypoints.
3. Enter conservative speed/turn/head values without assuming the example is
   safe for the site.
4. Review [CALIBRATION.md](CALIBRATION.md) and resolve every validation error.
5. Record the exact Python version, dependency installation result, test
   output, and configuration-file hash.

Both runner commands above omit `--live`; they perform parsing/validation only
([pc/arena_runner.py](pc/arena_runner.py#L468)).

## Separately authorized read-only inspection

Read-only inspection may establish environment compatibility but cannot prove a
command path. If the robot owner authorizes it, collect without POST requests,
service changes, deployment, or mode changes:

- firmware and hardware variant;
- exact service Python executable and imported SDK `__file__`;
- ROS Humble and BoosterRos2Interface paths;
- selected `K1_PYTHON`, `PYTHONPATH`, `K1_SDK_EXPECTED_ROOT`, `ROS_SETUP`, and
  `BOOSTER_ROS_SETUP` values without recording credentials;
- existing service/process and TCP-port ownership;
- repeated authenticated `GET /api/status`, `GET /api/guard/status`,
  `GET /api/nav`, and `GET /api/health` samples, where those routes are already
  deployed; and
- hashes of already installed files.

Confirm that source timestamps advance. A reachable port or HTTP 200 is not
freshness, SDK acceptance, or physical evidence.

All remote routes require a bearer token. `/api/health` alone may be read
without one from robot loopback. Handle the token through the approved local
secret process and never print or record it. Authentication is over plaintext
HTTP and therefore does not replace network isolation.

The packaged runner has no GET-only hardware-preflight flag. Invoking `--live`
does not stop after inspection; it can continue to lease acquisition and
physical commands. Do not use it as a read-only diagnostic.

## Candidate live sequence for review only

The current implementation would, if separately authorized:

1. open the configured UWB serial port;
2. connect each minimal PC client and poll status, guard, and nav feedback;
3. wait up to ten seconds for trusted UWB, fresh heading, fresh WALK mode, and
   an acquirable guard;
4. acquire one guard lease per robot;
5. run the deadline-adjusted waypoint loop;
6. send guarded zero during holds/dwell and authenticated simple head targets
   during dwell;
7. on exit, attempt zero, release each lease, stop polling, and close clients.

This sequence is documented from
[pc/arena_runner.py](pc/arena_runner.py#L358) so reviewers can analyze it. It is
not a command to perform it. The software acknowledgement phrase and local
enable flag do not substitute for human authorization or physical controls.

## Shutdown semantics

Normal runner teardown first attempts explicit zero for every robot. It then
disarms, which requires an acknowledged zero before requesting guard release,
and shuts down the PC workers. If a stop/release response is ambiguous, the PC
reports a hard fault and the on-robot lease should expire and attempt its
five-zero burst.

The word "should" is intentional: a blocked vendor SDK call can delay the guard
itself. Neither a PC exit nor an attempted zero is a guaranteed physical stop.
An approved live test therefore needs trained operators, clear abort criteria,
spotters, and immediate physical stop authority independent of this software.

Starting or stopping the candidate robot guard is also command-producing: it
attempts zero at startup and through its stop paths. Service management is not
part of read-only inspection.

## Before any proposed physical test

Stop after offline/read-only evidence and prepare a separate test record with:

- one named robot and hardware variant;
- exact package, robot-runtime, and configuration hashes;
- firmware, active SDK path, and service topology;
- one isolated variable;
- initial WALK state and zero commanded velocity;
- maximum speed, turn rate, duration, and travel envelope;
- arena clearance, keepouts, operator, spotters, and stop authority;
- expected software, firmware-side, and physical observations; and
- explicit abort/recovery conditions.

No live procedure is approved until that record is reviewed and the user gives
explicit authorization.
