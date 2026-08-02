# How the V10 Tracker Works

This package is a high-level WALK controller. It does not stream joint targets,
change robot mode, or run a learned CUSTOM policy.

## 1. Establish a trusted arena position

`pc/track_uwb.py` reads one explicit serial port in live mode. Every accepted
line must contain role metadata so an unidentified packet cannot silently
become tag zero.

Two position sources are supported:

- `mc` packets are solved by 2-D trilateration. Anchor count and reprojection
  residual must meet the configured thresholds before a fix enters the median
  or Kalman filter.
- device-solved `LO` packets contain a position but do not expose contributing
  anchors or residual. They are rejected unless `allow_unverified_lo` is
  explicitly enabled.

Position, timestamp, source, anchor count, residual, and trail are copied under
one lock into an immutable measurement. The runner rejects missing, malformed,
non-finite, future-dated, stale, low-anchor, high-residual, or unknown-source
measurements.

The UWB tag may be offset from the desired robot-center point. The runner
projects the tag using the calibrated heading and configured forward/left
offset. That projected center is the position used for walls, keepouts, peers,
and waypoints.

## 2. Establish fresh heading and WALK mode

On the K1, the telemetry process subscribes to LowState and odometry. The ROS
mode watcher subscribes to `/robot_states` with BEST_EFFORT, KEEP_LAST, depth 1.
Both publish atomic snapshots under the protected
`/run/booster-track-v10` runtime directory.

The bearer-authenticated bridge serves those snapshots to the PC. A first nav
snapshot establishes a timestamp baseline; heading becomes fresh only after a
later source timestamp advances. A frozen file present before connection
therefore cannot satisfy ARM.

Heading conversion is:

```text
arena_heading_deg =
    (imu_yaw_sign * degrees(raw_yaw) + imu_yaw_offset_deg) mod 360
```

Mode age is computed on the robot. The bridge rejects acquire, renewal,
nonzero body movement, and head queueing unless the observed mode is fresh
`walk`. Zero movement and release remain available so an owner can stop. No
route in this package changes mode.

## 3. Build one obstacle snapshot

Every control tick creates an immutable circular-obstacle list for each robot:

- a peer circle uses the sum of both configured robot radii;
- a static keepout uses robot radius plus keepout radius; and
- metadata records source/label for diagnostics.

The same list feeds tangent detours, reactive repulsion, and hard-hold logic.
If any enabled robot lacks trusted localization, the shared peer feed is
unhealthy and all enabled robots hold zero.

Planner input errors, malformed obstacles, and planner exceptions produce a
blocked route rather than a direct fail-open path.

## 4. Turn, walk, dwell, and look

Each robot follows a fixed waypoint loop:

1. `TURN`: command zero forward velocity and slew yaw rate until aligned to the
   first safe steering point.
2. `WALK`: scale the V10 Navigator's forward fraction by the reviewed speed cap
   and clamp yaw rate.
3. `DWELL`: command zero body velocity, keep the lease supervised, and queue
   configured head-scan targets at a lower rate.
4. `HOLD`: command zero for stale/invalid data, boundary violation, blocked
   geometry, hard proximity, mode/guard loss, or unknown navigator state.

The runner checks the safe interior before calling the Navigator. If the robot
center is outside it, the result is HOLD; this package does not command
automatic boundary recovery. Inside the boundary, the V10 Navigator retains
tangent-circle detours, repulsion, near-target alignment, arc steering, and
approach braking.

The command smoother ramps increases, applies reductions/stops immediately,
and forces a zero tick before reversing either axis.

## 5. Acquire and supervise WALK authority

After read-only connection and preflight, explicit ARM asks the robot bridge
for a 400 ms lease. The bridge first requires fresh WALK. The guard then makes
a positively acknowledged `Move(0, 0, 0)` call before it creates the ACTIVE
session.

Every owner operation carries:

- a random guard boot generation;
- a random lease session;
- a strictly increasing integer sequence; and
- finite bounded velocity values.

The PC validates the returned generation/session/sequence and exact
desired/acknowledged velocity. Public periodic status omits the session and
cannot renew ownership proof. An ambiguous move, zero, release, generation
change, or exhausted renewal window latches a PC hard fault; ordinary telemetry
recovery cannot silently re-arm it.

The HTTP process does not call `Move`. It forwards locomotion to a same-UID
Unix-socket guard. The guard is the persistent SDK WALK client and validates
ownership, deadline, sequence, bounds, and acknowledgement. Identical targets
can reuse the previous SDK acknowledgement while advancing the lease; changed
targets invoke the SDK.

## 6. Stop, release, and head semantics

A normal body hold sends one positively acknowledged zero, then renewal keeps
the zero lease supervised without spamming the SDK. Disarm requires an
acknowledged zero before release.

On release or lease expiry, the guard attempts exactly five zeros 50 ms apart
and then clears ownership. If the guard process exits, a lock-gated helper may
attempt another five-zero burst only after it proves the guard no longer owns
the SDK-motion lock.

Head control is intentionally separate from locomotion ownership. Queueing
requires authentication, fresh WALK, and fresh non-lost LowState
`HeadPitch`/`HeadYaw` feedback. Each bounded slew step is based on a newer
feedback sample. The POST response means queue acceptance only; status records
SDK-call acceptance separately from measured feedback.

## 7. What the software can prove

The package can prove its own validation decisions, HTTP authentication,
guard ownership replies, local SDK return handling, and advancing feedback.
It cannot prove from software exit status alone that firmware consumed a
command or that the robot moved/stopped as intended.

The lease is defense in depth, not a safety-rated stop timer. A blocking vendor
SDK call can delay guard expiry handling, the SDK lock is cooperative, and
plaintext HTTP still requires an isolated trusted network.

See [API_CONTRACT.md](API_CONTRACT.md) for exact payload/evidence semantics and
[TESTING_CHECKLIST.md](TESTING_CHECKLIST.md) before proposing robot-connected
work.
