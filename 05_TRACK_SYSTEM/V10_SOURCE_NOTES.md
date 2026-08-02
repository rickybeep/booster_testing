# V10 Source Notes and Local Adaptations

This folder was assembled by explicit allowlist from the active
`TRACK_CONTROL/BOOSTER_TRACK_V10` candidate and related V10 runtime files. It
is an isolated partner package, not a raw mirror. The development source tree
was not edited while this package was prepared.

## Lineage retained

- [pc/booster_nav.py](pc/booster_nav.py) contains the V10 geometry,
  turn/facing, circular detour, repulsion, braking, arrival, and peer-hold
  logic.
- [pc/track_uwb.py](pc/track_uwb.py) contains the V10 UWB parsing,
  trilateration, filtering, timestamp, provenance, and quality logic.
- [robot/runtime/k1_guard_protocol.py](robot/runtime/k1_guard_protocol.py)
  contains the bounded newline-framed Unix-socket protocol and same-UID peer
  check.
- [robot/runtime/k1_telemetry_daemon.py](robot/runtime/k1_telemetry_daemon.py)
  subscribes to SDK feedback and writes compact navigation/telemetry
  snapshots.
- [robot/runtime/k1_mode_watcher.py](robot/runtime/k1_mode_watcher.py) mirrors
  the ROS 2 robot mode into a compact snapshot.

## Package-specific reductions and hardening

- [pc/booster_config.py](pc/booster_config.py) contains only sanitized
  constants. Real hosts, tags, anchors, offsets, limits, and waypoints are
  absent.
- [pc/tracker_config.py](pc/tracker_config.py) provides a strict schema,
  rejects unknown or unsafe values, validates coupled stopping/clearance
  relationships, and reads bearer tokens from separate local regular files.
- [pc/arena_runner.py](pc/arena_runner.py) replaces the V10 dashboard and
  person/audio workflows with a fixed-waypoint TURN -> WALK -> DWELL loop.
  It never changes robot mode.
- [pc/booster_motion.py](pc/booster_motion.py) is a minimal authenticated
  client, not the full V10 panel client. It exposes only connection/status,
  lease ownership, WALK velocity, stop/release, and stopped head targets.
- [robot/runtime/k1_control_server.py](robot/runtime/k1_control_server.py) is a
  minimal bearer-authenticated bridge. It has no mode, DAMP, PREP, gait,
  camera, audio, hand, upload, or deployment routes.
- [robot/runtime/k1_motion_guard.py](robot/runtime/k1_motion_guard.py) requires
  a positively acknowledged zero before acquisition, exact
  generation/session/sequence ownership, a 300-500 ms lease, and a five-zero
  terminal burst. An SDK failure revokes the active lease immediately.
- [robot/runtime/k1_motion_zero_once.py](robot/runtime/k1_motion_zero_once.py)
  is a cooperative lock-gated shutdown helper that attempts and validates a
  finite five-zero burst only after the guard no longer owns the SDK lock.
- Runtime state moved to `/run/booster-track-v10`; the service templates
  require an external `runtime.env` that selects the exact interpreter, SDK
  root, and ROS setup scripts.
- Legacy launch/deploy tools, private configuration, dashboards, media, logs,
  and unrelated subsystems were not copied.

## Packaged navigation behavior

- The runner projects the configured UWB tag position to the robot center
  using fresh K1 heading feedback.
- Every enabled robot holds if any enabled robot has untrusted localization.
- Before asking the Navigator for motion, the runner sends HOLD whenever the
  center point is outside the configured safe interior. It does not command an
  automatic boundary recovery from outside that interior.
- Circular static keepouts and tracked peers are supplied to the planner using
  summed body/keepout radii. Planning errors and missing safe steering points
  fail closed.
- A hard peer/static hold ring with hysteresis sits outside the normal detour
  and repulsion layers.
- Near a waypoint the robot slows, stops, dwells, and queues a simple head scan
  only while the owned WALK session and feedback gates remain healthy.
- Close peers can symmetrically hold one another. This reduced runner does not
  include the dashboard's right-of-way/escape arbitration, so an operator may
  need to resolve a deadlock.

## Scope boundary

This package sends only high-level `B1LocoClient.Move(vx, 0, vyaw)` and
`RotateHead(pitch, yaw)` requests through the on-robot SDK. It does not publish
`LowCmd`/`MotorCmd`, enter CUSTOM, select a gait, load a checkpoint, or
implement the reported 22-joint low-level policy contract.

## Confirmed package facts

- All remote HTTP routes require an exact bearer token. Only loopback
  `GET /api/health` may omit it.
- Acquire, renew, nonzero move, and head requests require fresh observed WALK
  mode. Zero move and release remain available to the current owner when the
  mode gate closes.
- A changed move target is accepted by the PC only after the guard response
  matches generation, private session, strictly increasing sequence, desired
  velocity, and acknowledged velocity.
- Head responses distinguish queueing and local SDK-call acceptance from
  measured telemetry position.
- The example configuration is disabled and validation-only.

These statements are enforced by the local source and offline tests. They do
not prove installation, SDK/firmware compatibility, command consumption, or
physical behavior on either K1.

## Remaining unknowns and limitations

1. The vendor Python `Move` call has no in-process timeout. If it blocks, lease
   evaluation and subsequent zero attempts can be delayed.
2. The SDK lock is cooperative; unrelated software can ignore it.
3. HTTP bearer traffic has no TLS and must remain on an isolated,
   access-controlled robot network.
4. Education/Jetson and Geek/Qualcomm compatibility has not been verified from
   this package.
5. ROS/SDK mode-number equivalence, K1 `motor_state_parallel` joint ordering,
   and the installed meaning of SDK codes 100/400 remain unverified.
6. Linux `systemd-analyze verify`, exact on-robot interpreter/SDK resolution,
   DDS/RPC routing, firmware acceptance, stopping distance, balance, and
   collision behavior remain untested.

See [PACKAGE_MANIFEST.md](PACKAGE_MANIFEST.md) for the exact handoff inventory
and [TESTING_CHECKLIST.md](TESTING_CHECKLIST.md) for the evidence gates.
