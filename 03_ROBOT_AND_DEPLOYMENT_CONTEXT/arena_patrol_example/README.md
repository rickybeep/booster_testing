# Minimal tracked-arena patrol

This is the smallest useful example of how our tracking computer currently
drives a Booster K1 around the arena:

1. Read the K1 UWB tag position.
2. Read the robot IMU yaw and actual mode from the onboard bridge.
3. Calibrate IMU yaw to the arena coordinate frame, or use a previously
   measured heading offset.
4. Walk to a fixed safe waypoint.
5. Send repeated zero-velocity commands at the waypoint.
6. Move the head through a short look-around sequence.
7. Center the head and continue to the next waypoint.

It intentionally excludes human detection, cameras, video streaming, audio,
language-model calls, dashboard code, and encounter behavior.

## Files

- `arena_patrol.py` — UWB input, heading calibration, bounded waypoint
  navigation, HTTP bridge client, head sequence, and offline simulator.
- `arena_config.json` — the measured 18 ft × 18 ft arena, clockwise anchor
  layout, current tag offset, conservative motion limits, safety bands, and
  four example waypoints.
- `test_arena_patrol.py` — offline contract, geometry, state-machine, safety,
  and simulation tests.

No robot address or serial port is stored in the repository.

## Command path

```text
ULM3 UWB receiver ──serial──> tracking PC
                                  │
                                  ├── GET  /api/nav
                                  ├── GET  /api/status
                                  ├── POST /api/move
                                  └── POST /api/head
                                           │
                                           v
                              onboard K1 control bridge
                                           │
                  booster_robotics_sdk_python.B1LocoClient
                         Move(...) / RotateHead(...)
                                           │
                                           v
                                     K1 firmware
```

The Windows tracking computer does not create a raw DDS `LowCmd`. The Booster
Python SDK is initialized on the K1 itself, where loopback is the correct
interface:

```python
import booster_robotics_sdk_python as sdk

sdk.ChannelFactory.Instance().Init(0, "127.0.0.1")
client = sdk.B1LocoClient()
client.Init()

client.Move(vx, vy, vyaw)
client.RotateHead(pitch_rad, yaw_rad)
```

The same client can call
`client.ChangeMode(sdk.RobotMode.kWalking)`, but this patrol intentionally
does not change modes. Mode selection remains an explicit operator action.

The current onboard bridge maps the minimal HTTP contract as follows:

| Tracking-computer request | Onboard SDK action |
|---|---|
| `POST /api/move {"vx": ..., "vy": 0, "vyaw": ...}` | `B1LocoClient.Move(vx, vy, vyaw)` |
| `POST /api/head {"mode": "ease", "pitch": ..., "yaw": ..., ...}` | smoothed calls to `B1LocoClient.RotateHead(pitch, yaw)` |
| `GET /api/nav` | reads advancing SDK low-state telemetry: `yaw`, `gyro_z`, `odom`, `ts`, `rate_hz` |
| `GET /api/status` | reads actual mode plus the result/timestamp of the last robot-side `Move` call |

`POST /api/move` returning `{"rc": 0}` only proves that the onboard bridge
queued the target. The example also requires `/api/status.last_move_ts` to
advance while a nonzero target is active and faults if it does not.

## Coordinate and command conventions

- Arena heading `0°` points along `+X`; `+90°` points along `+Y`.
- Positive logical yaw is counter-clockwise.
- The direct Booster `Move(..., vyaw)` convention is represented by
  `sdk_yaw_sign`. It defaults to `+1`.
- The older V7 application used a clockwise-positive navigation convention
  and therefore had a `-1` conversion at its SDK boundary. That application
  convention is deliberately not carried into this clean example.
- Verify the sign with the robot safely supported before any floor patrol.
  If the observed hardware convention is reversed, use
  `--sdk-yaw-sign -1`. This is an actuation test and requires an agreed
  physical-test procedure.
- Normal stop is `Move(0, 0, 0)`. This example never uses DAMP as a routine
  stop and never requests PREP, CUSTOM, or WALK.
- The robot must already report `walk`. Any other mode produces zero motion.

The controller runs at 20 Hz. It continually sends the current velocity,
including zeros during every waypoint dwell. The onboard bridge independently
zeros stale targets after 0.6 seconds.

## Arena and boundary behavior

The included configuration describes:

- arena: `0.0–5.4864 m` on both axes;
- anchors: clockwise from the origin;
- anchor plane to tag plane: `0.2286 m`;
- UWB tag: `0.2032 m` behind the robot center;
- perimeter stop band: first `0.50 m` from an edge;
- warning/recovery band: `0.50–1.50 m` from an edge;
- safe waypoint interior: at least `1.50 m` from every edge.

Behavior is fail-closed:

- outside the measured arena or in the perimeter stop band: latched zero;
- in the warning band: slow return toward arena center;
- stale/untrusted UWB, stale nav source timestamp, stale status, mode not
  WALK, robot-side SDK error, or missing successful-`Move` progress: latched
  zero after motion has started.

Device-solved `LO=[...]` UWB fixes do not expose anchor health. Locally solved
`mc` fixes must use all four configured anchors and stay below the configured
reprojection residual.

## Heading calibration

Arena heading is:

```text
heading_deg = imu_yaw_sign × degrees(raw_imu_yaw) + heading_offset_deg
```

An existing offset can be supplied with `--heading-offset-deg`. With
`--calibrate-heading`, the controller:

1. requires fresh UWB/nav/status, actual WALK mode, and a start inside the safe
   interior;
2. walks straight at `0.20 m/s`;
3. stops after `0.80 m`;
4. computes the travel bearing from UWB;
5. derives the offset from that bearing and the current raw IMU yaw;
6. holds zero for one second before beginning the patrol.

Calibration is real robot motion. It is never enabled implicitly in live
mode.

## Offline use

Only the Python standard library is needed for simulation and tests:

```powershell
python arena_patrol.py
python -m unittest -v test_arena_patrol.py
```

The simulation includes an unknown IMU offset, the physical tag offset,
calibration, body turning, forward motion, waypoint dwell, and repeated laps.
It does not contact the robot or open a serial port.

## Read-only live observation

Install the two live-only dependencies:

```powershell
python -m pip install requests pyserial
```

Then observe real UWB/nav/status and print the controller state without
issuing any POST:

```powershell
python arena_patrol.py `
  --mode observe `
  --robot-url http://ROBOT_IP:8080 `
  --uwb-port COM_PORT `
  --heading-offset-deg HEADING_OFFSET_DEG `
  --seconds 60
```

`observe` performs serial reads and HTTP GETs only. It does not send a stop,
head command, mode change, or movement command.

## Proposed live execution

This repository addition was not deployed or run on a robot. A future live
run requires a clear arena, spotters, an accessible physical stop, confirmed
UWB geometry/tag identity, a lifted forward/yaw sign check, the robot already
in WALK, and explicit operator approval.

With a previously verified heading offset:

```powershell
python arena_patrol.py `
  --mode execute `
  --robot-url http://ROBOT_IP:8080 `
  --uwb-port COM_PORT `
  --heading-offset-deg HEADING_OFFSET_DEG `
  --confirm-live-motion I_UNDERSTAND_THIS_WILL_MOVE_THE_ROBOT `
  --seconds 30
```

To perform the active straight-line calibration instead, replace
`--heading-offset-deg ...` with `--calibrate-heading`. The exact confirmation
phrase is an accidental-run guard, not a substitute for the physical test
approval and setup above.
