# PC to K1 API and Evidence Contract

The candidate bridge listens on TCP port 8080 by default. It uses bearer
authentication but not TLS. Every remote route requires:

```http
Authorization: Bearer <provisioned-token>
```

Only `GET /api/health` may omit the token from robot loopback. Tokens are
separate local files, not JSON values or command-line arguments. Plaintext
authentication requires an isolated, access-controlled robot network.

## Read-only routes

| Route | Response purpose |
|---|---|
| `GET /api/status` | Mode/age, guard velocity/result, head state, capabilities |
| `GET /api/guard/status` | Guard state without the private active session |
| `GET /api/nav` | Source timestamp, yaw, gyro Z, odometry, source rate |
| `GET /api/telemetry` | Lower-rate IMU, joints, odometry, source timestamp |
| `GET /api/health` | Guard status and nav-source age |

A reachable route proves only that the HTTP/status layer responded. The PC
requires advancing nav source timestamps before heading is fresh.

## Acquire

```http
POST /api/guard/acquire
Content-Type: application/json

{"lease_ms": 400}
```

The bridge admits acquire only with fresh observed WALK. The guard accepts an
integer lease from 300 through 500 ms, makes a strict local
`Move(0, 0, 0)` call, and only then creates ownership.

A valid response has:

- `ready: true` and `state: "ACTIVE"`;
- nonempty `generation` and private `session`;
- `seq: 0`; and
- both `desired_velocity` and `ack_velocity` equal to
  `[0.0, 0.0, 0.0]`.

The PC rejects malformed or incomplete acquisition evidence.

## Move

```http
POST /api/move
Content-Type: application/json

{
  "generation": "<guard-boot-id>",
  "session": "<lease-id>",
  "seq": 1,
  "vx": 0.10,
  "vy": 0.0,
  "vyaw": -0.20
}
```

The guard accepts finite JSON numbers within absolute bounds:

- `|vx| <= 0.4` m/s;
- `|vy| <= 0.2` m/s; and
- `|vyaw| <= 0.8` rad/s.

The partner runner always sends `vy=0`. Its positive/negative navigation turn
convention is converted once by `K1_TURN_CMD_SIGN`.

Nonzero move requires fresh WALK at the bridge. Exact zero remains available
after a mode gate closes so an owner can stop.

A successful bridge reply is structurally:

```json
{
  "rc": 0,
  "guard": {
    "state": "ACTIVE",
    "ready": true,
    "generation": "...",
    "session": "...",
    "seq": 1,
    "desired_velocity": [0.1, 0.0, -0.2],
    "ack_velocity": [0.1, 0.0, -0.2]
  }
}
```

The PC requires top-level integer `rc == 0`, the owned generation/session,
exact sequence, ACTIVE/ready state, and exact desired/acknowledged velocity.
Any ambiguity latches a hard fault.

A changed target causes a local SDK call. An identical target may reuse the
previous SDK acknowledgement while advancing the lease. Therefore a matching
identical-target reply is ownership evidence, not proof of a new downstream
call.

## Renew and release

```http
POST /api/guard/renew
{"generation": "...", "session": "...", "seq": 2}

POST /api/guard/release
{"generation": "...", "session": "...", "seq": 3}
```

Renew requires fresh WALK. Release remains available after mode loss. Both
require the current generation/session and a strictly increasing positive
sequence.

Periodic public status deliberately omits `session` and cannot refresh the
PC's ownership proof. Only a matching private acquire/move/renew response can
do that.

Release and expiry attempt a bounded five-zero burst. A completed response
describes guard processing; it is not proof that all zeros reached firmware or
motors.

## Head

```http
POST /api/head
{"pitch": 0.0, "yaw": 0.45, "speed": 0.5}
```

The only accepted fields are `pitch`, `yaw`, and `speed`. Current limits are
plus/minus 0.60 rad pitch, plus/minus 1.20 rad yaw, and 0.05 through 1.50
rad/s.

Queueing requires authentication, fresh WALK, and fresh non-lost LowState head
feedback. A valid immediate response says:

```json
{
  "rc": 0,
  "accepted": true,
  "queued": true,
  "acceptance": "queued_only",
  "sdk_call_completed": false
}
```

Each later slew step requires a newer head-feedback timestamp. Status separates
`pending`, measured position/source/age, `last_sdk_call_accepted`, `last_rc`,
`last_error`, retry timing, and failure count. Mode loss or invalid feedback
discards the queued target; SDK failures back off and are eventually abandoned.

A newer valid POST supersedes the prior queued target and resets that target's
retry/failure budget. An SDK call already in flight cannot be cancelled, but
its success or failure is command-ID scoped and cannot overwrite, delay, or
abandon the newer command's state.

Queue acceptance is not SDK success. SDK success is not measured position.
LowState position is feedback evidence, not independent visual proof. The
packaged HeadYaw/HeadPitch naming is positional and must not be trusted for
deployment until the target K1's `motor_state_parallel` ordering is verified.

## Mode contract

There is no mode-change, DAMP, PREP, gait, or estop route in this bridge. The
robot must already be in WALK through the separate operator/vendor workflow.
DAMP is deliberately absent because it can make a standing robot limp.

## HTTP result classes

| Status | Meaning |
|---|---|
| `200` | This API layer accepted/returned the documented response |
| `400` | Malformed, unknown, non-finite, out-of-range, or ambiguous framing |
| `401` | Missing/invalid bearer authentication |
| `404` | Route does not exist |
| `409` | Fresh-WALK/head-baseline/guard ownership precondition failed |
| `503` | Local dependency or unexpected server operation failed |

The server rejects Transfer-Encoding and duplicate/comma-separated
Content-Length framing.

## Evidence levels

| Level | Evidence | Supported conclusion |
|---|---|---|
| 1 | Authenticated HTTP response | Intended bridge responded |
| 2 | Matching guard generation/session/sequence and velocity | Exact owner operation was processed |
| 3 | Acquire zero or changed-target acknowledgement plus local SDK result | Guard's local SDK call returned success |
| 4 | Advancing mode/nav/LowState data with no local error | Feedback pipeline is advancing |
| 5 | Correlated firmware-side log/result | Firmware consumer accepted the operation |
| 6 | Independent observation under an approved test | Physical behavior occurred as observed |

Levels 1 through 4 are not firmware or physical proof. LowState reception, a
local SDK return, HTTP 200, or clean program exit must not be called a
successful hardware test by itself.
