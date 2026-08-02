# V10 Motion-Guard Contract

> This documents the packaged implementation for offline review. It is not an
> installation procedure, safety certification, or authorization to command a
> robot.

## Responsibility boundary

The package has three distinct enforcement layers:

1. The PC checks configuration, UWB trust, arena/keepout geometry, peer data,
   fresh heading, fresh WALK mode, operator launch gates, and navigator state.
2. The HTTP bridge authenticates callers and requires fresh observed WALK mode
   for acquire, renew, nonzero move, and head requests. It still permits zero
   and release so an existing owner can stop after a mode transition.
3. The robot guard checks only locomotion ownership and transport invariants:
   generation, session, increasing sequence, finite bounded velocity, and lease
   freshness.

The guard never sees UWB, waypoints, walls, keepout labels, peer identities, or
the runner's HOLD reason. It cannot certify that a geometrically valid command
is physically safe. The PC-side readiness and patrol gates are visible in
[pc/arena_runner.py](pc/arena_runner.py#L232) and
[pc/arena_runner.py](pc/arena_runner.py#L327).

```text
validated PC intent
        |
        v
HTTP generation/session/sequence
        |
        v
authenticated, mode-gated robot bridge
        |
        v
same-UID Unix-socket guard
        |
        v
B1LocoClient.Move
```

## Guard states

| State | Lease owner | Nonzero accepted | Meaning |
|---|---|---|---|
| `STARTING` | No | No | Process created; startup zero not yet completed |
| `READY_IDLE` | No | No | Startup completed and a lease may be acquired |
| `ACTIVE` | Yes | Yes, for the exact owner | Lease is active |
| `EXPIRED` | No | No | Lease timed out and stop burst completed |
| `RELEASED` | No | No | Owner released and stop burst completed |
| `DEGRADED` | No reliable authority | No | Unexpected SDK result/error prevented a ready claim |

There are no robot-guard states named `DISARMED`, `HOLD`, or `LATCHED`. HOLD is
a PC navigation outcome; a PC hard fault is separate local state. Guard state
initialization and publication are in
[robot/runtime/k1_motion_guard.py](robot/runtime/k1_motion_guard.py#L88).

## Startup

The guard obtains a nonblocking cooperative SDK-motion file lock, initializes
the vendor client on loopback, and attempts `Move(0, 0, 0)`. Only then can it
enter `READY_IDLE` ([k1_motion_guard.py](robot/runtime/k1_motion_guard.py#L167)).

The candidate assumes return/exception codes 100 and 400 identify a zero
rejected by a non-walking mode. It tolerates those codes only on a zero and
records the outcome as stopped-but-not-delivered. Unknown return codes or
exceptions enter `DEGRADED`; they must not be relabeled as success. The exact
meaning and exclusivity of 100/400 must be verified against the installed K1
SDK/firmware before deployment.

Acknowledged velocity begins as unknown. A startup zero rejected by a
non-walking mode does not manufacture an acknowledged `[0, 0, 0]` value.

Startup is therefore command-producing even though the requested velocity is
zero. Starting the service is not a read-only diagnostic.

## Acquire, move, and renew

- Lease duration must be an integer from 300 to 500 ms; the PC requests 400 ms.
- Acquire is allowed only from `READY_IDLE`, `EXPIRED`, or `RELEASED`. It first
  requires a strict SDK-acknowledged zero, then creates a new random session at
  sequence zero. A safe-mode zero rejection is not tolerated at this point
  because the HTTP bridge already required fresh WALK.
- Every owner operation must carry the current boot generation and session.
- Sequence must be a positive integer and strictly increase.
- Move values must be finite JSON numbers within absolute limits of 0.4 m/s
  forward, 0.2 m/s lateral, and 0.8 rad/s yaw.
- A changed target invokes the SDK. An identical target is deduplicated only
  when both desired and acknowledged velocity already equal that target; it
  still advances sequence and lease.
- Renew advances sequence and the deadline without changing velocity.

The robot implementation is
[k1_motion_guard.py](robot/runtime/k1_motion_guard.py#L178). The PC requires an
exact session, sequence, desired velocity, and acknowledged velocity before
accepting movement locally
([pc/booster_motion.py](pc/booster_motion.py#L834)). Periodic public status
omits the session and cannot refresh ownership proof.

The PC normally attempts renewal every 100 ms and retries within the remaining
lease window. Loss of fresh WALK feedback, transport ambiguity, mismatched
acknowledgement, or an exhausted lease window creates a local hard fault and
removes PC motion authority; see
[pc/booster_motion.py](pc/booster_motion.py#L1061).

## Stop, release, and expiry

A normal PC stop sends an explicit zero and requires matching acknowledgement.
Disarm releases the lease only after that zero is acknowledged. If the zero or
release is ambiguous, the PC reports failure and the robot-side lease expiry is
the remaining bounded-intent fallback.

On explicit release or lease expiry, the guard attempts exactly five zeros,
50 ms apart, then clears session ownership and enters `RELEASED` or `EXPIRED`
([k1_motion_guard.py](robot/runtime/k1_motion_guard.py#L256)). It does not send
zeros forever after reaching a terminal state.

The systemd guard unit also invokes `k1_motion_zero_once.py` after the guard
exits. That helper creates a client only if it can obtain the same motion lock
and attempts a checked five-zero burst. Thus the guard is the sole persistent
and sole nonzero `Move` owner, not literally the only source file that can call
`Move`.

## Status and evidence

Guard status includes state, readiness, generation, lease age/remaining time,
sequence, desired velocity, acknowledged velocity, last local SDK result,
error, last stop reason, stop-burst count, zero outcome, and whether the active
target is nonzero
([k1_motion_guard.py](robot/runtime/k1_motion_guard.py#L317)).

This status supports several precise conclusions:

- exact session/sequence and matching velocities prove an owner response;
- a changed target with matching acknowledgement indicates that the local SDK
  call returned through the guard;
- an unchanged target may reuse an earlier local SDK acknowledgement only when
  desired and acknowledged velocity both match; and
- neither case proves firmware consumption or physical motion.

Firmware-side correlated evidence and independent observation remain required
for an approved physical test.

## Known limitations

- A blocking vendor `Move` call can block the Python guard loop and delay lease
  evaluation; the lease duration is not a guaranteed physical stop bound.
- The SDK-motion file lock is cooperative. An unrelated process can ignore it.
- HTTP uses bearer authentication but no TLS. Lease ownership is not network
  secrecy, and the head endpoint is outside the guard-session protocol.
- The bearer token does not protect against an observer able to read plaintext
  robot-network traffic.
- Status-file persistence and Python scheduling can delay guard progress.
- Five attempted zeros are bounded software intent, not proof that firmware or
  motors accepted them.

Any proposed physical validation must isolate one variable and follow
[TESTING_CHECKLIST.md](TESTING_CHECKLIST.md).
