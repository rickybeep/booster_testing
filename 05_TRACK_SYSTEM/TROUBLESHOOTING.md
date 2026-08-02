# Troubleshooting Guide

> Begin with offline or read-only evidence. Do not "try a small command" to
> diagnose reachability. GET requests require authorization on the packaged
> server except `/api/health` from robot loopback; no POST request is read-only.

## Offline configuration is rejected

Run the validator without `--live` and address the first exact error:

```powershell
py -3.10 pc\arena_runner.py --config config\tracker.local.json
```

The loader rejects missing/unknown keys, non-finite values, unsupported variant
labels, duplicate names/tags, malformed addresses, unsafe coupled freshness or
radius settings, invalid arena geometry, and waypoints outside the safe
interior. Do not weaken validation to make a site file pass. The shareable
example is expected to validate while remaining disabled.

## Validation unexpectedly contacts hardware

The packaged runner opens serial/network resources only after `--live` and all
live gates. Stop and verify that the invoked file is this package's
`pc/arena_runner.py`, `--live` is absent, and imports do not resolve from an
older tracker directory. A default validation run prints
`"live_motion": false`.

## Live gate rejects a host or token

The live path accepts only literal site-local RFC1918 IPv4 addresses; loopback,
link-local, multicast, unspecified, public, hostname, and documentation-range
values are rejected. Each enabled robot also needs a non-symlink regular token
file no larger than 4096 bytes, containing exactly one ASCII token matching
`[A-Za-z0-9._~+/=-]{32,512}` with only an optional final line ending. Keep
tokens out of JSON and Git. Passing this gate does not authorize a live run.

## UWB has no trusted pose

Inspect serial data without enabling motion. Check the explicit port, baud,
tag mapping, source timestamp, anchor wire order, height correction, and finite
coordinates.

- `mc` fixes must meet configured contributing-anchor and residual thresholds.
- Device-solved `LO` fixes do not expose those quality fields; they remain
  rejected unless `allow_unverified_lo` is explicitly true.
- A future-dated, stale, non-finite, malformed, or unknown-provenance fix is
  rejected.

Do not claim RF quality merely because a coordinate is present.

## Robot rotates the wrong way

Keep motion disabled. Recheck the arena axes, raw-yaw convention,
`imu_yaw_sign`, `imu_yaw_offset_deg`, wrapping, and the separate
PC-to-Booster turn-command sign using recorded/synthetic data. Do not fix an IMU
frame error by trial-and-error powered turning.

## Robot position orbits while the body is reoriented

Confirm the UWB tag-to-robot assignment and `tag_offset_m` convention. The
configured vector runs from tag point to desired robot center in robot-forward
and robot-left axes. A wrong sign, frame, or tag ID can create a circular
position error during stationary reorientation.

## Navigator remains in HOLD

Read the reported reason. Expected fail-closed causes include stale/invalid own
pose, stale heading/mode, unhealthy shared obstacle data, outside-safe-interior
position, hard-hold proximity, blocked target, no steering carrot, or unhealthy
guard session. Correct the underlying evidence; do not bypass the hold.

One untrusted enabled robot makes the shared obstacle feed unhealthy and holds
the whole fleet. This preserves unknown peers as hazards rather than silently
removing them.

## Robots stop near each other and never proceed

Both robots can enter the symmetric hard-hold ring. The packaged fixed-waypoint
runner does not include V10 dashboard right-of-way or divergent-escape
arbitration, so this deadlock is a known limitation. Do not shrink or disable
peer bubbles as an ad hoc recovery. Separate starts/paths or add a reviewed
arbitration design before proposing live use.

## HTTP is unreachable or returns 401

For separately authorized read-only inspection, confirm the intended robot
address, network/interface, TCP port ownership, service status, and installed
file hashes. A remote 401 means the bearer header is missing or does not exactly
match the server token. Do not print either token while comparing provisioning.

Port 8080 may already belong to another service; this candidate includes no
installer or port-conflict migration. Do not stop/restart services during a
read-only audit. This folder's candidate has not been deployed merely because
some other service answers on that port.

If a candidate service fails before binding, inspect the configured
`/etc/booster-track-v10/runtime.env` paths without executing or changing them.
The units deliberately fail when `K1_PYTHON`, the SDK expected root, or the ROS
setup scripts are missing or wrong. SDK processes also reject imports outside
`K1_SDK_EXPECTED_ROOT` and ambiguous initialization results. Do not work around
this by falling back to whichever global Python package imports first.

## GET works, but the command path is still unknown

That is expected. GET reachability proves an HTTP/status path only. Advancing
nav/mode timestamps prove their read paths. Neither proves that
`B1LocoClient.Move`, `RotateHead`, vendor transport, firmware, or motors accept
a command. The package has no non-actuating proof of the high-level actuator
consumer and no raw `rt/joint_ctrl` matched-subscription diagnostic.

## Guard cannot be acquired

Without issuing POST, inspect authenticated `/api/status` and
`/api/guard/status`:

- `STARTING`: startup zero has not completed;
- `READY_IDLE`, `EXPIRED`, or `RELEASED`: acquirable in principle;
- `ACTIVE`: another session owns the lease until valid release or expiry; and
- `DEGRADED` or `UNREACHABLE`: stop and investigate the recorded error.

Acquisition additionally requires fresh observed WALK mode at the HTTP bridge
and PC, then performs a strict SDK zero before it creates ownership. The runner
never changes mode. Do not acquire or release a lease merely to diagnose state;
both are command-path operations, and expiry/release sends a zero burst.

## Guard acknowledgement is ambiguous

Treat any generation, session, sequence, desired-velocity, or
acknowledged-velocity mismatch as a hard fault. A matching response proves the
guard handled that owner operation. For a changed target it also means the
guard's local SDK call returned successfully; for an identical target the guard
may reuse the previous acknowledgement without a new SDK call only when desired
and acknowledged values already match. Neither case is firmware-side
acceptance.

An expired 300-500 ms lease causes five zero attempts 50 ms apart. Those values
bound intended software behavior, not physical stopping time, because a vendor
call can block or downstream delivery can fail.

## Guard enters DEGRADED

Do not reacquire until the cause is understood and the process has been
reviewed. Unexpected SDK returns/exceptions revoke useful ownership; a failed
nonzero move triggers an immediate finite stop burst. The candidate tolerates
codes 100 and 400 only on zero and records them as
stopped-but-not-delivered, not as delivered. Confirm from the exact installed
SDK/firmware that both codes exclusively mean a safe non-walking zero rejection;
until then, that interpretation is an unknown rather than a proven fact.

## Stop or release result is uncertain

Keep the verdict `unknown`. The PC only releases after a matching explicit-zero
acknowledgement. If zero or release is ambiguous, the lease-expiry stop burst is
the remaining software fallback. Verify terminal guard state, stop-burst count,
zero outcome, firmware logs, and independent physical state in any later
approved test; a clean PC exit is insufficient.

## Mode is stale or not WALK

Check whether `/run/booster-track-v10/mode.json` is produced by the ROS mode
watcher with an advancing timestamp, then inspect ROS and
BoosterRos2Interface read-only. The
bridge rejects acquire/renew and nonzero body/head commands without fresh WALK.
Zero and release remain allowed so an owner can stop. Never switch modes as a
troubleshooting shortcut; entering DAMP can make a standing robot limp.

Also verify that the ROS `current_mode` numeric value for WALK equals the
installed SDK `RobotMode.kWalking` value. Fresh timestamps do not prove that
cross-interface enum mapping.

## Head POST was accepted but no head result is known

The POST response means the bounded target was queued. It deliberately reports
`acceptance: "queued_only"` and `sdk_call_completed: false`. Queueing and each
slew step require LowState no older than 0.75 s with both unique, finite,
non-lost `HeadPitch`/`HeadYaw` values inside the command envelopes plus 0.05 rad.
Later
authenticated status reports `pending`, measured position/source/age,
`last_sdk_call_accepted`, `last_rc`, `last_error`, retry timing, and failure
count. A mode loss or bad feedback discards the queued target; SDK failures back
off and are eventually abandoned. A local SDK return is not a position result,
and LowState position is not independent visual proof.

The packaged telemetry names joints by `motor_state_parallel` position. Verify
the exact K1 ordering before treating the first two entries as HeadYaw and
HeadPitch; plausible values alone cannot prove identity.

## Education/Jetson and Geek/Qualcomm differ

Treat configuration labels as descriptions, not compatibility switches.
Record the exact service interpreter, imported SDK path/version, firmware,
available ROS interface, and return/exception behavior on each robot. A
reported `None` success convention is normalized only in specific packaged SDK
calls; do not generalize it to every method or SDK build.

## CUSTOM or learned policy questions arise

Stop and use the separate low-level deployment review. This package uses
high-level WALK and a simple head call; it does not enter PREP/CUSTOM, publish
LowCmd/MotorCmd, define the 22-joint order/modes/gains/weights, or deploy a
checkpoint. WALK evidence cannot validate that low-level path.

## Tests pass but the package still may not be ready

Offline tests use fakes and synthetic inputs. They do not prove the active SDK,
service layout, network/auth provisioning, UWB RF quality, firmware delivery,
stopping distance, or physical collision avoidance. Follow
[TESTING_CHECKLIST.md](TESTING_CHECKLIST.md) and retain `unknown` for every
untested layer.
