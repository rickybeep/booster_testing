# Robot Runtime Candidate

> These files are offline-review artifacts. They have not been installed from
> this package and this document does not authorize copying, enabling, starting,
> stopping, or replacing services on a robot.

The robot runtime is the narrow bridge between the Windows tracker and the K1's
installed Booster platform. Its exact source/unit allowlist is
[runtime/runtime_manifest.json](runtime/runtime_manifest.json).

## Components

| Component | Role | Command behavior |
|---|---|---|
| `k1_motion_guard.py` | Persistent Unix-socket lease owner and SDK WALK client | Calls `B1LocoClient.Move`; sends an initial zero and bounded zero burst on release/expiry |
| `k1_guard_protocol.py` | Bounded local protocol and same-UID peer validation | No robot command |
| `k1_control_server.py` | Bearer-authenticated HTTP feedback, fresh-WALK gates, lease/move proxy, and simple head queue | Proxies body commands to guard; calls `RotateHead` asynchronously |
| `k1_motion_zero_once.py` | Lock-gated post-exit fallback | Attempts a checked five-zero burst only after obtaining the SDK-motion file lock |
| `k1_telemetry_daemon.py` | LowState/odometry snapshots | Read-only SDK subscribers |
| `k1_mode_watcher.py` | ROS `/robot_states` mode snapshot | Read-only ROS subscriber |
| `k1_runtime_health.py` | Local guard/server status report | Read-only |
| `runtime.env.example` | Non-runnable interpreter/SDK/ROS path schema | No robot command |
| `systemd/*.service`, `k1-panel-v10.target` | Candidate supervision model | Starting/stopping the guard can issue zero commands |

The guard is the sole persistent and sole nonzero `Move` owner in this package.
The historically named one-shot helper is an exceptional, finite zero-only
path serialized by the same cooperative file lock.

## External prerequisites

The candidate service templates assume:

- Linux, systemd, Python 3, `fcntl`, Unix sockets, and `SO_PEERCRED`;
- user and group `booster`;
- an operator-verified K1-capable `booster_robotics_sdk_python`;
- a verified ROS 2 setup script (Humble is the reported environment);
- a verified BoosterRos2Interface setup script;
- `booster_interface.msg.RobotStatesMsg` and `/robot_states`;
- release content under
  `/opt/booster-track-v10/current/robot_runtime`; and
- local runtime state under `/run/booster-track-v10`.

The control-server unit also expects a provisioned regular token file at
`/etc/booster-track-v10/control.token`. The implementation rejects a symlink,
files larger than 4096 bytes, anything other than one restricted 32-512
character ASCII token (plus an optional final line ending), and unsafe POSIX
mode bits. No token is included here.

These paths are encoded in the candidate units, especially
[k1-mode-watcher.service](runtime/systemd/k1-mode-watcher.service). The vendor
SDK, ROS installation, firmware, base locomotion services, and OS are not
vendored here.

Every unit requires an external `/etc/booster-track-v10/runtime.env`; the
non-runnable schema is
[runtime.env.example](runtime/runtime.env.example). It selects `K1_PYTHON`,
`PYTHONPATH`, `K1_SDK_EXPECTED_ROOT`, `ROS_SETUP`, and `BOOSTER_ROS_SETUP`, and
sets `PYTHONNOUSERSITE=1`. SDK-using processes reject an import whose real path
falls outside the selected absolute SDK root, and accept only `None` or integer
zero from SDK initialization calls. This is intended to prevent accidental use
of the reported older global B1 SDK; it still requires the operator to select
the correct K1 installation.

The guard and head process initialize the vendor channel on `127.0.0.1` because
they run on the robot beside the vendor services. This is unrelated to the PC's
network interface choice.

## Service relationships

`k1-panel-v10.target` requires the motion guard and control server. Telemetry
and mode watcher are wanted helpers and themselves declare `Requires=` and
`After=` on the guard. Those directives enforce activation ordering/failure,
but do not guarantee that an already-running helper is stopped after every
unexpected guard crash; the units intentionally do not claim `BindsTo=`
semantics. Independently, the PC rejects stale feedback, a lost guard session,
or a changed guard generation.

The control server refuses startup when the local guard health precheck fails.
The guard unit runs the one-shot zero helper after exit. Consequently, service
startup and shutdown are not read-only operations even when every velocity is
zero.

There is no reviewed installer, migration, rollback, port-conflict resolver, or
deployed-file hash verifier in this package. Existing robot services must not
be replaced from these templates without a separate deployment review.

The target deliberately has no `[Install]` section; it is a manual-start
candidate, not boot-enabled. Service restarts use bounded systemd start limits.
`k1_runtime_health.py` reports process/guard readiness as `guard_ready`; it
never reports a `motion_allowed` decision.

## Network and variant limits

The HTTP server binds to `0.0.0.0:8080` by default and requires an exact bearer
token for every remote route. Only `/api/health` may omit it from robot
loopback. The server has no TLS, so use only on an isolated, access-controlled
robot network and treat the token as visible to anyone able to observe traffic.

The server requires fresh observed WALK mode for acquire, renew, nonzero move,
and head requests. Zero move and release remain available to an existing owner
when the mode gate closes. A guard lease provides locomotion ownership, not
network secrecy; authenticated head requests remain outside the guard-session
protocol.

Head queueing also requires fresh, non-lost `HeadPitch` and `HeadYaw` values in
the packaged LowState telemetry snapshot. The snapshot must be no older than
0.75 s and both unique, finite joint positions must remain within the command
envelopes plus 0.05 rad. The sender bases each further slew step on a newer
feedback timestamp and records SDK-call acceptance separately from measured
position. This is stronger than an open-loop local estimate, but it is still
not independent observation of physical motion.

A newer queued head target resets retry state and supersedes older results. A
vendor call already in flight cannot be cancelled, but its completion is
command-ID scoped so it cannot mutate the newer command's status.

The telemetry daemon assigns those joint names by position in
`motor_state_parallel`, and the mode gate compares a ROS `current_mode` number
with SDK `RobotMode` values. Exact K1 joint ordering and ROS/SDK enum equivalence
are deployment-blocking facts to verify on each target; the configuration label
does not prove either mapping.

Education/Jetson and Geek/Qualcomm are configuration labels only. Firmware
`1.7.0.7` and a firmware-local K1 SDK v1.5.6 were reported previously, while a
different global SDK appeared older and unversioned. None of those environment
facts has been reverified by installing this candidate. Capture the active SDK
import path and behavior from the exact service interpreter before deployment.

The runtime sends high-level WALK/head calls only. It does not publish the
22-joint CUSTOM command vector or solve learned-policy deployment.
