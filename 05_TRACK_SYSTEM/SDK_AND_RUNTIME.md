# SDK, Robot Variants, and Runtime Prerequisites

## Where the Booster SDK runs

The Windows PC does not import the Booster SDK. It sends authenticated HTTP
requests to the candidate bridge on each robot. The on-robot guard, telemetry
daemon, and head bridge import `booster_robotics_sdk_python` and initialize
the vendor channel on robot loopback:

```python
sdk.ChannelFactory.Instance().Init(0, "127.0.0.1")
```

Locomotion uses high-level `B1LocoClient.Move(vx, vy, vyaw)` in existing WALK
mode. Head slew uses `B1LocoClient.RotateHead(pitch, yaw)`. The runner never
changes mode, and this package does not publish LowCmd, enter CUSTOM, or solve
the separate 22-joint learned-policy handoff.

Relevant implementations:

- [pc/booster_motion.py](pc/booster_motion.py): authenticated PC client and
  lease ownership checks;
- [robot/runtime/k1_motion_guard.py](robot/runtime/k1_motion_guard.py): sole
  persistent/nonzero WALK `Move` owner in this package;
- [robot/runtime/k1_control_server.py](robot/runtime/k1_control_server.py):
  HTTP, feedback, and head bridge; and
- [robot/runtime/k1_telemetry_daemon.py](robot/runtime/k1_telemetry_daemon.py):
  read-only SDK feedback subscriber.

## PC prerequisites

- Windows 10/11 and Python 3.10;
- direct dependencies pinned in [requirements.txt](requirements.txt): NumPy
  1.26.4, pyserial 3.5, requests 2.32.3, and pytest 9.1.1;
- an explicitly selected UWB serial port for any live run; and
- controlled IP reachability to each configured robot bridge.

No virtual environment, package cache, transitive lock, wheelhouse, SDK wheel,
ROS installation, or robot firmware is included.

## Robot prerequisites and explicit runtime selection

The candidate service templates expect Linux/systemd, `fcntl`, Unix-domain
sockets, `SO_PEERCRED`, a `booster` user/group, and release content at
`/opt/booster-track-v10/current/robot_runtime`.

Every unit also requires a separately provisioned
`/etc/booster-track-v10/runtime.env`. The non-runnable template is
[runtime.env.example](robot/runtime/runtime.env.example). Operators must verify
and set:

- `K1_PYTHON`: exact executable paired with the firmware-compatible K1 SDK;
- `PYTHONPATH`: exact SDK/dependency search path;
- `K1_SDK_EXPECTED_ROOT`: exact permitted root for the imported
  `booster_robotics_sdk_python` module;
- `PYTHONNOUSERSITE=1`;
- `ROS_SETUP`: exact ROS 2 setup script; and
- `BOOSTER_ROS_SETUP`: exact BoosterRos2Interface setup script.

SDK-using processes reject an imported SDK outside the selected root and
accept only `None` or integer zero from SDK/channel initialization. This is a
guard against accidentally resolving the reported older global B1 SDK; it is
not proof that the selected installation is compatible.

The mode watcher additionally expects `booster_interface.msg.RobotStatesMsg`
and `/robot_states`. Candidate runtime snapshots, socket, and lock live under
`/run/booster-track-v10` with the service umask/runtime-directory settings.
The control server requires a separate regular token file at
`/etc/booster-track-v10/control.token`; no token is included.

## Candidate process model

| Unit | Responsibility | Motion authority |
|---|---|---|
| `k1-motion-guard.service` | SDK WALK client, ownership lease, sequence checks, expiry, and bounded stop bursts | Sole persistent and sole nonzero `Move` owner in this package |
| `k1-control-server.service` | Authenticated HTTP proxy, status, and feedback-gated head slew | No direct `Move`; calls `RotateHead` |
| `k1-telemetry.service` | LowState/odometry snapshots | Read-only |
| `k1-mode-watcher.service` | ROS mode snapshot | Read-only |
| `k1-panel-v10.target` | Groups the four candidate services | None |

The lock-gated `k1_motion_zero_once.py` helper may create a temporary SDK
client only after the guard releases its cooperative lock, and only to attempt
five checked zero commands after guard exit.

The target has no `[Install]` section, so this candidate is not boot-enabled.
Services use `Restart=on-failure`, a 2 s delay, and a three-start/30 s limit.
Starting or stopping the guard is not read-only because startup, acquisition,
expiry, release, and shutdown paths can attempt zero velocity.

There is no reviewed installer, migration, rollback, existing-service
replacement, port-conflict resolver, or live deployment workflow in this
package.

## Education/Jetson and Geek/Qualcomm

The configuration labels `education_jetson` and
`geek_qualcomm_qrb5165` identify the intended hardware variants. A label does
not prove binary or service compatibility. Before installation on either
robot, independently verify:

- hardware/OS identity and firmware;
- exact interpreter and SDK module `__file__`;
- SDK class availability, initialization, return values, and exceptions;
- ROS/BoosterRos2Interface setup paths, message type, and `/robot_states`;
- loopback DDS/RPC behavior and network port ownership;
- service-user access to the release tree, token, and runtime directory; and
- head and WALK support on that exact firmware.

Reported context only--not reverified by this package--is firmware `1.7.0.7`, a
firmware-local K1-aware SDK v1.5.6, a separate older/unversioned global B1 SDK,
ROS 2 Humble, BoosterRos2Interface, and a 22-joint K1 CUSTOM contract.

Three exact cross-interface contracts remain deployment-blocking unknowns:

1. ROS `/robot_states.current_mode` numbers must equal the installed SDK
   `RobotMode` numbers used by the HTTP WALK gate.
2. `motor_state_parallel` ordering must match the packaged 22-name list,
   especially index 0 HeadYaw and index 1 HeadPitch, before telemetry may gate
   head motion.
3. SDK return/exception codes 100 and 400 must exclusively mean the assumed
   safe non-WALK rejection when a zero is attempted.

Verify these against current K1 headers/bindings, BoosterRos2Interface sources,
and read-only observations on each exact target. Plausible values or advancing
timestamps are not mapping evidence.

## Rates and freshness

- Runner and navigation loop: configured to 20 Hz in the example.
- Robot navigation snapshot: up to 20 Hz.
- PC status/mode polling: 2 Hz.
- Guard lease: 400 ms by default; accepted range 300-500 ms.
- Lease-renew attempt: every 100 ms.
- PC moving-command deadman: 0.90 s.
- PC mode freshness limit: 1.25 s.
- Example heading freshness: 0.65 s for Education and 1.25 s for Geek; schema
  validation forbids a heading threshold above the mode threshold.
- Example UWB fix freshness: 0.60 s; schema validation forbids a UWB threshold
  above the motion deadman.
- Robot head baseline freshness: 0.75 s.

These are software timings, not measured network latency, braking distance, or
physical guarantees.

## Network boundary

The HTTP server binds to `0.0.0.0:8080` by default. Every remote route requires
an exact bearer token; only loopback `GET /api/health` may be unauthenticated.
The server rejects redirects at the PC client and rejects ambiguous request
framing, but it has no TLS. Use only on an isolated, access-controlled robot
network and assume anyone able to observe traffic can observe the token.

See [API_CONTRACT.md](API_CONTRACT.md) for route-level semantics and
[TESTING_CHECKLIST.md](TESTING_CHECKLIST.md) for offline, read-only, and
separately authorized hardware evidence levels.
