# Track System Verification Checklist

> Status: offline-review candidate. Nothing in this checklist authorizes
> installation, service management, a POST request, a mode change, head motion,
> or body motion.

## Evidence rule

Test the layer named by the claim. A clean process exit, HTTP 200 response,
advancing LowState stream, guard acknowledgement, or local SDK return proves
only that particular layer. None alone proves firmware consumption or physical
motion.

Record exact commands, timestamps, file/config hashes, stdout/stderr, and the
observer for every result. Use `pass`, `fail`, or `unknown`; do not turn missing
evidence into a pass.

## Level 0: package gate

- [ ] Review the intended inventory in [PACKAGE_MANIFEST.md](PACKAGE_MANIFEST.md).
- [ ] Confirm the robot subtree exactly matches
      [runtime_manifest.json](robot/runtime/runtime_manifest.json).
- [ ] Remove generated `.pytest_cache`, `__pycache__`, and `*.pyc` files from
      the release material.
- [ ] Confirm imports and relative Markdown links resolve inside this package.
- [ ] Confirm the example remains disabled and uses documentation addresses.
- [ ] Search for credentials, real private addresses, captures, logs, and
      personal material.
- [ ] Keep each real bearer token in its untracked local regular file; verify
      the PC and robot use the same 32-512 character ASCII token without
      printing it, and never paste it into a committed configuration, command
      log, screenshot, or report.
- [ ] Verify [SHA256SUMS.txt](SHA256SUMS.txt) against all intended package
      files. The bundled runtime JSON remains a separate filename allowlist for
      only `robot/runtime`.
- [ ] Reconcile every reported environment fact with the exact target robot.

## Level 1: offline validation

These commands do not require a robot or UWB receiver. Creating the environment
may contact a Python package index; it still must not contact the arena network.

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
$env:PYTHONDONTWRITEBYTECODE = "1"
.\.venv\Scripts\python -m pytest -q -p no:cacheprovider
.\.venv\Scripts\python pc\arena_runner.py --config config\tracker.example.json
```

Expected runner observation: valid configuration, no enabled robots, and
`live_motion: false`. Because `--live` is absent, the runner must not open a
serial device, network connection, SDK client, or command path.

The default pytest selection excludes tests marked `hardware` or
`live_motion`. It currently checks package/manifest integrity, strict config
and launch gates, geometry and navigation behavior, motion-guard leases and
zero bursts, PC ownership/acknowledgement handling, robot HTTP authentication
and mode gates, head retry state, and status freshness. See
[tests/README.md](tests/README.md) for the exact file mapping.

- [ ] Dependency installation completed and versions were recorded.
- [ ] The complete default suite passed; attach its exact output and count.
- [ ] Disabled-example validation produced the expected non-live result.
- [ ] A copied, ignored `config/tracker.local.json` also validates with
      `live_motion_enabled: false` and all robots disabled.
- [ ] No test or validation command opened real serial/network endpoints.

An offline pass establishes only the tested Python behavior.

## Level 2: separately authorized read-only inspection

This package has not been deployed from this folder. Perform this level only on
an already installed runtime and only with the robot owner's authorization. Do
not copy files, start/stop/restart services, change mode, acquire a guard lease,
or call any POST endpoint. Starting or stopping the motion guard attempts SDK
zero commands and is therefore not read-only.

### Robot environment

- [ ] Record hardware identity without assuming the configured variant label
      is correct.
- [ ] Record firmware version, service Python executable, active SDK
      `__file__`, available package metadata, and SDK return conventions.
- [ ] Record ROS 2 Humble and BoosterRos2Interface setup paths and message type.
- [ ] Prove from installed sources/types that ROS `current_mode` numeric values
      equal the SDK `RobotMode` values used by the bridge; record WALK exactly.
- [ ] Prove the installed K1 `motor_state_parallel` ordering, including
      HeadYaw/HeadPitch indices, before relying on named head feedback.
- [ ] Prove the exact installed SDK/firmware meaning of zero return/exception
      codes 100 and 400; do not infer it from the candidate allowlist.
- [ ] Confirm the external runtime environment selects absolute, verified
      `K1_PYTHON` and `K1_SDK_EXPECTED_ROOT` values and disables user-site
      imports; do not infer these from the global shell.
- [ ] Inspect status only for `k1-panel-v10.target`, `k1-motion-guard.service`,
      `k1-control-server.service`, `k1-mode-watcher.service`, and
      `k1-telemetry.service` if those candidate units are already installed.
- [ ] Confirm the process bound to TCP port 8080 and compare installed files to
      independently recorded hashes.

### Authenticated GET observations

Except for `/api/health` from robot loopback, the packaged server requires the
same bearer token used by the PC client. Obtain it through the approved local
secret-handling process; do not print or record it. Repeatedly sample only:

- `GET /api/status`;
- `GET /api/guard/status`;
- `GET /api/nav`;
- `GET /api/telemetry`; and
- `GET /api/health`.

- [ ] Unauthorized remote GETs return 401.
- [ ] `mode_age_s` is finite and fresh, or the result is explicitly recorded
      as unknown/stale.
- [ ] `/api/nav.ts` advances; yaw, gyro Z, odometry, and rate fields are finite.
- [ ] Guard state, readiness, generation, error, zero outcome, and active
      target are recorded without acquiring a lease.
- [ ] No conclusion about command delivery is drawn from GET reachability.

The current HTTP `/api/health` response contains guard status and nav-source
age; it does not report motor temperatures or a motor-health verdict. The
separate local runtime-health helper reports `guard_ready`, never
`motion_allowed`.

## Level 3: non-actuating transport conclusions

Read-only evidence can prove that the PC reaches the intended authenticated
HTTP service, that the control server reaches the local Unix-socket guard, and
that telemetry/mode snapshots are advancing. It cannot exercise the body or
head actuator path without a command.

The package uses high-level `B1LocoClient.Move`, not raw `rt/joint_ctrl`
LowCmd. It contains no definitive DDS matched-subscription diagnostic for that
low-level topic. Do not substitute LowState reception, HTTP reachability, or a
historical local `Write()` return for matched firmware-subscriber evidence.

- [ ] State exactly which transport boundary was observed.
- [ ] Leave firmware acceptance and physical response as `unknown` unless
      independently demonstrated by firmware-side evidence in an approved test.
- [ ] Verify no unrelated process is an uncoordinated WALK or CUSTOM owner.

## Level 4: live command testing

No Level-4 test is approved by this document. Before proposing one, provide:

- one named robot and verified hardware variant;
- exact package, runtime, dependency, and local-config hashes;
- firmware, active SDK import path, and service topology;
- a single isolated variable and minimum command envelope;
- verified initial WALK state and zero intended velocity;
- maximum speed, turn rate, duration, distance, and head range;
- clear arena, physical keepouts, operator, spotters, and independent stop
  authority;
- expected software, firmware-side, and physical observations for success and
  failure; and
- abort, ambiguous-result, and recovery procedures.

The operator must provide explicit approval after reviewing that plan. The
`--live` flag, configuration enable, and acknowledgement string are software
gates, not human authorization.

## Evidence expected if a live test is later approved

Guard acquisition itself performs a strict zero SDK call before creating the
session; it is command-producing and not a read-only probe. For a changed body
target, correlate the HTTP request, guard generation,
session, sequence, desired velocity, acknowledged velocity, local SDK result,
fresh WALK mode, and independent firmware/physical observation. An identical
target may advance the lease without another SDK call only when desired and
acknowledged velocity already match, so its response cannot prove a new
downstream delivery.

For a head target, the POST response proves queue acceptance only. Later
authenticated status can distinguish `pending`, fresh measured position and
age, `last_sdk_call_accepted`, `last_rc`, and `last_error`. Correlate advancing,
non-lost LowState head feedback with the target. The packaged baseline rejects
samples older than 0.75 s, missing/duplicate/non-finite/lost joints, or reported
positions outside each command envelope plus 0.05 rad. A local SDK return alone
is not a position result, and LowState alone is not independent visual proof.

For stop/release/expiry, record each acknowledgement, guard terminal state,
stop-burst count, zero outcome, firmware-side evidence, and independent robot
observation. Five attempted zeros and a 300-500 ms lease are bounded software
intent, not a guaranteed physical stopping deadline: a blocking vendor call or
downstream failure can delay or prevent physical effect.

## Result record

For every completed level, preserve:

| Field | Required value |
|---|---|
| Candidate identity | commit or package hash; never just `latest` |
| Robot identity | none for offline work, otherwise named hardware |
| Environment | OS, Python, dependencies; firmware/SDK/ROS when applicable |
| Configuration | hash plus secret-free calibration summary |
| Exact action | command or read-only observation performed |
| Expected result | success and failure observations defined in advance |
| Actual evidence | timestamps, output, firmware evidence, observer |
| Verdict | pass, fail, or unknown with reason |
