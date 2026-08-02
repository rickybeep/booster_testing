# Package Manifest

> Status: self-contained V10 partner candidate for offline review. It is not a
> deployed release, has no live-test result, and has not been committed or
> pushed as part of this preparation.

## Intended handoff inventory

### Package root

- `.gitattributes`
- `.gitignore`
- `README.md`
- `BASELINE_WALK_AND_HEAD.md`
- `BASELINE.md` (compatibility pointer)
- `HOW_THE_TRACKER_WORKS.md`
- `API_CONTRACT.md`
- `SDK_AND_RUNTIME.md`
- `V10_SOURCE_NOTES.md`
- `CALIBRATION.md`
- `MOTION_GUARD_INTEGRATION.md`
- `OPERATIONS.md`
- `TESTING_CHECKLIST.md`
- `TROUBLESHOOTING.md`
- `PACKAGE_MANIFEST.md`
- `SHA256SUMS.txt`
- `requirements.txt`
- `pytest.ini`

### Configuration

- `config/README.md`
- `config/tracker.example.json`

The example is deliberately non-live. Real addresses, token-file paths,
calibration, tag assignments, and waypoints belong in ignored
`config/tracker.local.json` and are not handoff content.

### PC runtime

- `pc/README.md`
- `pc/arena_runner.py`
- `pc/tracker_config.py`
- `pc/booster_config.py`
- `pc/booster_nav.py`
- `pc/booster_motion.py`
- `pc/track_uwb.py`

This subtree is the fixed-waypoint Windows controller: strict config, UWB
input, geometry/navigation, authenticated HTTP feedback, guarded WALK velocity,
and a simple stopped head scan. It has no Booster SDK import and no deployment
client.

### Robot runtime

- `robot/README.md`
- `robot/runtime/__init__.py`
- `robot/runtime/k1_control_server.py`
- `robot/runtime/k1_guard_protocol.py`
- `robot/runtime/k1_mode_watcher.py`
- `robot/runtime/k1_motion_guard.py`
- `robot/runtime/k1_motion_zero_once.py`
- `robot/runtime/k1_runtime_health.py`
- `robot/runtime/k1_telemetry_daemon.py`
- `robot/runtime/runtime.env.example`
- `robot/runtime/runtime_manifest.json`
- `robot/runtime/systemd/k1-control-server.service`
- `robot/runtime/systemd/k1-mode-watcher.service`
- `robot/runtime/systemd/k1-motion-guard.service`
- `robot/runtime/systemd/k1-panel-v10.target`
- `robot/runtime/systemd/k1-telemetry.service`

The JSON runtime manifest is the exact allowlist for files under
`robot/runtime`. Its status is `offline_review_only_not_deployed`. It is a
filename inventory. [SHA256SUMS.txt](SHA256SUMS.txt) is the separate
cryptographic inventory for every distributable package file except itself.

`runtime.env.example` is deliberately non-runnable. The real, untracked
`/etc/booster-track-v10/runtime.env` must select a verified interpreter, SDK
module root, and ROS setup scripts for one target robot; it is not included.

### Offline tests

- `tests/README.md`
- `tests/conftest.py`
- `tests/test_bundle.py`
- `tests/test_config.py`
- `tests/test_motion_client_contract.py`
- `tests/test_motion_guard.py`
- `tests/test_navigation.py`
- `tests/test_pc_tracking_safety.py`
- `tests/test_robot_runtime_safety.py`
- `tests/test_status_contract.py`

Generated test caches and bytecode are not release files.

## Dependency boundary

`requirements.txt` directly pins:

- `numpy==1.26.4`;
- `pyserial==3.5`;
- `requests==2.32.3`; and
- `pytest==9.1.1`.

These are direct pins, not a transitive lock. No Pixi/Conda lock, wheelhouse,
virtual environment, or interpreter is included. Reproducibility therefore
depends on recording the resolver output and Python version used for the final
offline run.

Robot-side prerequisites are external: Linux/systemd, Python 3, the installed
Booster Python SDK, ROS 2 Humble, BoosterRos2Interface and its message package,
vendor services/firmware, and UWB hardware. Their binaries and licenses are not
part of this folder.

## Provenance

- V10-derived and reduced/adapted: selected navigation, motion-client, UWB,
  guard, telemetry, and mode-watcher logic.
- Candidate-specific: fixed-waypoint runner, strict shareable schema,
  bearer-authenticated minimal bridge, service templates, tests, and handoff
  documentation.
- Reported but unverified environment context: firmware `1.7.0.7`, a
  firmware-local K1 SDK v1.5.6, a separate older/unversioned global B1 SDK, ROS
  2 Humble, BoosterRos2Interface, and a 22-joint K1 low-level contract.

The last item is historical context, not proof about either target robot. The
22-joint CUSTOM path is not implemented by this WALK package.

## Explicit exclusions

- Real bearer tokens, passwords, SSH keys, `.env` files, credentials, and
  private endpoint/configuration files.
- Logs, captures, processed-person material, images, video, audio, crash dumps,
  PID/port files, `.pytest_cache`, `__pycache__`, and bytecode.
- iPad, dashboard, camera, stream, person/human detection, cloud, upload,
  hand-control, speech, and unrelated UI code.
- Mode-change, PREP, DAMP, gait-selection, low-level LowCmd/MotorCmd, learned
  policy, training, checkpoint, simulation, and CUSTOM-control code.
- Historical tracker trees, archives, unrelated repositories, vendor SDK/ROS
  binaries, firmware, OS images, and existing robot services.
- Install/deploy scripts, service replacement, migration, rollback, and
  production secret provisioning. Those workflows have not been reviewed into
  this candidate.

## Recorded offline verification

On 2026-07-31, the final candidate was checked on Windows with Python 3.10.9,
NumPy 1.26.4, pyserial 3.5, requests 2.32.3, and pytest 9.1.1. With bytecode and
pytest caching disabled:

- the complete default suite passed: `115 passed`;
- the disabled example parsed with no enabled robots and
  `live_motion: false`;
- 23 Python files compiled to AST in memory;
- both distributed JSON files parsed;
- the exact 53-file release inventory and all 52 recorded file hashes matched;
- local Markdown links, the robot-runtime allowlist, excluded-artifact rules,
  and selected secret/private-address patterns passed; and
- generated pytest/bytecode caches were absent after the run.

These were offline operations. No serial device, network endpoint, robot SDK,
SSH session, service, deployment action, POST request, head command, body
command, or joint command was used.

## Offline release checks

Before staging or sharing, repeat these checks against the exact final bytes:

1. Remove all generated caches and files outside the intended inventory.
2. Run the default tests from the final snapshot with bytecode/cache creation
   disabled and preserve the exact result.
3. Run disabled-example and ignored-local-config validation without `--live`.
4. Search the final bytes for secrets, real private addresses, absolute source
   workspace paths, and excluded personal/capture material.
5. Regenerate and review `SHA256SUMS.txt`; its offline test must match every
   intended file and reject missing or extra distributable files.
6. Record dependency resolution and licenses as required by the recipients.
7. Keep deployment and all live command testing explicitly unperformed unless
   separately reviewed and authorized.

Passing these release checks establishes a clean offline candidate only, not
hardware compatibility, firmware acceptance, physical safety, or deployment
readiness. Linux systemd verification, target-specific environment inspection,
installation, and every live command test remain separate work.
