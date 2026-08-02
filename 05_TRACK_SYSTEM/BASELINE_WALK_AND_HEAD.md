# V10 WALK and Head Baseline

> Status: self-contained offline-review candidate. It has not been installed
> from this folder on either K1 variant and has not passed an approved physical
> motion test.

## Scope

This baseline is the partner-facing subset of the V10 arena tracker. It reads
UWB position and robot heading on a Windows PC, follows configured waypoints,
uses circular keepouts for static obstacles and other tracked robots, sends
high-level WALK velocity through an on-robot lease guard, and performs a simple
head scan while stopped.

It does not publish `LowCmd` or `MotorCmd`, stream joint targets, change robot
mode, enter PREP or CUSTOM, deploy a learned policy, or include the broader
iPad, person-detection, camera, audio, cloud, and dashboard workflows. The
minimal PC client states that public boundary explicitly in
[pc/booster_motion.py](pc/booster_motion.py#L1).

## Candidate identity

| Field | Value |
|---|---|
| Tracker lineage | `TRACK_CONTROL/BOOSTER_TRACK_V10` |
| Shared UWB lineage | `TRACK_CONTROL/track_uwb.py` |
| Package status | Offline review only; uncommitted and not deployed |
| Configuration schema | `1` |
| PC baseline | Windows, Python 3.10, direct dependencies pinned in `requirements.txt` |
| Robot runtime identity | `V10-partner-candidate`, protocol `1` |
| Supported labels | `education_jetson`, `geek_qualcomm_qrb5165` |
| Physically verified variants | None from this package |
| Live-motion authorization | None |

The robot runtime's own file allowlist and undeployed status are recorded in
[robot/runtime/runtime_manifest.json](robot/runtime/runtime_manifest.json).
That JSON file is a runtime-subtree allowlist. The final bytes of every
distributable package file except the checksum file itself are recorded in
[SHA256SUMS.txt](SHA256SUMS.txt).

## Packaged command path

The PC runner produces forward velocity and yaw-rate intent. It always sends
zero lateral velocity. The PC applies the configured turn-sign conversion and
requires a matching guard generation, session, sequence, desired velocity, and
acknowledged velocity before recording a command as locally accepted; see
[pc/booster_motion.py](pc/booster_motion.py#L834).

The bearer-authenticated robot HTTP service applies a fresh-WALK gate and
forwards locomotion requests to the Unix-socket guard.
The persistent guard owns `B1LocoClient.Move`; a separate lock-gated helper may
attempt a checked five-zero shutdown burst only after the guard releases its
cooperative motion lock. The guard contract is documented
in [MOTION_GUARD_INTEGRATION.md](MOTION_GUARD_INTEGRATION.md).

Local acknowledgement proves only that the guard processed the request and its
SDK state agrees. A repeated target may reuse an earlier SDK acknowledgement,
and even a fresh SDK return is not firmware-side or physical-motion proof. Head
POST success proves queue acceptance only. Later head status separates local
SDK-call acceptance from fresh, non-lost LowState head-position feedback; that
feedback supports a reported-position claim but is not independent visual or
causal proof.

## Reported environment context

The following facts came from earlier robot inspection and project records;
they have not been reverified by this copied package:

- robot firmware was reported as `1.7.0.7`;
- a K1-aware `booster_robotics_sdk_python` v1.5.6 was reported inside the
  firmware installation;
- another globally visible Python SDK appeared to be an older, unversioned
  B1-oriented build;
- ROS 2 Humble and BoosterRos2Interface were reported on the robot; and
- K1 low-level feedback was reported as 22 joints.

The 22-joint detail belongs to the separate CUSTOM/learned-policy handoff. This
WALK package neither tests nor relies on that low-level command contract.

Before any installation, record the exact firmware, Python executable, SDK
`__file__`, package metadata, ROS setup paths, service user, network interface,
and return/exception behavior on the specific robot. Do not infer compatibility
from the hardware-variant label alone.

## Provenance classes

- Copied and then adapted: `pc/booster_motion.py`, `pc/booster_nav.py`,
  `pc/track_uwb.py`, and selected robot guard/runtime files.
- Package-specific: `pc/arena_runner.py`, `pc/tracker_config.py`, the sanitized
  configuration, minimal HTTP bridge, systemd templates, tests, and handoff
  documentation.
- External prerequisites: robot firmware, vendor SDK, ROS 2,
  BoosterRos2Interface, systemd, and physical UWB hardware.

See [V10_SOURCE_NOTES.md](V10_SOURCE_NOTES.md) for the detailed adaptation
record and [PACKAGE_MANIFEST.md](PACKAGE_MANIFEST.md) for the package inventory.

## Reproducibility and evidence status

The direct PC dependencies are pinned, but there is no transitive environment
lock file. The whole-package SHA-256 inventory freezes these source bytes; it
does not freeze package-index resolution, external robot dependencies, or the
target environment.

Offline tests use fakes and synthetic geometry. Their result establishes only
the tested software behavior. It does not establish robot compatibility,
firmware command acceptance, stopping distance, keepout safety, or physical
motion. Record the exact final test output only after all package edits are
complete.

No statement in this document authorizes deployment, mode changes, POST
requests to a robot, SDK commands, or live motion.
