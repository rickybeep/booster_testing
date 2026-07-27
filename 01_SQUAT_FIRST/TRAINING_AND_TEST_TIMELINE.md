# Squat train-to-robot attempt timeline

## Motion creation

The squat was procedurally generated from the K1 URDF geometry. Default timing:

- Stand: 1 second
- Descend: 3 seconds
- Bottom hold: 5 seconds
- Rise: 3 seconds
- Final stand: 2 seconds
- Knee depth: 1.4 radians
- Sample rate: 50 Hz

The generator uses minimum-jerk interpolation and moves the trunk so the
ankles remain fixed in the sagittal plane. The CSV was converted through the
Booster BeyondMimic pipeline.

## Training progression

| Run | Purpose | Saved artifacts |
|---|---|---|
| `2026-07-09_17-38-07_smoke` | Initial one-iteration task smoke | `model_0` |
| `2026-07-10_12-23-12_approved_motion_smoke` | Rebuilt-motion smoke | `model_0` |
| `2026-07-10_12-23-47_approved_baseline_100` | Short baseline | `model_0`, `model_99` |
| `2026-07-10_12-51-20_approved_1000` | First longer baseline | `model_0`, `model_999` |
| `2026-07-10_13-45-03_approved_1000_checkpoint100` | Frequent checkpoint run | `model_0` through `model_600` |
| `2026-07-10_14-09-48_approved_resume_200_to_1000` | Resume and export | models 200-600, export, video |
| `2026-07-10_15-49-32_approved_resume_600_to_800` | Resume and export | models 600, 700, 799, export, video |
| `2026-07-10_16-26-59_latency40ms_resume_799_to_999` | 40 ms-delay experiment | models 800, 900, 998, export |
| `2026-07-10_17-16-29_cliff_10_30ms_resume_799_to_1200` | Selected 10-30 ms-delay run | models 800, 900, 1000, 1100, 1198, exports |

The package contains the selected final run in full except for unrelated root
workspace diff data. Two earlier replay videos are included and explicitly
labelled as representative earlier checkpoints, not `model_1198`.

## Offline evaluation

The first robustness result was invalidated because a MuJoCo call reset the
root height and shoulder pose. The test was fixed to preserve initialized
`qpos` and `qvel`, then rerun.

- Current deploy gains: 16/20 randomized passes.
- Controlled delay: pass at 0, 10, 20, and 30 ms; fail at 35 and 40 ms.
- Recorded training gains with current conservative effort limits: 20/20.
- Controlled candidate delay tests also passed at 45, 50, 60, and 80 ms.
- Checkpoints 800, 900, 1000, 1100, and 1198 were compared. Earlier
  checkpoints did not eliminate the 35 ms cliff.

These are simulation results, not hardware safety evidence.

## Physical handoff canary

Before running the squat, a raw full-body hold was used to test entry into
CUSTOM. The probe locally created 22 `MotorCmd` entries and called `Write()` at
500 Hz.

In the robot log on 2026-07-14:

1. WALK exited to PREP.
2. PREP exited to CUSTOM.
3. The robot became effectively limp.
4. Firmware logged 607 failures to obtain a custom command snapshot.
5. The firmware loop reported approximately 500.167 Hz.
6. Firmware reported `User motor command size mismatch, expected 22, got 0`.
7. The robot was returned to PREP.

The actual squat policy was not run during this event.

## Independent handoff audit

The audit found that:

- The raw Python SDK path used a legacy/unversioned B1-style runtime.
- Its writer defaulted to BEST_EFFORT and exposed no Python match-count API.
- The probe used PARALLEL ankle coordinates; the official K1 deployment route
  uses SERIAL.
- LowState reception and local `Write()` completion were not proof of a
  matched `rt/joint_ctrl` reader.
- The official ROS 2 controller used RELIABLE depth-1 QoS but had process
  ownership, publication-gap, freshness, and safety-gate defects.

The isolated prototype addresses those source-level defects but has not been
deployed.
