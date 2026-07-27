# Phase 1: learned squat

## Desired outcome

Create, train, deploy, and physically verify a controlled whole-body squat for
the Booster K1.

The robot should:

1. Begin in a stable upright stance.
2. Descend smoothly into an agreed squat depth.
3. Maintain balance and controlled feet, knees, torso, and arms.
4. Pause briefly if that is part of the agreed reference.
5. Rise smoothly to a stable upright stance.

Exact depth, tempo, arm styling, pause duration, and repetition count should
be agreed during the first session. We are open to replacing our existing
reference if your workflow produces a better motion.

## What the example contains

`example/` contains:

- `k1_squat_reference_50fps.csv` — our procedural 14-second squat reference.
- `k1_squat_reference_50fps.npz` — the converted 50 Hz, 22-joint
  BeyondMimic-style motion.
- `squat_simulation_example.mp4` — a representative simulation replay from
  our prior training work.

The MP4 is only an example of what we tried. It is not proof of physical
success and should not constrain the improved behavior you create.

For more detail, see:

- [`TRAINING_AND_TEST_TIMELINE.md`](TRAINING_AND_TEST_TIMELINE.md)
- [`SELECTED_RUN_SUMMARY.md`](SELECTED_RUN_SUMMARY.md)
- [`../03_ROBOT_AND_DEPLOYMENT_CONTEXT/WHAT_WE_TRIED.md`](../03_ROBOT_AND_DEPLOYMENT_CONTEXT/WHAT_WE_TRIED.md)

## Current status

- We generated and trained against the procedural reference.
- We evaluated selected checkpoints offline in Isaac Lab and MuJoCo.
- We exported a policy, but did not obtain firmware-confirmed physical policy
  execution.
- A current-position hold canary entered CUSTOM, but firmware reported that
  it had no valid command snapshot and received zero motor commands where 22
  were required.

We therefore want the first collaboration milestone to include both the
learned squat and a deployment path that is demonstrably accepted by the
robot firmware.

## Suggested milestone sequence

1. Agree on squat style and physical success criteria.
2. Run your known training workflow with the K1 model.
3. Review a clean simulation video and quantitative evaluation.
4. Export the selected policy and validate the deployment interface without
   actuation.
5. Test a separately approved safe hold with firmware-side acceptance
   evidence.
6. Conduct the squat test only after the transport and safety gates pass.
