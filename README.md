# Booster K1 motion-training collaboration

This repository is the working handoff for two learned Booster K1 behaviors:

1. **Phase 1 — squat**
2. **Phase 2 — sit down on the floor, pause, and stand back up**

We are starting with the squat. The immediate goal is to complete one
end-to-end cycle together: agree on the motion, train and evaluate it, export
the policy, adapt the deployment path to our K1, and conduct a coordinated
physical test. We want both a working behavior and a workflow our team can
repeat afterward.

## What we have already tried

For the squat, we created a procedural 22-joint reference, converted it for a
BeyondMimic-style workflow, trained with Isaac Lab and RSL-RL, and reviewed
the result in simulation and MuJoCo. We produced a plausible simulation
replay but did not complete a reliable physical deployment.

For the sit/get-up motion, we processed a human reference video through
GVHMR and produced an early K1 retarget. It communicates the desired overall
sequence, but the contact strategy and motion still need substantial work.

The files here are examples and starting points. This repository does **not**
contain our old squat checkpoint, training workspace, or deployment code.
We would prefer to begin from your known K1 workflow and adapt these references
to it.

## Repository guide

- [`01_SQUAT_FIRST/PHASE_1_BRIEF.md`](01_SQUAT_FIRST/PHASE_1_BRIEF.md) —
  desired squat and included example.
- [`01_SQUAT_FIRST/OUR_PROGRESS.md`](01_SQUAT_FIRST/OUR_PROGRESS.md) — short
  summary of our prior squat workflow and result.
- [`02_SIT_GETUP_NEXT/PHASE_2_BRIEF.md`](02_SIT_GETUP_NEXT/PHASE_2_BRIEF.md) —
  desired floor sit/get-up motion and reference material.
- [`03_ROBOT_AND_DEPLOYMENT_CONTEXT/README.md`](03_ROBOT_AND_DEPLOYMENT_CONTEXT/README.md) —
  robot variants, supplied model, and deployment context.
- [`03_ROBOT_AND_DEPLOYMENT_CONTEXT/arena_patrol_example/`](03_ROBOT_AND_DEPLOYMENT_CONTEXT/arena_patrol_example/) —
  minimal tracked-arena walking, calibration, bounds, and head-command example.
- [`KICKOFF_QUESTIONS.md`](KICKOFF_QUESTIONS.md) — decisions for the first
  working session.

## Robots

We have:

- Booster K1 Education with NVIDIA Jetson compute
- Booster K1 Geek with Qualcomm compute

The repository includes the stock 22-DOF K1 model and meshes. We have not
added a custom payload or attachment for either behavior.

## Requested collaboration output

For each behavior, we would like:

- Motion/reference data in the format used by the training workflow
- Training task, configuration, rewards, and curriculum
- Evaluation criteria and useful diagnostics
- Selected checkpoint and exported policy
- Deployment code compatible with the agreed K1 target
- A coordinated physical-test procedure
- Concise setup and run instructions
- A walkthrough so our team can retrain and extend the behavior

## Large files

Videos, motion arrays, model data, and meshes use Git LFS. Install Git LFS
before cloning or run `git lfs pull` after cloning to retrieve them.
