# Booster K1 learned-motion project

## What we are asking for

We would like your help creating two learned behaviors for our Booster K1
robots:

1. **Phase 1 — squat**
2. **Phase 2 — sit down on the floor, pause, and stand back up**

We are starting with the squat. The immediate goal is to work through one
complete motion-training cycle together: motion definition, training,
evaluation, policy export, deployment, and a safe physical test. We want both
a working behavior and a workflow our team can repeat afterward.

This folder is intentionally a small project brief, not a dump of every
experiment we have run.

## Folder guide

- [`01_SQUAT_FIRST/PHASE_1_BRIEF.md`](01_SQUAT_FIRST/PHASE_1_BRIEF.md) —
  first task, desired outcome, and one simulation example.
- [`02_SIT_GETUP_NEXT/PHASE_2_BRIEF.md`](02_SIT_GETUP_NEXT/PHASE_2_BRIEF.md) —
  second task, source video, GVHMR example, and K1 retarget example.
- [`03_ROBOT_AND_DEPLOYMENT_CONTEXT/README.md`](03_ROBOT_AND_DEPLOYMENT_CONTEXT/README.md) —
  our two robot variants and the low-level deployment issue encountered so
  far.
- [`03_ROBOT_AND_DEPLOYMENT_CONTEXT/WHAT_WE_TRIED.md`](03_ROBOT_AND_DEPLOYMENT_CONTEXT/WHAT_WE_TRIED.md) —
  concise train, evaluate, export, and attempted-deployment history.
- [`KICKOFF_QUESTIONS.md`](KICKOFF_QUESTIONS.md) — short list of decisions for
  the first working session.
- [`SHARE_SECURITY_REVIEW.md`](SHARE_SECURITY_REVIEW.md) — pre-share
  credential, metadata, and privacy review.
- [`GITHUB_REPOSITORY_NOTES.md`](GITHUB_REPOSITORY_NOTES.md) — private
  repository, Git LFS, and commit-safeguard notes.
- `04_MANIFEST/` — file inventory and SHA-256 checksums.

## Important context

We have two robots:

- Booster K1 Education with NVIDIA Jetson compute
- Booster K1 Geek with Qualcomm compute

Our current examples are references showing what we tried. They are not
finished behaviors and have not been validated as successful physical
deployments.

For the first phase, we would like to agree on one robot, one squat target,
one training workflow, and one safe deployment path. After the squat works
end-to-end, we will apply the same process to the sit/get-up motion.

## Requested collaboration output

For each behavior, we would like:

- Motion/reference data in the format used by the training workflow
- Training task, configuration, rewards, and curriculum
- Evaluation criteria and useful diagnostics
- Selected checkpoint and exported policy
- Deployment code compatible with the agreed K1 target
- Safe physical-test procedure and acceptance criteria
- Concise setup/run instructions in a shared repository
- A walkthrough so our team can retrain and extend the behavior

The included robot model is the stock 22-DOF K1 model. We did not add a custom
payload or attachment for either motion.
