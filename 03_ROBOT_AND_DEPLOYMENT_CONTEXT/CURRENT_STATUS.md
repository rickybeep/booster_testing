# Current status

## Confirmed

- The physical robot inspected in the recorded event is a K1 Education,
  model version 1.2.1, running firmware
  `1.7.0.7-release-00040-2026-07-02-global`.
- K1 feedback and the firmware CUSTOM consumer expect exactly 22 joints.
- The squat CSV has 701 keyframes; conversion produces 700 sampled frames at
  50 Hz, 22 joints, and 23 tracked bodies.
- `model_1198.pt` is a valid deterministic 119-observation/22-action policy.
- Its TorchScript export has identical actor and normalizer tensors and exact
  outputs on the seeded equivalence probe.
- The corrected current deployment configuration passes 16/20 randomized
  MuJoCo trials. It passes controlled delay through 30 ms and fails at 35 ms.
- Changing only `kp`/`kd` to the values recorded during training produces
  20/20 in the same offline test and passes the controlled delay checks
  through 80 ms.
- The physical hold canary entered CUSTOM, but firmware repeatedly failed to
  obtain a command snapshot and reported `expected 22, got 0`.
- No physical squat-policy execution has been accepted or verified.

## Strong hypotheses

- The raw Python DDS writer did not have a compatible matched firmware reader.
  Candidate reasons include SDK/runtime provenance, QoS/type incompatibility,
  DDS profile selection, interface selection, or discovery.
- The mismatch between training gains and the original deployment gains is the
  dominant source of the policy's simulated delay cliff.
- The most reliable deployment route for the inspected Education firmware is
  the installed Booster ROS 2 interface with a parent-owned, RELIABLE,
  continuously publishing `joint_ctrl` writer.

## Unknown

- Whether the Education firmware reader matches the exact official ROS 2
  writer when checked onboard without publishing.
- Whether firmware accepts a complete 22-entry sample after that match.
- Which gains and effort units are physically correct for either robot.
- Actual command latency and tail distribution.
- The Geek robot's exact firmware, edition metadata, ROS interface, DDS
  profile, SDK/runtime, and compatibility with K1 Pro deployment code.
- Physical behavior of this squat checkpoint.

## Current recommendation

Use the K1 Education as the first integration target because it has the
strongest evidence trail. First settle the non-actuating writer/subscriber
match. Then validate a current-position hold with firmware-side acceptance.
Only after transport, gain calibration, latency, limits, and watchdogs are
known should the squat policy be considered.
