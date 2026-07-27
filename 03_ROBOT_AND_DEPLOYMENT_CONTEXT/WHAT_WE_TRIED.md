# What we trained and how we attempted deployment

This is a concise history for technical orientation. It is not a claim that
the squat worked on physical hardware.

## 1. Motion preparation

We created a procedural 14-second K1 squat from the stock K1 geometry:

- 1 second standing
- 3 seconds descending
- 5 seconds at the bottom
- 3 seconds rising
- 2 seconds standing
- 1.4-radian knee target
- Minimum-jerk interpolation at 50 Hz

The generator adjusted the trunk while keeping the ankles fixed in the
sagittal plane. The resulting CSV was converted through the Booster
BeyondMimic motion pipeline into a 700-sample NPZ with 22 joints and 23
tracked bodies.

## 2. Training

We used the Booster BeyondMimic task with Isaac Lab and RSL-RL. Work progressed
from one-iteration smoke tests to baseline runs, resumed runs, and controlled
latency experiments.

The retained sequence comprises nine run directories. The selected final run
continued from the earlier policy and trained through checkpoint 1198 with a
10–30 ms delay curriculum:

`2026-07-10_17-16-29_cliff_10_30ms_resume_799_to_1200`

The policy contract was 119 observations to 22 joint actions. The selected
checkpoint was exported to TorchScript and ONNX.

See
[`../01_SQUAT_FIRST/TRAINING_AND_TEST_TIMELINE.md`](../01_SQUAT_FIRST/TRAINING_AND_TEST_TIMELINE.md)
and
[`../01_SQUAT_FIRST/SELECTED_RUN_SUMMARY.md`](../01_SQUAT_FIRST/SELECTED_RUN_SUMMARY.md).

## 3. Offline evaluation

The evaluation used both Isaac Lab playback and a MuJoCo robustness harness.
An early MuJoCo result was invalidated after discovering that initialization
reset root height and shoulder state; the test was corrected and rerun.

With the deployment gains then configured:

- 16 of 20 randomized MuJoCo trials passed.
- Controlled replay passed through 30 ms delay.
- It failed at 35 and 40 ms.

Using the gains recorded during training produced 20 of 20 passes in the same
offline harness and survived the controlled checks through 80 ms. This was an
offline comparison only; it did not establish physically correct gains or
effort units.

Fresh regression verification on 2026-07-27 produced:

```text
164 passed, 40 warnings in 4.75s
```

The warnings were TorchScript deprecation warnings. The hardware-enable gates
remained false throughout the offline audit.

## 4. First attempted hardware handoff

Before executing the policy, we used a current-position full-body hold canary.
The raw Python probe:

- Received K1 LowState.
- Created 22 `MotorCmd` entries.
- Published through the B1-style LowCmd writer at approximately 500 Hz.
- Requested the PREP-to-CUSTOM transition.

The robot entered CUSTOM but became effectively limp. Firmware then logged
607 failures to obtain a custom-command snapshot and reported:

```text
User motor command size mismatch, expected 22, got 0
```

The robot was returned to PREP. The squat checkpoint was never executed during
this event.

Receiving LowState and completing a local `Write()` call therefore did not
prove that the firmware had a compatible matched command reader.

## 5. What the independent audit found

The raw path was built around an older, unversioned B1-oriented Python SDK. It
used BEST_EFFORT publication, exposed no Python subscription-match count, and
sent PARALLEL ankle coordinates. The official K1 ROS deployment source used
RELIABLE depth-1 QoS and SERIAL joint coordinates.

The official ROS deployment source also needed corrections around publisher
process ownership, continuous safe publication, state freshness, direct
subscription matching, and safety gates. Those corrections were implemented
and unit-tested only in the isolated duplicate; they were not deployed.

See [`DEPLOYMENT_PATH_NOTES.md`](DEPLOYMENT_PATH_NOTES.md).

## 6. What we want to do differently with your workflow

For Phase 1, we would like to use your proven K1 train-and-deploy path and make
firmware-side command acceptance an explicit prerequisite:

1. Confirm the exact target robot, SDK/interface, DDS profile, QoS, joint
   ordering, coordinate representation, and gains.
2. Prove a matched firmware command subscriber without publishing.
3. Review the motion and train a clean squat policy.
4. Review simulation results and export the selected policy.
5. With separate approval, validate a current-position hold and require
   firmware-side evidence of a complete 22-command sample.
6. Only then attempt the learned squat.

The same end-to-end workflow can then be adapted to the sit/get-up behavior.
