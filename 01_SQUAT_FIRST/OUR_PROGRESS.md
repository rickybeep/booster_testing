# Our squat progress so far

## Reference motion

We generated a procedural 14-second squat from the stock K1 geometry:

- 1 second standing
- 3 seconds descending
- 5 seconds at the bottom
- 3 seconds rising
- 2 seconds standing
- 1.4-radian knee target
- Minimum-jerk interpolation at 50 Hz

The CSV was converted into a 22-joint motion reference for our
BeyondMimic-style experiments.

## Training and evaluation

We used Booster BeyondMimic, Isaac Lab, RSL-RL, and MuJoCo. Our work included
short smoke tests, longer training runs, resumed runs, simulation playback,
policy export, and basic robustness experiments.

The result looked plausible in simulation, but our physical deployment path
was not reliable enough to validate the behavior on the robot. We therefore
do not consider the prior policy complete or ready for reuse.

## What is included here

The `example/` folder contains only:

- The procedural squat CSV
- The converted NPZ reference
- One representative simulation replay

It does not contain the old policy checkpoint, training run directories,
training code, deployment code, or exports. The examples are intended to
communicate the motion and give us a concrete starting point.

## What we would like to do together

We would like to rebuild the squat using your established K1 training and
deployment workflow, understand the important design choices, and leave a
clean reproducible implementation in this repository.
