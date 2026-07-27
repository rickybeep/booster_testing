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

See [`OUR_PROGRESS.md`](OUR_PROGRESS.md) for a short description of our prior
workflow and why we would like to restart from your proven K1 setup.

## Phase 1 collaboration target

1. Agree on squat style, depth, timing, and success criteria.
2. Put the reference into your preferred motion format.
3. Train and evaluate it using your existing K1 learned-motion workflow.
4. Review simulation video and quantitative results together.
5. Export the selected policy and adapt your deployment path to our chosen
   robot.
6. Conduct a coordinated, supported physical test.

The completed milestone should leave the runnable task, configuration,
checkpoint, export, deployment path, and short instructions in this
repository.
