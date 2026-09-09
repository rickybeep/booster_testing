# Booster Squat Deploy

This repository deploys the toggle-controlled Booster K1 squat policy in
MuJoCo or on a real robot. The primary ONNX model is a plain feed-forward
policy: deployment builds its observation, appends the operator's binary squat
command, and applies the returned joint position offsets.

A second, kneel (motion-tracking) policy (`tasks/squat/kneel.py`,
`models/kneel.onnx`) is available as an alternate. On the real robot the
controller button pressed selects which policy runs a crouch cycle; in MuJoCo
it is chosen with `--policy kneel`.

## Install

[Pixi](https://pixi.sh) manages the Python environment on Linux x86-64 and
ARM64:

```bash
pixi install --locked
```

MuJoCo also needs the robot data checkout. The upstream Python wheel does not
contain the XML and mesh assets, so point deployment at the checkout itself:

```bash
git clone https://github.com/BoosterRobotics/booster_assets ../booster_assets
export BOOSTER_ASSETS_DIR="$(realpath ../booster_assets)"
```

Real-robot control uses the repository's Pixi-managed `ros` environment. ROS 2
Humble comes from RoboStack, and the required `booster_interface` messages are
built locally from `ros2_ws/src/booster_interface`:

```bash
pixi run ros-build
```

`scripts/ros-env.sh` sources only the resulting local
`ros2_ws/install/setup.bash`; it never sources `/opt/ros` or the robot's system
Python environment. The `deploy` task depends on `ros-build`, so a normal launch
builds the local interface automatically.

The high-level mode client is IntelligentRoboticsLab's
[`booster-sdk`](https://github.com/IntelligentRoboticsLab/booster_sdk), pinned
to version `0.1.2-alpha.2` and installed by Pixi from PyPI.

## Run

The sole task is `squat`, so it is selected when `--task` is omitted:

```bash
pixi run list-tasks
pixi run deploy-mujoco
pixi run deploy
```

Run `pixi run deploy` on the robot. MuJoCo deployment does not activate the ROS
workspace and can be run on a development machine.

`pixi run deploy --task squat` remains available for explicit selection.
Use `--webots` with `deploy` when the ROS topics are provided by Webots.
In MuJoCo, `pixi run deploy-mujoco --policy kneel` runs the
kneel policy instead of the default squat policy; `--list` prints the
policy names each task offers.

On the real robot, controller input arrives on
`/remote_controller_state`. Deployment runs a `py_trees` workflow that observes
the current high-level robot mode. In PREP, DAMP, or an unknown mode it sends no
joint commands and requests no mode changes. While in WALK, press controller B
(or keyboard `s`) to start the squat policy, or controller X (or keyboard `m`)
to start the kneel policy; either enters CUSTOM mode for a crouch.
The policy publishes its standing command during the mode transition and begins
crouching only after the SDK confirms CUSTOM. Press either button again to
stand. The workflow returns the firmware to WALK only after the policy reports
standing and measured joint velocities have remained below the configured
settling tolerance for five workflow ticks. For the squat policy, standing means
the measured hip pitch and knee pitch joints are back within
`standing_joint_pos_tolerance` of the stance; for the kneel policy it
means the ONNX trajectory state has returned to its standing sentinel.
In MuJoCo, keyboard `s`/`m` retain the original immediate toggle behavior for
whichever policy was selected at launch.

The settling gate defaults to a maximum joint speed of `0.25` rad/s. This value
and the five-tick settling window are configured by
`BoosterRobotControllerCfg`.

Policy inference runs in a replaceable worker process, but ROS publication
remains in the parent process. Each new crouch clears the prior command,
action-ready handshake, completion flags, and shared action buffer before
starting a freshly reset policy worker. This keeps repeated crouches from
reusing middleware or policy state from the previous cycle.

MuJoCo initializes the robot at `MujocoControllerCfg.init_pos` with the default
joint positions from the ONNX metadata.

## ONNX contract

The model takes a single `[1, 73]` `obs` input and returns a single `[1, 22]`
`actions` output. The observation is, in order, base angular velocity (3),
projected gravity (3), joint positions relative to the default pose (22), joint
velocities (22), the previous action (22), and the binary squat command (1).
Actions are joint position offsets: targets are
`default_joint_pos + action_scale * action`. Deployment scales the two head
joints down to 10% of the trained action scale; the head does not contribute to
balance and the full range is unnecessarily lively on hardware.

The observation intentionally omits trunk translation and base linear velocity,
so the same vector is built from signals available in both MuJoCo and on the
real robot. Startup and policy reset only clear the previous action.

The policy carries no internal trajectory, so the safety fallback compares the
measured trunk orientation against vertical instead of against a reference pose.
It stops the policy when the upright gravity projection drops below
`min_upright_projection` (`0.5`, roughly 60 degrees of tilt).

## Kneel ONNX contract

`models/kneel.onnx` is stateful. Its inputs are `obs` (`[1, 120]`),
`squat_enabled`, and `squat_state_in` (`[1, 3]`); every call returns actions
for the 20 non-head joints, `squat_state_out`, and the next reference arrays.
Deployment feeds the returned state and references into the next control step
without interpreting the state machine, holds both head joints at zero, and
restores the standing state `[0, 0, 1]` plus the embedded frame-zero reference
on reset. MuJoCo initializes the robot from that embedded reference and draws
it as a ghost. This policy has no orientation safety fallback; it was ported
as tested on the `sitdown-testing` branch.

## Gain overrides

The ONNX metadata supplies the default deployment stiffness and damping. Task
specific overrides are loaded from `tasks/squat/gain_overrides.json` and are
applied by joint name on top of those defaults. The included override sets both
ankle pitch and roll damping values to `2.0` on each leg.

Either section is optional:

```json
{
  "stiffness": {
    "Left_Ankle_Pitch": 40.0
  },
  "damping": {
    "Left_Ankle_Pitch": 2.0
  }
}
```

Set `gain_overrides_path` to `null` in the task policy configuration to disable
file-based overrides. The kneel policy applies no file overrides; it uses the
gains embedded in its ONNX metadata.

## Development

```bash
pixi run lint
pixi run ros-build
```

The tracked policy artifacts are `tasks/squat/models/squat.onnx` and
`tasks/squat/models/kneel.onnx`. Their metadata is validated at startup and
is the source of truth for observation layout, joint order, default positions,
gains, and action scaling.
