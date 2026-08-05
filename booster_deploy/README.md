# Booster Squat Deploy

This repository deploys the toggle-controlled Booster K1 squat policy in
MuJoCo or on a real robot. The ONNX model owns the reversible squat state
machine; deployment only persists its opaque state and supplies the operator's
enabled/disabled command.

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

On the real robot, controller input arrives on
`/remote_controller_state`. Deployment runs a `py_trees` workflow that observes
the current high-level robot mode. In PREP, DAMP, or an unknown mode it sends no
joint commands and requests no mode changes. While in WALK, press controller B
(or keyboard `s`) to start the policy and enter CUSTOM mode for a crouch. Press
it again to stand. The workflow returns the firmware to WALK only after the
ONNX trajectory reports its standing sentinel and measured joint positions and
velocities have remained within the configured standing tolerances for five
workflow ticks. In MuJoCo, keyboard `s` retains the original immediate toggle
behavior.

The standing gate defaults to a maximum joint-position error of `0.20` rad and
a maximum joint speed of `0.25` rad/s. These values and the five-tick settling
window are configured by `BoosterRobotControllerCfg`.

MuJoCo initializes the robot directly from the model's embedded frame-zero
root pose, orientation, and joint positions.

## Stateful ONNX contract

The model inputs are `obs`, `squat_enabled`, and `squat_state_in`. Every call
returns actions, `squat_state_out`, and the next reference arrays. Deployment
feeds the returned state and references into the next control step without
interpreting the state machine. Startup and policy reset restore disabled
standing state `[0, 0, 1]` and the embedded frame-zero reference.

The policy observation intentionally omits trunk translation and base linear
velocity, so the same 122-value observation is constructed from signals
available in both MuJoCo and on the real robot.

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
file-based overrides.

## Development

```bash
pixi run lint
pixi run ros-build
```

The tracked policy artifact is `tasks/squat/models/squat.onnx`. Its metadata is
validated at startup and is the source of truth for observation layout, joint
order, default positions, gains, and action scaling.
