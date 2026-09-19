# Booster Walk, Squat and Sit Deploy

This repository deploys joystick-controlled learned walking and a
toggle-controlled squat and sit policies on the Booster K1, either in MuJoCo or
on a real robot. All policies remain loaded in one inference worker so switching does
not interrupt the low-level ROS publisher.

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

The default task is `walk`:

```bash
pixi run list-tasks
pixi run deploy-mujoco
pixi run deploy
```

Run `pixi run deploy` on the robot. MuJoCo deployment does not activate the ROS
workspace and can be run on a development machine.

`pixi run deploy --task squat` remains available for running the squat model by
itself.
Use `--webots` with `deploy` when the ROS topics are provided by Webots.

On the real robot, controller input arrives on `/remote_controller_state`.
Deployment observes the current high-level mode and remains inert in PREP,
DAMP, or an unknown mode. It also remains in firmware WALK until controller B
(or keyboard `s`) is pressed. That first press starts the learned walk policy
with a zero velocity command. Deployment requests CUSTOM only after the first
learned joint command has reached the ROS publisher, then stays in CUSTOM for
both learned policies.

The left stick commands forward/backward and lateral velocity; horizontal
movement of the right stick commands yaw. The left-stick translational vector
is zero inside a radial `0.1` stick dead zone. Outside it, nonzero translation
is constrained to `0.2–0.75 m/s`, including diagonal input. Yaw reaches
`1.5 rad/s` at full right-stick deflection. After learned walking is active,
press controller B (or keyboard `s`) to switch to the squat policy and crouch,
or controller X (or keyboard `m`) to switch to the sit policy. Press either
button again to stand; once the active policy reports standing, walking
resumes with a freshly reset policy state. In MuJoCo, the gait command stays
zero and keyboards `s`/`m` control the same policy switches.

While learned walking is active, the D-pad controls the head independently of
the gait: hold left/right for yaw and up/down for pitch. The target moves at
`0.8 rad/s`, remains latched when released, and is clamped to the K1 joint
limits (yaw `±1.0 rad`, pitch `-0.349–0.855 rad`). Positive pitch looks down.

Policy inference runs in a worker process while ROS subscription and
publication remain in the parent process. Leaving WALK/CUSTOM stops low-level
inference and clears the publication handshake.

MuJoCo initializes the robot at `MujocoControllerCfg.init_pos` with the default
joint positions from the ONNX metadata.

## Policy models

The gait and squat models are loaded from `tasks/walk/models/gait_history.onnx` and
`tasks/squat/models/squat.onnx`. Both return 22 joint actions, including the head.
The sit model is described separately below.
Deployment maps actions and gains by joint name to all 22 robot message slots,
then overrides head targets in slots 0–1 with manual D-pad control. Model metadata
supplies the joint order, default pose, gains, and action scales.

The gait model consumes `history` of shape `[1, 50, 72]`: 50 chronological frames
containing angular velocity (3), projected gravity (3), joint position offsets
(22), joint velocities (22), and previous model actions (22). Head state is
included. Velocity commands are supplied separately as `instant` of shape
`[1, 3]` and do not enter the history. After a reset, deployment fills the history
with the first frame. The model input clamps forward, backward, and lateral
velocity independently to `1.5 m/s`, and angular velocity to `2.5 rad/s`.

## Sit ONNX contract

`tasks/sit/models/sit.onnx` is a stateful motion-tracking policy with the same
contract as the kneel policy on the `kneeling` branch. Its inputs are `obs`
(`[1, 120]`), `squat_enabled`, and `squat_state_in` (`[1, 3]`); every call
returns actions for the 20 non-head joints, `squat_state_out`, and the next
reference arrays. Deployment feeds the returned state and references into the
next control step without interpreting the state machine, holds both head
joints at zero, and restores the standing state `[0, 0, 1]` plus the embedded
frame-zero reference each time the sit policy is selected. Standing means the
trajectory state has returned to `[0, 0, 1]`. The sit policy has no
orientation safety fallback and uses only the gains embedded in its ONNX
metadata.

## Gain overrides

Each ONNX model supplies its default deployment stiffness and damping. Task
specific overrides are loaded from the corresponding
`tasks/<policy>/gain_overrides.json` and applied by joint name. Gains switch
together with the active policy. The walk override raises both head joints from
the model's stiffness of `4.0` to `10.0` for firmer D-pad tracking; the squat
policy keeps its own gains.

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

The tracked policy artifacts are `tasks/walk/models/gait_history.onnx`,
`tasks/squat/models/squat.onnx`, and `tasks/sit/models/sit.onnx`. Their metadata is validated at startup and is
the source of truth for observation layout, joint order, default positions,
gains, and action scaling.
