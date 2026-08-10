# Booster Walk and Squat Deploy

This repository deploys joystick-controlled learned walking and a
toggle-controlled squat policy on the Booster K1, either in MuJoCo or on a real
robot. Both policies remain loaded in one inference worker so switching does
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
press controller B (or keyboard `s`) to switch to the squat policy and crouch.
Press it again to stand; once the measured standing pose is restored, walking
resumes with freshly seeded history. In MuJoCo, the gait command stays zero and
keyboard `s` controls the same policy switch.

Policy inference runs in a worker process while ROS subscription and
publication remain in the parent process. Leaving WALK/CUSTOM stops low-level
inference and clears the publication handshake.

MuJoCo initializes the robot at `MujocoControllerCfg.init_pos` with the default
joint positions from the ONNX metadata.

## Walk ONNX contract

`tasks/walk/models/walk.onnx` takes a `[1, 50, 72]` `history` input and a
separate `[1, 3]` instantaneous velocity command. Each history frame contains
base angular velocity (3), projected gravity (3), joint positions relative to
the default pose (22), joint velocities (22), and the previous action (22).
Like Maelstrom's history-stacked gait wrapper, the first frame fills every
history slot; subsequent steps discard the oldest frame and append the newest.
Head position and velocity observations are masked to zero. Velocity commands
are constrained to a nonzero translational magnitude of `0.2–0.75 m/s` before
inference; an exact zero remains zero.

The output is one `[1, 22]` action tensor. Joint targets are
`default_joint_pos + action_scale * action`, with metadata providing the joint
order, defaults, gains, and action scales.

## Squat ONNX contract

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

## Gain overrides

Each ONNX model supplies its default deployment stiffness and damping. Task
specific overrides are loaded from the corresponding
`tasks/<policy>/gain_overrides.json` and applied by joint name. Gains switch
together with the active policy.

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

The tracked policy artifacts are `tasks/walk/models/walk.onnx` and
`tasks/squat/models/squat.onnx`. Their metadata is validated at startup and is
the source of truth for observation layout, joint order, default positions,
gains, and action scaling.
