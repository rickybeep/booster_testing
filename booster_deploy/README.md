# Booster Walk, Squat and Sit Deploy

This repository deploys joystick-controlled learned walking and
toggle-controlled squat and sit policies on the Booster K1, either in MuJoCo or
on a real robot. Policy inference runs in C++ with ONNX Runtime; joystick
handling, the firmware-mode workflow, and the policy control API are Python.
All policies remain loaded in one C++ node so switching does not interrupt the low-level ROS
publisher.

## Install

[Pixi](https://pixi.sh) manages the environment on Linux x86-64, Linux ARM64,
and macOS ARM64:

```bash
pixi install --locked
```

MuJoCo also needs the robot data checkout. The upstream Python wheel does not
contain the XML and mesh assets, so point deployment at the checkout itself:

```bash
git clone https://github.com/BoosterRobotics/booster_assets ../booster_assets
export BOOSTER_ASSETS_DIR="$(realpath ../booster_assets)"
```

Deployment uses the repository's Pixi-managed `ros` environment. ROS 2 Humble
comes from RoboStack. The `booster_interface` messages and the C++
`booster_policy` package are built locally from `ros2_ws/src`:

```bash
pixi run ros-build
```

`scripts/ros-env.sh` sources only the resulting local
`ros2_ws/install/setup.bash`; it never sources `/opt/ros` or the robot's system
Python environment. The `deploy`, `deploy-mujoco`, `policy-node`, and `test`
tasks depend on `ros-build`, so a normal launch builds the workspace
automatically.

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

Run `pixi run deploy` on the robot. MuJoCo deployment runs the same C++ policy
core in-process through its Python bindings and can be run on a development
machine.

`pixi run deploy --task squat` remains available for running the squat model by
itself.
Use `--webots` with `deploy` when the ROS topics are provided by Webots.

On the real robot, controller input arrives on `/remote_controller_state`.
Deployment observes the current high-level mode and remains inert in PREP,
DAMP, or an unknown mode. It also remains in firmware WALK until controller B
(or keyboard `s`) is pressed. That first press starts the learned walk policy
with a zero velocity command. Deployment requests CUSTOM only after the first
learned joint command has reached the ROS publisher, then stays in CUSTOM for
all learned policies.

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

Leaving WALK/CUSTOM stops low-level inference and clears the publication
handshake. On exit, including Ctrl-C and policy faults, deployment requests
WALKING before stopping the policy node.

MuJoCo initializes the robot at `MujocoControllerCfg.init_pos` with the default
joint positions from the ONNX metadata.

## Architecture

`pixi run deploy` starts two processes:

- **`booster_policy` (C++)**, from `ros2_ws/src/booster_policy`. It subscribes to
  `/low_state`, runs the walk, squat and sit ONNX models on a dedicated control
  thread at `policy_dt` (50 Hz), and publishes `joint_ctrl`. It is controlled
  through `/booster_policy/command` (`PolicyCommand`), the
  `/booster_policy/start` (`StartPolicy`) and `/booster_policy/stop`
  (`std_srvs/Trigger`) services, and reports on `/booster_policy/status`
  (`PolicyStatus`). If commands stop arriving for `command_timeout` (0.5 s),
  it zeroes the velocity and keeps balancing. A trunk tilt beyond
  `min_upright_projection` faults the session and stops publishing.
- **The Python joystick node** (`BoosterRobotPortal`). It reads
  `/remote_controller_state`, runs the walk/squat/sit workflow, switches firmware
  modes with `booster-sdk`, and forwards commands to the policy node at the
  policy rate.

`deploy.py` launches the node with parameters generated from the task's
`PolicyCfg` and robot config, and stops it on exit. It runs in its own process
session, so terminal Ctrl-C reaches only the Python process, which leaves
CUSTOM mode first. With `--webots`, the node advances one policy step per
`policy_dt / low_state_dt` `/low_state` messages instead of using the wall
clock.

The C++ core (`include/booster_policy/policy.hpp`) has no ROS dependency. It
is also built as the `booster_policy_core` Python module, which MuJoCo and
the tests use.

## Python API

`booster_deploy.policy_client.PolicyClient` starts, stops, and commands the
policy node from any Python process. Run the node on its own, without the
joystick node:

```bash
pixi run policy-node            # or: pixi run policy-node --task squat
```

Then command it:

```python
from booster_deploy.policy_client import PolicyClient

with PolicyClient() as policy:  # creates and spins its own ROS node
    policy.start()              # blocks until joint commands are published
    policy.set_velocity(0.3, 0.0, 0.0)  # vx, vy (m/s), yaw rate (rad/s)
    policy.set_head_target(0.2, 0.0)    # yaw, pitch (rad)
    policy.squat()              # switch to the squat policy and crouch
    policy.stand()              # stand, then resume walking
    policy.sit()                # switch to the sit policy and sit down
    policy.stand()              # stand, then resume walking
    print(policy.status)        # PolicyStatus for this session
```

The client republishes its latched command every 20 ms. The robot must be put
in CUSTOM mode, for example with `booster_sdk`'s `BoosterClient`, before the
published joint commands take effect, and returned to WALKING before
`stop()`. Only one client should command the node at a time; `pixi run deploy`
already runs the joystick client.

## Policy models

The gait and squat models are loaded from `tasks/walk/models/gait.onnx` and
`tasks/squat/models/squat.onnx`. Both return 22 joint actions, including the head.
The sit model is described separately below.
Deployment maps actions and gains by joint name to all 22 robot message slots,
then overrides head targets in slots 0–1 with manual D-pad control. Model metadata
supplies the joint order, default pose, gains, and action scales.

The gait model consumes `history` of shape `[1, 50, 72]`: 50 chronological frames
containing angular velocity (3), projected gravity (3), joint position offsets
(22), joint velocities (22), and previous model actions (22). Head position and
velocity are zeroed. Velocity commands are supplied separately as `instant` of shape
`[1, 3]` and do not enter the history. After a reset, deployment fills the history
with the first frame. The model input clamps forward, backward, and lateral
velocity independently to `1.5 m/s`, and angular velocity to `2.4 rad/s`.

## Sit ONNX contract

`tasks/sit/models/sit.onnx` is booster-lab's SuperMimic motion tracker
(`Mjlab-SuperMimic-Flat-Beeple-K1`, fine-tuned on the Beeple K1 with an arm
torque penalty), and `tasks/sit/models/sit.motion.json` beside it is the sit
clip it plays (`kneel-sitdown-tracking:v2`). The ankle damping in the
metadata is the crank-motor kd, since the firmware applies kd per motor.

The model takes `actor_obs` (`[1, 720]`), 10 chronological frames of joint
position offsets, joint velocities, angular velocity, projected gravity and
previous actions (all 22 joints), and `goal` (`[1, 570]`), the next 10
reference frames: the reference trunk orientation in the robot trunk frame
(first two rotation-matrix columns), its heading-frame linear velocity,
trunk-frame angular velocity, height, joint position offsets and joint
velocities. It returns `actions` for all 22 joints. After a reset the history
is filled with the first frame and the previous action holds the clip's first
frame, as in training.

Deployment plays the clip with booster-lab's toggle-tracking timing: a sit
request plays it forward into the seated loop (`loop_start_frame` to
`loop_end_frame`), which repeats while sit is held; releasing sit plays the
stand-up, which ends holding the last frame. Releasing sit while sitting down
plays back toward standing, and asking again while standing up plays back
into the loop; played back, the goal's reference rates are reversed. Each sit
aligns the clip's heading with the robot's IMU heading as it starts. Standing
means the playhead is in its standing phase. While seated the head follows
the operator or looks around, the ankles wiggle (hidden from the
observation), and the seated arm torque cap applies; otherwise the head
follows the policy's own target at the head rate. The sit policy has no
orientation safety fallback and uses only the gains embedded in its ONNX
metadata. Set `sit_checkpoint_path` to `None` to leave it unloaded; commanding
sit then faults the session.

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

Set `walk_gain_overrides_path` or `squat_gain_overrides_path` to `None` in the
task's `PolicyCfg` to disable file-based overrides.

## Development

```bash
pixi run lint
pixi run ros-build
pixi run test
```

`tests/test_policy_core.py` checks the C++ core against ONNX Runtime in Python.
`tests/test_policy_node.py` launches the real node with a fake `/low_state`
publisher and drives it through `PolicyClient`.

The tracked policy artifacts are `tasks/walk/models/gait.onnx`,
`tasks/squat/models/squat.onnx`, and `tasks/sit/models/sit.onnx` (with its clip
`tasks/sit/models/sit.motion.json`). Their metadata is validated at startup and is
the source of truth for observation layout, joint order, default positions,
gains, and action scaling.
