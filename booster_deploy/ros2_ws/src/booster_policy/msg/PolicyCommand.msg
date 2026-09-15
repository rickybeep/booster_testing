# Latched operator command for the policy node. Publish it periodically: the
# velocity is zeroed when no command arrives within `command_timeout`.

# Base velocity command: forward (m/s), lateral (m/s), and yaw rate (rad/s).
float32 vx
float32 vy
float32 yaw_rate

# Head joint targets (rad), applied while the walk policy is active.
float32 head_yaw
float32 head_pitch

# True switches to the squat policy and crouches; false stands up again.
bool squat
