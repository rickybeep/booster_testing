# Latched operator command for the policy node. Publish it periodically: the
# velocity is zeroed when no command arrives within `command_timeout`.

# Base velocity command: forward (m/s), lateral (m/s), and yaw rate (rad/s).
float32 vx
float32 vy
float32 yaw_rate

# Head joint targets (rad), applied while the walk policy is active.
float32 head_yaw
float32 head_pitch

uint8 POSE_SQUAT=0
uint8 POSE_SIT=1

# True switches to the `pose_policy` policy and crouches or sits; false stands
# up again. The pose is latched until walking resumes.
bool squat
uint8 pose_policy
