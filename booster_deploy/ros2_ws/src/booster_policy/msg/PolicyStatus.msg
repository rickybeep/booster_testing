uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_FAULT=2

uint8 POLICY_WALK=0
uint8 POLICY_SQUAT=1
uint8 POLICY_SIT=2

builtin_interfaces/Time stamp

uint8 state
# Session granted by the most recent start request; 0 before the first start.
uint32 session
uint8 active_policy

# The session has published a joint command and `joint_ctrl` has a subscriber.
bool ready
bool squat_commanded
bool squat_started
bool standing_pose_complete
bool command_timed_out

# Largest absolute joint velocity in the latest /low_state, or NaN before one arrives.
float32 max_joint_speed
float32 inference_ms
uint64 step
string fault
