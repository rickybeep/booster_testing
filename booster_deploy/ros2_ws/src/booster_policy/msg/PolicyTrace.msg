# Observation only; never subscribe to joint_ctrl for recording, because its
# subscriber count participates in controller readiness.
builtin_interfaces/Time stamp
uint32 session
uint64 step
uint64 low_state_sequence
bool command_published
float32 inference_ms
uint8 active_policy
bool standing_pose_complete
bool command_timed_out
string fault
# Exact low-state message from which this inference step's RobotState was made.
booster_interface/LowState measured
# Output is valid when output_valid; only transmitted when command_published.
booster_interface/LowCmd command

# v2 diagnostics: source clock is steady_clock on this robot, not PC time.
bool output_valid
uint64 source_steady_ns
float64 low_state_age_s
float32 stand_stable_s
bool requested_sit
uint8 requested_pose
float32[] velocity_command
float32[] head_target
# Empty outside SIT. Exact ONNX input/output and pre-override robot targets.
float32[] sit_observation
int64[] sit_state_in
int64[] sit_state_out
float32[] sit_raw_actions
float32[] sit_raw_targets
float32[] sit_reference_q
float32[] sit_reference_dq
float32[] sit_reference_quat
float32[] sit_next_reference_q
float32[] sit_next_reference_dq
float32[] sit_body_pos
float32[] sit_body_quat
float32[] sit_body_lin_vel
float32[] sit_body_ang_vel
float32 sit_enabled
float32 sit_arm_limit
float32 sit_orientation_error_rad
