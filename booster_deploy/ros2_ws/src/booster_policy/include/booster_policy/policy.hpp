#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace booster_policy {

// Walk and squat control all 22 K1 joints, including the head; the sit policy
// controls the other 20, and while seated the head follows the operator or
// looks around.
inline constexpr std::size_t kPolicyJointCount = 22;

// Walk policy layout: 50 frames of angular velocity (3), projected gravity (3),
// joint position offsets (22), joint velocities (22), and previous actions (22).
// Head position and velocity are zeroed in the observation.
inline constexpr std::size_t kWalkHistoryLength = 50;
inline constexpr std::size_t kWalkObservationSize = 72;
inline constexpr std::size_t kWalkCommandSize = 3;
inline constexpr float kMaxTranslationalSpeed = 1.5F;
inline constexpr float kMaxYawRate = 2.4F;

// Squat policy layout: the walk frame followed by a scalar squat command.
inline constexpr std::size_t kSquatObservationSize = 73;
inline constexpr float kSquatHeadActionScaleMultiplier = 0.1F;

// Sit policy layout: reference joint positions and velocities (44), anchor
// orientation error (6), angular velocity (3), joint position offsets (22),
// joint velocities (22), previous actions (20), and projected gravity (3).
inline constexpr std::size_t kSitObservationSize = 120;
inline constexpr std::size_t kSitActionCount = 20;
inline constexpr std::size_t kSitStateSize = 3;

enum class PolicyMode { kWalk, kSquat };
enum class ActivePolicy { kWalk, kSquat, kSit };
// Pose policy a walk-mode crouch command switches to.
enum class PosePolicy { kSquat, kSit };

// Joint arrays use the robot message order given by `joint_names`.
struct RobotConfig {
  std::vector<std::string> joint_names;
  std::vector<float> default_joint_pos;
  std::vector<float> joint_stiffness;
  std::vector<float> joint_damping;
};

struct PolicyConfig {
  // kWalk runs the joystick gait and switches to the squat or sit policy on
  // command; kSquat runs the squat policy by itself.
  PolicyMode mode = PolicyMode::kWalk;
  std::string walk_model_path;
  // Empty paths disable file-based gain overrides.
  std::string walk_gain_overrides_path;
  std::string squat_model_path;
  std::string squat_gain_overrides_path;
  // Walk mode only; empty leaves sit unavailable. Sit uses its ONNX gains.
  std::string sit_model_path;
  // Sit arm torque caps in Nm: each arm target stays within limit / stiffness
  // of the measured joint position. Hold applies while seated, move while
  // sitting down or standing up. Zero or less disables that cap. Real K1 arm
  // motors stop reporting after seconds near 10 Nm or minutes at 4-5 Nm.
  // The stand-up needs full arm torque; a 6 Nm move cap made it fall over.
  float sit_arm_hold_torque_limit = 2.0F;
  float sit_arm_move_torque_limit = 0.0F;
  // While seated, each ankle pitch (toes up and down) drifts on its own
  // between random poses within +/- this amplitude (rad): eased moves with
  // random pauses. It fades in once seated; a stand request fades it out and
  // waits about 0.3 s before standing. Zero disables it.
  float sit_ankle_wiggle_amplitude = 0.35F;
  // While seated, once the operator has left the head target alone for a few
  // seconds, the head looks around at random like the track-control gaze.
  bool sit_head_look_around = true;
  // Seconds between Step calls; times the seated head and ankle motion.
  float policy_dt = 0.02F;
  bool enable_safety_fallback = true;
  // Smallest upright gravity projection tolerated before faulting; 0.5 is
  // roughly 60 degrees of trunk tilt.
  float min_upright_projection = 0.5F;
  float standing_joint_pos_tolerance = 0.3F;
  // ONNX Runtime intra-op threads; 0 lets ONNX Runtime decide.
  int onnx_threads = 1;
  RobotConfig robot;
};

struct RobotState {
  std::array<float, 3> angular_velocity{};  // Base frame, rad/s.
  std::array<float, 3> projected_gravity{0.0F, 0.0F, -1.0F};
  // World-from-base (w, x, y, z); only the sit policy uses it.
  std::array<float, 4> root_quat{1.0F, 0.0F, 0.0F, 0.0F};
  std::vector<float> joint_pos;
  std::vector<float> joint_vel;
};

struct PolicyCommand {
  std::array<float, 3> velocity{};     // vx, vy, yaw rate.
  std::array<float, 2> head_target{};  // Yaw, pitch.
  // True switches to `pose` and crouches or sits; false stands up again.
  bool squat = false;
  PosePolicy pose = PosePolicy::kSquat;
};

struct JointCommand {
  std::vector<float> position;
  std::vector<float> stiffness;
  std::vector<float> damping;
};

// World-from-base (w, x, y, z) for IMU roll/pitch/yaw in the XYZ convention.
std::array<float, 4> QuaternionFromRpy(float roll, float pitch, float yaw);
// Gravity in the base frame for IMU roll/pitch/yaw in the XYZ convention.
std::array<float, 3> ProjectedGravityFromRpy(float roll, float pitch, float yaw);
// Gravity in the base frame for a (w, x, y, z) world-from-base quaternion.
std::array<float, 3> ProjectedGravityFromQuaternion(float w, float x, float y, float z);

// ONNX session with preallocated float or int64 input and output tensors.
class OnnxModel {
 public:
  OnnxModel(const std::string& path, const std::string& label, int threads);

  const std::string& label() const { return label_; }
  const std::map<std::string, std::string>& metadata() const { return metadata_; }
  const std::vector<std::string>& input_names() const { return input_names_; }
  const std::vector<std::string>& output_names() const { return output_names_; }
  const std::vector<std::vector<int64_t>>& input_shapes() const { return input_shapes_; }
  const std::vector<std::vector<int64_t>>& output_shapes() const { return output_shapes_; }
  const std::vector<ONNXTensorElementDataType>& input_types() const { return input_types_; }
  const std::vector<ONNXTensorElementDataType>& output_types() const { return output_types_; }

  // Allocates the input and output buffers once the shapes are validated.
  void BindBuffers();
  // Typed buffer access; throws if the tensor has a different element type.
  std::vector<float>& input(std::size_t index);
  const std::vector<float>& input(std::size_t index) const;
  const std::vector<float>& output(std::size_t index) const;
  std::vector<int64_t>& int64_input(std::size_t index);
  const std::vector<int64_t>& int64_input(std::size_t index) const;
  const std::vector<int64_t>& int64_output(std::size_t index) const;
  void Run();

 private:
  std::string label_;
  Ort::Session session_{nullptr};
  std::map<std::string, std::string> metadata_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;
  std::vector<ONNXTensorElementDataType> input_types_;
  std::vector<ONNXTensorElementDataType> output_types_;
  // One slot per tensor; only the vector matching its element type is used.
  std::vector<std::vector<float>> inputs_;
  std::vector<std::vector<float>> outputs_;
  std::vector<std::vector<int64_t>> int64_inputs_;
  std::vector<std::vector<int64_t>> int64_outputs_;
  std::vector<Ort::Value> input_values_;
  std::vector<Ort::Value> output_values_;
  std::vector<const char*> input_name_ptrs_;
  std::vector<const char*> output_name_ptrs_;
};

// Deployment wrapper for the binary-command squat ONNX.
class SquatPolicy {
 public:
  explicit SquatPolicy(const PolicyConfig& config);

  void Reset();
  const JointCommand& Step(const RobotState& state, bool squat);
  // Whether standing is commanded and the legs are back at their stance.
  bool IsStandingPose(const RobotState& state) const;
  bool upright_violation() const { return upright_violation_; }
  const JointCommand& gains() const { return command_; }

 private:
  void ValidateModel() const;
  void ValidateRobotConfig();

  const PolicyConfig& config_;
  OnnxModel model_;
  std::vector<std::size_t> policy_to_robot_;
  std::vector<std::size_t> standing_joint_indices_;
  std::vector<float> default_joint_pos_;
  std::vector<float> action_scale_;
  std::vector<float> last_action_;
  JointCommand command_;
  bool squat_commanded_ = false;
  bool upright_violation_ = false;
};

// Stateful deployment wrapper for the sit motion-tracking ONNX.
//
// The model carries its own trajectory state machine and returns the next
// reference frame on every call; deployment only feeds the returned state and
// reference back in. It has no orientation safety fallback.
class SitPolicy {
 public:
  explicit SitPolicy(const PolicyConfig& config);

  // Restores the standing state and the embedded frame-zero reference.
  void Reset();
  // `head_target` (yaw, pitch) is followed only while seated.
  const JointCommand& Step(const RobotState& state, bool sit,
                           const std::array<float, 2>& head_target);
  // Standing means the trajectory state is back at its [0, 0, 1] sentinel.
  bool IsStandingPose() const;
  const JointCommand& gains() const { return command_; }
  // Offsets (rad) last added to the left and right ankle-pitch targets.
  const std::array<float, 2>& ankle_offsets() const { return ankle_offset_; }

 private:
  // One foot's eased move from `from` to `to` (fractions of the amplitude),
  // followed by a pause.
  struct FootMotion {
    float pose = 0.0F;
    float from = 0.0F;
    float to = 0.0F;
    float elapsed = 0.0F;
    float duration = 0.0F;
    float hold = 0.0F;
  };

  void ValidateModel() const;
  void ValidateRobotConfig() const;
  // Feeds the returned state back in and latches the next reference frame.
  void LatchOutputs();
  // Sit requested and the trajectory in its seated loop.
  bool Seated(bool sit) const;
  void SteerHead(const RobotState& state, bool seated, const std::array<float, 2>& head_target);
  // Advances the look-around and returns its head goal (yaw, pitch).
  std::array<float, 2> LookAround();
  void WiggleAnkles(bool seated);
  void MoveFoot(FootMotion& foot);
  void LimitArmTorque(const RobotState& state, bool seated);
  float Uniform(float low, float high);

  const PolicyConfig& config_;
  OnnxModel model_;
  std::size_t anchor_index_ = 0;
  std::vector<std::size_t> policy_to_robot_;
  // Robot joint index of each shoulder and elbow joint.
  std::vector<std::size_t> arm_indices_;
  // Policy joint index of the left and right ankle-pitch joints.
  std::vector<std::size_t> ankle_pitch_indices_;
  std::mt19937 rng_;
  // Last commanded head (yaw, pitch); starts at the measured pose.
  std::array<float, 2> head_position_{};
  bool head_initialized_ = false;
  bool was_seated_ = false;
  // Operator head target last seen and seconds since it last changed.
  std::array<float, 2> operator_target_{};
  float operator_idle_ = 0.0F;
  // Current look-around gaze and how long to keep it once reached.
  std::array<float, 2> look_target_{};
  float look_hold_ = 0.0F;
  // Ankle wiggle fade (0..1), settle time still owed before a stand-up may
  // start, each foot's motion, and the offsets and rates last sent.
  float wiggle_envelope_ = 0.0F;
  float wiggle_settle_left_ = 0.0F;
  std::array<FootMotion, 2> feet_{};
  std::array<float, 2> ankle_offset_{};
  std::array<float, 2> ankle_offset_rate_{};
  // Policy joint index for each of the 20 action slots.
  std::vector<std::size_t> action_to_policy_;
  std::vector<std::size_t> head_indices_;
  std::vector<float> default_joint_pos_;
  std::vector<float> action_scale_;
  std::vector<float> last_action_;
  std::vector<float> ref_joint_pos_;
  std::vector<float> ref_joint_vel_;
  std::array<float, 4> ref_anchor_quat_{};
  std::array<float, 4> init_reference_yaw_inv_{};
  std::array<float, 4> init_root_yaw_inv_{};
  // The root heading is latched on the first step after a reset.
  bool root_yaw_initialized_ = false;
  JointCommand command_;
};

// History-encoder joystick gait with in-process squat and sit policies.
class WalkPolicy {
 public:
  explicit WalkPolicy(const PolicyConfig& config);

  void Reset();
  const JointCommand& Step(const RobotState& state, const PolicyCommand& command);

  ActivePolicy active_policy() const { return active_policy_; }
  bool squat_started() const { return squat_started_; }
  bool squat_complete() const { return squat_complete_; }
  bool IsStandingPose(const RobotState& state) const;
  bool upright_violation() const { return upright_violation_; }

  // Model inputs from the latest walk step, for diagnostics and tests.
  const std::vector<float>& history() const { return model_.input(0); }
  const std::vector<float>& command_input() const { return model_.input(1); }
  const JointCommand& walk_gains() const { return command_; }
  const JointCommand& squat_gains() const { return squat_.gains(); }
  bool has_sit() const { return sit_ != nullptr; }
  // Only available when a sit model is configured.
  const JointCommand& sit_gains() const;
  const std::array<float, 2>& sit_ankle_offsets() const;

 private:
  void ValidateModel() const;
  void ValidateRobotConfig() const;
  const JointCommand& WalkInference(const RobotState& state, const PolicyCommand& command);
  void StartPose(PosePolicy pose);
  void ResumeWalk();

  const PolicyConfig& config_;
  OnnxModel model_;
  SquatPolicy squat_;
  std::unique_ptr<SitPolicy> sit_;
  std::vector<std::size_t> policy_to_robot_;
  std::array<std::size_t, 2> head_indices_{};
  // Head joint positions in the model's joint order.
  std::array<std::size_t, 2> head_policy_indices_{};
  std::vector<float> default_joint_pos_;
  std::vector<float> action_scale_;
  std::vector<float> last_action_;
  std::array<float, kWalkObservationSize> observation_{};
  JointCommand command_;
  ActivePolicy active_policy_ = ActivePolicy::kWalk;
  bool history_initialized_ = false;
  bool squat_started_ = false;
  bool squat_complete_ = false;
  bool return_to_walk_ = false;
  bool upright_violation_ = false;
};

// Owns the configured policy and derives the squat lifecycle events the
// deployment workflow waits on.
class PolicyController {
 public:
  explicit PolicyController(PolicyConfig config);
  // The policies keep a reference to the owned config.
  PolicyController(const PolicyController&) = delete;
  PolicyController& operator=(const PolicyController&) = delete;

  void Reset();
  // Runs one policy step. After an upright violation the returned command is
  // still valid, but callers should stop publishing once they have sent it.
  const JointCommand& Step(const RobotState& state, const PolicyCommand& command);

  const PolicyConfig& config() const { return config_; }
  ActivePolicy active_policy() const;
  bool squat_started() const { return squat_started_; }
  bool standing_pose_complete() const { return standing_pose_complete_; }
  bool upright_fault() const { return upright_fault_; }
  // Only available in walk mode.
  const WalkPolicy& walk() const;

 private:
  void ValidateState(const RobotState& state) const;

  PolicyConfig config_;
  std::unique_ptr<WalkPolicy> walk_;
  std::unique_ptr<SquatPolicy> squat_;
  bool previous_squat_command_ = false;
  bool squat_started_ = false;
  bool standing_pose_complete_ = false;
  bool upright_fault_ = false;
};

}  // namespace booster_policy
