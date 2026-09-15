#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace booster_policy {

// Both policies control all 22 K1 joints, including the head.
inline constexpr std::size_t kPolicyJointCount = 22;

// Walk policy layout: 50 frames of angular velocity (3), projected gravity (3),
// joint position offsets (22), joint velocities (22), and previous actions (22).
inline constexpr std::size_t kWalkHistoryLength = 50;
inline constexpr std::size_t kWalkObservationSize = 72;
inline constexpr std::size_t kWalkCommandSize = 3;
inline constexpr float kMaxTranslationalSpeed = 1.5F;
inline constexpr float kMaxYawRate = 2.5F;

// Squat policy layout: the walk frame followed by a scalar squat command.
inline constexpr std::size_t kSquatObservationSize = 73;
inline constexpr float kSquatHeadActionScaleMultiplier = 0.1F;

enum class PolicyMode { kWalk, kSquat };
enum class ActivePolicy { kWalk, kSquat };

// Joint arrays use the robot message order given by `joint_names`.
struct RobotConfig {
  std::vector<std::string> joint_names;
  std::vector<float> default_joint_pos;
  std::vector<float> joint_stiffness;
  std::vector<float> joint_damping;
};

struct PolicyConfig {
  // kWalk runs the joystick gait and switches to the squat policy on command;
  // kSquat runs the squat policy by itself.
  PolicyMode mode = PolicyMode::kWalk;
  std::string walk_model_path;
  // Empty paths disable file-based gain overrides.
  std::string walk_gain_overrides_path;
  std::string squat_model_path;
  std::string squat_gain_overrides_path;
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
  std::vector<float> joint_pos;
  std::vector<float> joint_vel;
};

struct PolicyCommand {
  std::array<float, 3> velocity{};     // vx, vy, yaw rate.
  std::array<float, 2> head_target{};  // Yaw, pitch.
  bool squat = false;
};

struct JointCommand {
  std::vector<float> position;
  std::vector<float> stiffness;
  std::vector<float> damping;
};

// Gravity in the base frame for IMU roll/pitch/yaw in the XYZ convention.
std::array<float, 3> ProjectedGravityFromRpy(float roll, float pitch, float yaw);
// Gravity in the base frame for a (w, x, y, z) world-from-base quaternion.
std::array<float, 3> ProjectedGravityFromQuaternion(float w, float x, float y, float z);

// ONNX session with preallocated float input and output tensors.
class OnnxModel {
 public:
  OnnxModel(const std::string& path, const std::string& label, int threads);

  const std::string& label() const { return label_; }
  const std::map<std::string, std::string>& metadata() const { return metadata_; }
  const std::vector<std::string>& input_names() const { return input_names_; }
  const std::vector<std::string>& output_names() const { return output_names_; }
  const std::vector<std::vector<int64_t>>& input_shapes() const { return input_shapes_; }
  const std::vector<std::vector<int64_t>>& output_shapes() const { return output_shapes_; }

  // Allocates the input and output buffers once the shapes are validated.
  void BindBuffers();
  std::vector<float>& input(std::size_t index) { return inputs_.at(index); }
  const std::vector<float>& input(std::size_t index) const { return inputs_.at(index); }
  const std::vector<float>& output(std::size_t index) const { return outputs_.at(index); }
  void Run();

 private:
  std::string label_;
  Ort::Session session_{nullptr};
  std::map<std::string, std::string> metadata_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;
  std::vector<std::vector<float>> inputs_;
  std::vector<std::vector<float>> outputs_;
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

// History-encoder joystick gait with an in-process squat policy.
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

 private:
  void ValidateModel() const;
  void ValidateRobotConfig() const;
  const JointCommand& WalkInference(const RobotState& state, const PolicyCommand& command);
  void StartSquat();
  void ResumeWalk();

  const PolicyConfig& config_;
  OnnxModel model_;
  SquatPolicy squat_;
  std::vector<std::size_t> policy_to_robot_;
  std::array<std::size_t, 2> head_indices_{};
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
