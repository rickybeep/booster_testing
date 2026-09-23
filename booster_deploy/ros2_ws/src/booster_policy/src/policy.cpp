#include "booster_policy/policy.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace booster_policy {
namespace {

const std::array<const char*, 6> kExpectedObservations = {
    "base_ang_vel", "projected_gravity", "joint_pos", "joint_vel", "actions", "command",
};
const std::array<const char*, 5> kWalkObservations = {
    "base_ang_vel", "projected_gravity", "joint_pos", "joint_vel", "actions",
};
const std::unordered_map<std::string, std::string> kJointAliases = {
    {"Head_Yaw", "AAHead_yaw"},
    {"Head_Pitch", "Head_pitch"},
    {"Left_Shoulder_Pitch", "ALeft_Shoulder_Pitch"},
    {"Right_Shoulder_Pitch", "ARight_Shoulder_Pitch"},
};
const std::array<const char*, 2> kHeadJoints = {"Head_Yaw", "Head_Pitch"};
const std::array<const char*, 7> kSitObservations = {
    "command",    "motion_anchor_ori_b", "base_ang_vel", "joint_pos",
    "joint_vel",  "actions",             "projected_gravity",
};
const std::vector<std::string> kSitInputs = {"obs", "squat_enabled", "squat_state_in"};
const std::vector<std::string> kSitOutputs = {
    "actions",     "squat_state_out", "joint_pos",      "joint_vel",
    "body_pos_w",  "body_quat_w",     "body_lin_vel_w", "body_ang_vel_w",
};
const std::array<int64_t, kSitStateSize> kSitStandingState = {0, 0, 1};

// Squat depth shows up almost entirely in these joints: measured in MuJoCo
// they sit within 0.11 rad of the default pose while standing and 0.85 rad
// away at the bottom of a squat. Roll and ankle joints drift with stance and
// are a poor depth signal, so they are deliberately excluded.
const std::array<const char*, 2> kStandingJointPatterns = {"_Hip_Pitch", "_Knee_Pitch"};

Ort::Env& SharedEnv() {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "booster_policy");
  return env;
}

std::string Join(const std::vector<std::string>& values) {
  std::string joined;
  for (const auto& value : values) {
    joined += joined.empty() ? value : "," + value;
  }
  return joined;
}

std::string ShapeString(const std::vector<int64_t>& shape) {
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < shape.size(); ++i) {
    out << (i ? ", " : "") << shape[i];
  }
  out << "]";
  return out.str();
}

std::vector<std::string> Csv(const OnnxModel& model, const std::string& key) {
  const auto it = model.metadata().find(key);
  if (it == model.metadata().end()) {
    throw std::invalid_argument(model.label() + " ONNX metadata is missing '" + key + "'");
  }
  std::vector<std::string> values;
  std::size_t start = 0;
  while (true) {
    const std::size_t end = it->second.find(',', start);
    values.push_back(it->second.substr(start, end - start));
    if (end == std::string::npos) {
      return values;
    }
    start = end + 1;
  }
}

std::vector<float> FloatCsv(const OnnxModel& model, const std::string& key) {
  std::vector<float> values;
  for (const auto& text : Csv(model, key)) {
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    while (end != nullptr && std::isspace(static_cast<unsigned char>(*end))) {
      ++end;
    }
    if (text.empty() || end == text.c_str() || *end != '\0' || errno == ERANGE) {
      throw std::invalid_argument(
          model.label() + " ONNX metadata '" + key + "' has a non-numeric value: '" + text + "'");
    }
    values.push_back(static_cast<float>(value));
  }
  return values;
}

bool AllEqual(const std::vector<float>& values, const std::vector<float>& expected) {
  return values == expected;
}

std::string ResolveJointName(const std::string& policy_name) {
  const auto it = kJointAliases.find(policy_name);
  return it == kJointAliases.end() ? policy_name : it->second;
}

std::size_t RobotIndex(const RobotConfig& robot, const std::string& policy_name,
                       const std::string& label) {
  const std::string name = ResolveJointName(policy_name);
  const auto it = std::find(robot.joint_names.begin(), robot.joint_names.end(), name);
  if (it == robot.joint_names.end()) {
    throw std::invalid_argument(label + " ONNX joint '" + policy_name +
                                "' is not in the robot config");
  }
  return static_cast<std::size_t>(it - robot.joint_names.begin());
}

void CheckFinite(const std::vector<float>& values, const std::string& label,
                 const std::string& key) {
  if (values.size() != kPolicyJointCount ||
      !std::all_of(values.begin(), values.end(), [](float v) { return std::isfinite(v); })) {
    throw std::invalid_argument(label + " ONNX " + key + " must contain 22 finite values");
  }
}

bool DefaultPoseMatches(const std::vector<float>& policy_default, const RobotConfig& robot,
                        const std::vector<std::size_t>& policy_to_robot) {
  for (std::size_t i = 0; i < policy_default.size(); ++i) {
    if (std::abs(policy_default[i] - robot.default_joint_pos[policy_to_robot[i]]) > 1e-6F) {
      return false;
    }
  }
  return true;
}

void ApplyGainOverrides(const std::string& path, const RobotConfig& robot, JointCommand* gains) {
  if (path.empty()) {
    return;
  }
  std::ifstream file(path);
  if (!file) {
    throw std::invalid_argument("Gain overrides not found: " + path);
  }
  nlohmann::json overrides;
  try {
    file >> overrides;
  } catch (const nlohmann::json::exception& exc) {
    throw std::invalid_argument("Invalid gain overrides JSON " + path + ": " + exc.what());
  }
  if (!overrides.is_object()) {
    throw std::invalid_argument("Gain overrides must be a JSON object: " + path);
  }
  for (const auto& [section, joint_values] : overrides.items()) {
    std::vector<float>* target = nullptr;
    if (section == "stiffness") {
      target = &gains->stiffness;
    } else if (section == "damping") {
      target = &gains->damping;
    } else {
      throw std::invalid_argument("Unknown gain override section: " + section);
    }
    if (!joint_values.is_object()) {
      throw std::invalid_argument("Gain override '" + section + "' must be an object");
    }
    for (const auto& [joint_name, value] : joint_values.items()) {
      const auto it = std::find(robot.joint_names.begin(), robot.joint_names.end(), joint_name);
      if (it == robot.joint_names.end()) {
        throw std::invalid_argument("Unknown joint in " + section + " overrides: " + joint_name);
      }
      if (!value.is_number() || value.get<double>() < 0.0) {
        throw std::invalid_argument("Invalid " + section + " override for " + joint_name +
                                    ": " + value.dump());
      }
      (*target)[static_cast<std::size_t>(it - robot.joint_names.begin())] = value.get<float>();
    }
  }
}

// Isaac Lab quat_apply_inverse(q, (0, 0, -1)).
std::array<float, 3> RotateGravityInverse(float w, float x, float y, float z) {
  const std::array<float, 3> v = {0.0F, 0.0F, -1.0F};
  // t = 2 * cross(xyz, v)
  const std::array<float, 3> t = {
      2.0F * (y * v[2] - z * v[1]),
      2.0F * (z * v[0] - x * v[2]),
      2.0F * (x * v[1] - y * v[0]),
  };
  // v - w * t + cross(xyz, t)
  return {
      v[0] - w * t[0] + (y * t[2] - z * t[1]),
      v[1] - w * t[1] + (z * t[0] - x * t[2]),
      v[2] - w * t[2] + (x * t[1] - y * t[0]),
  };
}

using Quat = std::array<float, 4>;  // (w, x, y, z), Isaac Lab convention.

Quat QuatMul(const Quat& a, const Quat& b) {
  return {
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  };
}

// Isaac Lab quat_inv: conjugate divided by the squared norm.
Quat QuatInv(const Quat& q) {
  const float norm_sq = std::max(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3], 1e-9F);
  return {q[0] / norm_sq, -q[1] / norm_sq, -q[2] / norm_sq, -q[3] / norm_sq};
}

// Isaac Lab yaw_quat: the heading-only part of `q`.
Quat YawQuat(const Quat& q) {
  const float yaw = std::atan2(2.0F * (q[0] * q[3] + q[1] * q[2]),
                               1.0F - 2.0F * (q[2] * q[2] + q[3] * q[3]));
  return {std::cos(yaw * 0.5F), 0.0F, 0.0F, std::sin(yaw * 0.5F)};
}

// Isaac Lab matrix_from_quat, row-major.
std::array<float, 9> MatrixFromQuat(const Quat& q) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float two_s = 2.0F / (w * w + x * x + y * y + z * z);
  return {
      1.0F - two_s * (y * y + z * z), two_s * (x * y - z * w),        two_s * (x * z + y * w),
      two_s * (x * y + z * w),        1.0F - two_s * (x * x + z * z), two_s * (y * z - x * w),
      two_s * (x * z - y * w),        two_s * (y * z + x * w),        1.0F - two_s * (x * x + y * y),
  };
}

JointCommand MakeJointCommand(const RobotConfig& robot) {
  return JointCommand{robot.default_joint_pos, robot.joint_stiffness, robot.joint_damping};
}

}  // namespace

std::array<float, 4> QuaternionFromRpy(float roll, float pitch, float yaw) {
  // Isaac Lab quat_from_euler_xyz.
  const float cy = std::cos(yaw * 0.5F);
  const float sy = std::sin(yaw * 0.5F);
  const float cr = std::cos(roll * 0.5F);
  const float sr = std::sin(roll * 0.5F);
  const float cp = std::cos(pitch * 0.5F);
  const float sp = std::sin(pitch * 0.5F);
  return {cy * cr * cp + sy * sr * sp, cy * sr * cp - sy * cr * sp, cy * cr * sp + sy * sr * cp,
          sy * cr * cp - cy * sr * sp};
}

std::array<float, 3> ProjectedGravityFromRpy(float roll, float pitch, float yaw) {
  const auto q = QuaternionFromRpy(roll, pitch, yaw);
  return RotateGravityInverse(q[0], q[1], q[2], q[3]);
}

std::array<float, 3> ProjectedGravityFromQuaternion(float w, float x, float y, float z) {
  return RotateGravityInverse(w, x, y, z);
}

OnnxModel::OnnxModel(const std::string& path, const std::string& label, int threads)
    : label_(label) {
  if (!std::ifstream(path)) {
    throw std::invalid_argument(label + " policy not found: " + path);
  }
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(threads);
  options.SetInterOpNumThreads(1);
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  session_ = Ort::Session(SharedEnv(), path.c_str(), options);

  Ort::AllocatorWithDefaultOptions allocator;
  const Ort::ModelMetadata model_metadata = session_.GetModelMetadata();
  for (const auto& key : model_metadata.GetCustomMetadataMapKeysAllocated(allocator)) {
    metadata_[key.get()] =
        model_metadata.LookupCustomMetadataMapAllocated(key.get(), allocator).get();
  }
  for (std::size_t i = 0; i < session_.GetInputCount(); ++i) {
    input_names_.emplace_back(session_.GetInputNameAllocated(i, allocator).get());
    // The shape info views the type info, which must outlive it.
    const Ort::TypeInfo type_info = session_.GetInputTypeInfo(i);
    const auto info = type_info.GetTensorTypeAndShapeInfo();
    input_shapes_.push_back(info.GetShape());
    input_types_.push_back(info.GetElementType());
  }
  for (std::size_t i = 0; i < session_.GetOutputCount(); ++i) {
    output_names_.emplace_back(session_.GetOutputNameAllocated(i, allocator).get());
    // The shape info views the type info, which must outlive it.
    const Ort::TypeInfo type_info = session_.GetOutputTypeInfo(i);
    const auto info = type_info.GetTensorTypeAndShapeInfo();
    output_shapes_.push_back(info.GetShape());
    output_types_.push_back(info.GetElementType());
  }
}

void OnnxModel::BindBuffers() {
  const auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  const auto bind = [this, &memory_info](const std::vector<std::vector<int64_t>>& shapes,
                                         const std::vector<ONNXTensorElementDataType>& types,
                                         const std::vector<std::string>& names,
                                         std::vector<std::vector<float>>* buffers,
                                         std::vector<std::vector<int64_t>>* int64_buffers,
                                         std::vector<Ort::Value>* values) {
    for (std::size_t i = 0; i < shapes.size(); ++i) {
      std::size_t size = 1;
      for (const int64_t dim : shapes[i]) {
        size *= static_cast<std::size_t>(dim);
      }
      const bool is_int64 = types[i] == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64;
      if (!is_int64 && types[i] != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        throw std::invalid_argument(label_ + " ONNX tensor '" + names[i] +
                                    "' must be float or int64");
      }
      buffers->emplace_back(is_int64 ? 0 : size, 0.0F);
      int64_buffers->emplace_back(is_int64 ? size : 0, 0);
    }
    // Create the tensors after all buffers exist so their storage is stable.
    for (std::size_t i = 0; i < shapes.size(); ++i) {
      if (types[i] == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64) {
        values->push_back(Ort::Value::CreateTensor<int64_t>(
            memory_info, (*int64_buffers)[i].data(), (*int64_buffers)[i].size(),
            shapes[i].data(), shapes[i].size()));
      } else {
        values->push_back(Ort::Value::CreateTensor<float>(
            memory_info, (*buffers)[i].data(), (*buffers)[i].size(), shapes[i].data(),
            shapes[i].size()));
      }
    }
  };
  bind(input_shapes_, input_types_, input_names_, &inputs_, &int64_inputs_, &input_values_);
  bind(output_shapes_, output_types_, output_names_, &outputs_, &int64_outputs_,
       &output_values_);
  for (const auto& name : input_names_) {
    input_name_ptrs_.push_back(name.c_str());
  }
  for (const auto& name : output_names_) {
    output_name_ptrs_.push_back(name.c_str());
  }
}

namespace {

template <typename Buffers>
auto& TypedBuffer(Buffers& buffers, const std::vector<ONNXTensorElementDataType>& types,
                  std::size_t index, ONNXTensorElementDataType expected,
                  const std::string& label) {
  if (types.at(index) != expected) {
    throw std::logic_error(label + " ONNX tensor " + std::to_string(index) +
                           " accessed with the wrong element type");
  }
  return buffers.at(index);
}

}  // namespace

std::vector<float>& OnnxModel::input(std::size_t index) {
  return TypedBuffer(inputs_, input_types_, index, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, label_);
}
const std::vector<float>& OnnxModel::input(std::size_t index) const {
  return TypedBuffer(inputs_, input_types_, index, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, label_);
}
const std::vector<float>& OnnxModel::output(std::size_t index) const {
  return TypedBuffer(outputs_, output_types_, index, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, label_);
}
std::vector<int64_t>& OnnxModel::int64_input(std::size_t index) {
  return TypedBuffer(int64_inputs_, input_types_, index, ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64,
                     label_);
}
const std::vector<int64_t>& OnnxModel::int64_input(std::size_t index) const {
  return TypedBuffer(int64_inputs_, input_types_, index, ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64,
                     label_);
}
const std::vector<int64_t>& OnnxModel::int64_output(std::size_t index) const {
  return TypedBuffer(int64_outputs_, output_types_, index, ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64,
                     label_);
}

void OnnxModel::Run() {
  session_.Run(Ort::RunOptions{nullptr}, input_name_ptrs_.data(), input_values_.data(),
               input_values_.size(), output_name_ptrs_.data(), output_values_.data(),
               output_values_.size());
}

SquatPolicy::SquatPolicy(const PolicyConfig& config)
    : config_(config), model_(config.squat_model_path, "Squat", config.onnx_threads) {
  ValidateModel();
  model_.BindBuffers();

  const auto joint_names = Csv(model_, "joint_names");
  default_joint_pos_ = FloatCsv(model_, "default_joint_pos");
  action_scale_ = FloatCsv(model_, "action_scale");
  if (action_scale_.size() != kPolicyJointCount) {
    throw std::invalid_argument("Squat ONNX action scale must contain 22 values");
  }
  for (const char* head_joint : kHeadJoints) {
    const auto it = std::find(joint_names.begin(), joint_names.end(), head_joint);
    if (it == joint_names.end()) {
      throw std::invalid_argument(std::string("Squat ONNX is missing joint ") + head_joint);
    }
    action_scale_[static_cast<std::size_t>(it - joint_names.begin())] *=
        kSquatHeadActionScaleMultiplier;
  }
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    policy_to_robot_.push_back(RobotIndex(config_.robot, joint_names[i], "Squat"));
    for (const char* pattern : kStandingJointPatterns) {
      if (joint_names[i].find(pattern) != std::string::npos) {
        standing_joint_indices_.push_back(i);
        break;
      }
    }
  }
  ValidateRobotConfig();

  // The policy artifact owns its deployment gains. Applying them here prevents
  // stale robot config values from changing policy behavior.
  command_ = MakeJointCommand(config_.robot);
  command_.stiffness = FloatCsv(model_, "joint_stiffness");
  command_.damping = FloatCsv(model_, "joint_damping");
  CheckFinite(command_.stiffness, "Squat", "joint_stiffness");
  CheckFinite(command_.damping, "Squat", "joint_damping");
  ApplyGainOverrides(config_.squat_gain_overrides_path, config_.robot, &command_);
  Reset();
}

void SquatPolicy::ValidateModel() const {
  const std::vector<std::string> expected_inputs = {"obs"};
  const std::vector<std::string> expected_outputs = {"actions"};
  if (model_.input_names() != expected_inputs) {
    throw std::invalid_argument("Unexpected squat ONNX inputs: " + Join(model_.input_names()) +
                                "; expected obs");
  }
  if (model_.output_names() != expected_outputs) {
    throw std::invalid_argument("Unexpected squat ONNX outputs: " +
                                Join(model_.output_names()) + "; expected actions");
  }
  const std::vector<int64_t> obs_shape = {1, kSquatObservationSize};
  if (model_.input_shapes()[0] != obs_shape) {
    throw std::invalid_argument("Squat ONNX obs must be [1, 73], got " +
                                ShapeString(model_.input_shapes()[0]));
  }
  const std::vector<int64_t> action_shape = {1, kPolicyJointCount};
  if (model_.output_shapes()[0] != action_shape) {
    throw std::invalid_argument("Squat ONNX actions must be [1, 22], got " +
                                ShapeString(model_.output_shapes()[0]));
  }
  const auto observations = Csv(model_, "observation_names");
  if (observations != std::vector<std::string>(kExpectedObservations.begin(),
                                               kExpectedObservations.end())) {
    throw std::invalid_argument("Unexpected squat observation layout: " + Join(observations));
  }
  const auto commands = Csv(model_, "command_names");
  if (commands != std::vector<std::string>{"squat"}) {
    throw std::invalid_argument("Unexpected squat command layout: " + Join(commands));
  }

  // This wrapper builds raw, unscaled, history-free observations. Anything else
  // in the artifact would silently change what the policy sees.
  const auto scales = FloatCsv(model_, "observation_terms_scale");
  if (!std::all_of(scales.begin(), scales.end(), [](float v) { return v == 1.0F; })) {
    throw std::invalid_argument("Squat ONNX expects observation scaling");
  }
  const auto history = FloatCsv(model_, "observation_terms_history_length");
  if (!std::all_of(history.begin(), history.end(), [](float v) { return v == 0.0F; })) {
    throw std::invalid_argument("Squat ONNX expects observation history");
  }
  const auto clips = Csv(model_, "observation_terms_clip");
  if (!std::all_of(clips.begin(), clips.end(), [](const auto& v) { return v == "-inf;inf"; })) {
    throw std::invalid_argument("Squat ONNX expects observation clipping: " + Join(clips));
  }
  if (Csv(model_, "joint_names").size() != config_.robot.joint_names.size()) {
    throw std::invalid_argument("Squat ONNX joint count does not match the robot");
  }
}

void SquatPolicy::ValidateRobotConfig() {
  for (std::size_t i = 0; i < policy_to_robot_.size(); ++i) {
    if (policy_to_robot_[i] != i) {
      throw std::invalid_argument("Squat ONNX joint order does not match the K1 config");
    }
  }
  if (!DefaultPoseMatches(default_joint_pos_, config_.robot, policy_to_robot_)) {
    throw std::invalid_argument("K1 default_joint_pos does not match squat ONNX metadata");
  }
}

void SquatPolicy::Reset() {
  last_action_.assign(kPolicyJointCount, 0.0F);
  squat_commanded_ = false;
  upright_violation_ = false;
}

const JointCommand& SquatPolicy::Step(const RobotState& state, bool squat) {
  squat_commanded_ = squat;
  float* obs = model_.input(0).data();
  std::copy(state.angular_velocity.begin(), state.angular_velocity.end(), obs);
  std::copy(state.projected_gravity.begin(), state.projected_gravity.end(), obs + 3);
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    obs[6 + i] = state.joint_pos[policy_to_robot_[i]] - default_joint_pos_[i];
    obs[28 + i] = state.joint_vel[policy_to_robot_[i]];
    obs[50 + i] = last_action_[i];
  }
  obs[72] = squat ? 1.0F : 0.0F;

  model_.Run();
  const std::vector<float>& action = model_.output(0);

  // Projected gravity z is -1 when upright and rises toward zero as the trunk
  // tips over.
  upright_violation_ = config_.enable_safety_fallback &&
                       -state.projected_gravity[2] < config_.min_upright_projection;

  std::copy(action.begin(), action.end(), last_action_.begin());
  command_.position = config_.robot.default_joint_pos;
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    command_.position[policy_to_robot_[i]] = default_joint_pos_[i] + action_scale_[i] * action[i];
  }
  return command_;
}

bool SquatPolicy::IsStandingPose(const RobotState& state) const {
  if (squat_commanded_) {
    return false;
  }
  float max_error = 0.0F;
  for (const std::size_t i : standing_joint_indices_) {
    max_error =
        std::max(max_error, std::abs(state.joint_pos[policy_to_robot_[i]] - default_joint_pos_[i]));
  }
  return max_error <= config_.standing_joint_pos_tolerance;
}

SitPolicy::SitPolicy(const PolicyConfig& config)
    : config_(config), model_(config.sit_model_path, "Sit", config.onnx_threads) {
  ValidateModel();
  model_.BindBuffers();

  const auto joint_names = Csv(model_, "joint_names");
  const auto body_names = Csv(model_, "body_names");
  const auto anchor = model_.metadata().find("anchor_body_name");
  if (anchor == model_.metadata().end()) {
    throw std::invalid_argument("Sit ONNX metadata is missing 'anchor_body_name'");
  }
  const auto anchor_it = std::find(body_names.begin(), body_names.end(), anchor->second);
  if (anchor_it == body_names.end()) {
    throw std::invalid_argument("Sit ONNX anchor body '" + anchor->second +
                                "' is not in body_names");
  }
  anchor_index_ = static_cast<std::size_t>(anchor_it - body_names.begin());
  default_joint_pos_ = FloatCsv(model_, "default_joint_pos");
  action_scale_ = FloatCsv(model_, "action_scale");
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    policy_to_robot_.push_back(RobotIndex(config_.robot, joint_names[i], "Sit"));
    const bool head = std::find_if(kHeadJoints.begin(), kHeadJoints.end(), [&](const char* name) {
                        return joint_names[i] == name;
                      }) != kHeadJoints.end();
    (head ? head_indices_ : action_to_policy_).push_back(i);
  }
  ValidateRobotConfig();

  // The policy artifact owns its deployment gains; sit has no file overrides.
  command_ = MakeJointCommand(config_.robot);
  command_.stiffness = FloatCsv(model_, "joint_stiffness");
  command_.damping = FloatCsv(model_, "joint_damping");
  CheckFinite(command_.stiffness, "Sit", "joint_stiffness");
  CheckFinite(command_.damping, "Sit", "joint_damping");
  Reset();
}

void SitPolicy::ValidateModel() const {
  if (model_.input_names() != kSitInputs) {
    throw std::invalid_argument("Unexpected sit ONNX inputs: " + Join(model_.input_names()) +
                                "; expected " + Join(kSitInputs));
  }
  if (model_.output_names() != kSitOutputs) {
    throw std::invalid_argument("Unexpected sit ONNX outputs: " + Join(model_.output_names()) +
                                "; expected " + Join(kSitOutputs));
  }
  const auto expect_shape = [](const std::vector<int64_t>& actual,
                                   const std::vector<int64_t>& expected, const char* name) {
    if (actual != expected) {
      throw std::invalid_argument(std::string("Sit ONNX ") + name + " must be " +
                                  ShapeString(expected) + ", got " + ShapeString(actual));
    }
  };
  const auto body_count = static_cast<int64_t>(Csv(model_, "body_names").size());
  const auto joints = static_cast<int64_t>(kPolicyJointCount);
  const auto state_size = static_cast<int64_t>(kSitStateSize);
  expect_shape(model_.input_shapes()[0], {1, kSitObservationSize}, "obs");
  expect_shape(model_.input_shapes()[1], {1, 1}, "squat_enabled");
  expect_shape(model_.input_shapes()[2], {1, state_size}, "squat_state_in");
  expect_shape(model_.output_shapes()[0], {1, kSitActionCount}, "actions");
  expect_shape(model_.output_shapes()[1], {1, state_size}, "squat_state_out");
  expect_shape(model_.output_shapes()[2], {1, joints}, "joint_pos");
  expect_shape(model_.output_shapes()[3], {1, joints}, "joint_vel");
  expect_shape(model_.output_shapes()[5], {1, body_count, 4}, "body_quat_w");
  if (model_.input_types()[2] != ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64 ||
      model_.output_types()[1] != ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64) {
    throw std::invalid_argument("Sit ONNX trajectory state must be int64");
  }
  if (Csv(model_, "observation_names") !=
      std::vector<std::string>(kSitObservations.begin(), kSitObservations.end())) {
    throw std::invalid_argument("Unexpected sit observation layout: " +
                                Join(Csv(model_, "observation_names")));
  }
  if (Csv(model_, "joint_names").size() != kPolicyJointCount) {
    throw std::invalid_argument("Sit ONNX joint count does not match the robot");
  }
}

void SitPolicy::ValidateRobotConfig() const {
  for (std::size_t i = 0; i < policy_to_robot_.size(); ++i) {
    if (policy_to_robot_[i] != i) {
      throw std::invalid_argument("Sit ONNX joint order does not match the K1 config");
    }
  }
  if (!DefaultPoseMatches(default_joint_pos_, config_.robot, policy_to_robot_)) {
    throw std::invalid_argument("K1 default_joint_pos does not match sit ONNX metadata");
  }
  if (action_scale_.size() != kSitActionCount || action_to_policy_.size() != kSitActionCount) {
    throw std::invalid_argument("Sit ONNX action scale must match the 20 non-head joints");
  }
}

void SitPolicy::LatchOutputs() {
  const auto& next_state = model_.int64_output(1);
  std::copy(next_state.begin(), next_state.end(), model_.int64_input(2).begin());
  ref_joint_pos_ = model_.output(2);
  ref_joint_vel_ = model_.output(3);
  const auto& body_quat = model_.output(5);
  std::copy_n(body_quat.begin() + static_cast<std::ptrdiff_t>(4 * anchor_index_), 4,
              ref_anchor_quat_.begin());
}

void SitPolicy::Reset() {
  last_action_.assign(kSitActionCount, 0.0F);
  root_yaw_initialized_ = false;

  // A disabled standing call returns the exact frame-zero reference embedded
  // in the model. Its action is deliberately discarded.
  std::fill(model_.input(0).begin(), model_.input(0).end(), 0.0F);
  model_.input(1)[0] = 0.0F;
  std::copy(kSitStandingState.begin(), kSitStandingState.end(), model_.int64_input(2).begin());
  model_.Run();
  LatchOutputs();
  init_reference_yaw_inv_ = QuatInv(YawQuat(ref_anchor_quat_));
}

const JointCommand& SitPolicy::Step(const RobotState& state, bool sit) {
  if (!root_yaw_initialized_) {
    init_root_yaw_inv_ = QuatInv(YawQuat(state.root_quat));
    root_yaw_initialized_ = true;
  }
  // Orientation of the reference anchor in the current root frame, with both
  // headings measured from where they were when the policy was selected.
  const Quat current = QuatMul(init_root_yaw_inv_, state.root_quat);
  const Quat reference = QuatMul(init_reference_yaw_inv_, ref_anchor_quat_);
  const auto rotation = MatrixFromQuat(QuatMul(QuatInv(current), reference));

  float* obs = model_.input(0).data();
  std::copy(ref_joint_pos_.begin(), ref_joint_pos_.end(), obs);
  std::copy(ref_joint_vel_.begin(), ref_joint_vel_.end(), obs + 22);
  // First two columns of the rotation matrix, row by row.
  for (std::size_t row = 0; row < 3; ++row) {
    obs[44 + 2 * row] = rotation[3 * row];
    obs[45 + 2 * row] = rotation[3 * row + 1];
  }
  std::copy(state.angular_velocity.begin(), state.angular_velocity.end(), obs + 50);
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    obs[53 + i] = state.joint_pos[policy_to_robot_[i]] - default_joint_pos_[i];
    obs[75 + i] = state.joint_vel[policy_to_robot_[i]];
  }
  std::copy(last_action_.begin(), last_action_.end(), obs + 97);
  std::copy(state.projected_gravity.begin(), state.projected_gravity.end(), obs + 117);
  // The ONNX input keeps its training name; it enables sitting.
  model_.input(1)[0] = sit ? 1.0F : 0.0F;

  model_.Run();
  const std::vector<float>& action = model_.output(0);
  std::copy(action.begin(), action.end(), last_action_.begin());
  LatchOutputs();

  command_.position = config_.robot.default_joint_pos;
  for (std::size_t a = 0; a < kSitActionCount; ++a) {
    const std::size_t i = action_to_policy_[a];
    command_.position[policy_to_robot_[i]] = default_joint_pos_[i] + action_scale_[a] * action[a];
  }
  for (const std::size_t i : head_indices_) {
    command_.position[policy_to_robot_[i]] = 0.0F;
  }
  return command_;
}

bool SitPolicy::IsStandingPose() const {
  const auto& current = model_.int64_input(2);
  return std::equal(current.begin(), current.end(), kSitStandingState.begin());
}

WalkPolicy::WalkPolicy(const PolicyConfig& config)
    : config_(config),
      model_(config.walk_model_path, "Walk", config.onnx_threads),
      squat_(config) {
  if (!config_.sit_model_path.empty()) {
    sit_ = std::make_unique<SitPolicy>(config_);
  }
  ValidateModel();
  model_.BindBuffers();

  const auto joint_names = Csv(model_, "joint_names");
  default_joint_pos_ = FloatCsv(model_, "default_joint_pos");
  action_scale_ = FloatCsv(model_, "action_scale");
  for (const auto& name : joint_names) {
    policy_to_robot_.push_back(RobotIndex(config_.robot, name, "Walk"));
  }
  for (std::size_t i = 0; i < kHeadJoints.size(); ++i) {
    head_indices_[i] = RobotIndex(config_.robot, kHeadJoints[i], "Walk");
    const auto it = std::find(joint_names.begin(), joint_names.end(), kHeadJoints[i]);
    if (it == joint_names.end()) {
      throw std::invalid_argument(std::string("Walk ONNX is missing head joint ") + kHeadJoints[i]);
    }
    head_policy_indices_[i] = static_cast<std::size_t>(it - joint_names.begin());
  }
  ValidateRobotConfig();

  // The publisher indexes full robot arrays, including the two head slots.
  command_ = MakeJointCommand(config_.robot);
  const auto stiffness = FloatCsv(model_, "joint_stiffness");
  const auto damping = FloatCsv(model_, "joint_damping");
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    command_.stiffness[policy_to_robot_[i]] = stiffness[i];
    command_.damping[policy_to_robot_[i]] = damping[i];
  }
  ApplyGainOverrides(config_.walk_gain_overrides_path, config_.robot, &command_);
  Reset();
}

void WalkPolicy::ValidateModel() const {
  if (model_.input_names() != std::vector<std::string>{"history", "instant"}) {
    throw std::invalid_argument("Unexpected walk ONNX inputs: " + Join(model_.input_names()) +
                                "; expected history,instant");
  }
  if (model_.output_names() != std::vector<std::string>{"actions"}) {
    throw std::invalid_argument("Unexpected walk ONNX outputs: " + Join(model_.output_names()) +
                                "; expected actions");
  }
  const std::vector<int64_t> action_shape = {1, kPolicyJointCount};
  if (model_.output_shapes()[0] != action_shape) {
    throw std::invalid_argument("Walk ONNX actions must be [1, 22], got " +
                                ShapeString(model_.output_shapes()[0]));
  }
  const std::vector<int64_t> history_shape = {1, kWalkHistoryLength, kWalkObservationSize};
  if (model_.input_shapes()[0] != history_shape) {
    throw std::invalid_argument("Walk ONNX history must be [1, 50, 72], got " +
                                ShapeString(model_.input_shapes()[0]));
  }
  const std::vector<int64_t> instant_shape = {1, kWalkCommandSize};
  if (model_.input_shapes()[1] != instant_shape) {
    throw std::invalid_argument("Walk ONNX instant must be [1, 3], got " +
                                ShapeString(model_.input_shapes()[1]));
  }
  if (Csv(model_, "observation_names") !=
      std::vector<std::string>(kWalkObservations.begin(), kWalkObservations.end())) {
    throw std::invalid_argument("Unexpected walk observation layout");
  }
  if (Csv(model_, "command_names") != std::vector<std::string>{"twist"}) {
    throw std::invalid_argument("Unexpected walk command layout");
  }
  if (!AllEqual(FloatCsv(model_, "observation_terms_scale"), std::vector<float>(5, 1.0F))) {
    throw std::invalid_argument("Walk ONNX expects unscaled observations");
  }
  // Commands are fed through `instant`; every state term has the full history.
  const std::vector<float> expected_history(5, kWalkHistoryLength);
  if (!AllEqual(FloatCsv(model_, "observation_terms_history_length"), expected_history)) {
    throw std::invalid_argument("Unexpected walk history layout");
  }
  const std::vector<float> expected_flatten(5, 0.0F);
  if (!AllEqual(FloatCsv(model_, "observation_terms_flatten_history_dim"), expected_flatten)) {
    throw std::invalid_argument("Unexpected walk observation flattening");
  }
  const auto clips = Csv(model_, "observation_terms_clip");
  if (clips != std::vector<std::string>(5, "-inf;inf")) {
    throw std::invalid_argument("Walk ONNX expects observation clipping: " + Join(clips));
  }
  if (Csv(model_, "joint_names").size() != kPolicyJointCount) {
    throw std::invalid_argument("Walk ONNX must control 22 joints");
  }
  const auto excluded = model_.metadata().find("excluded_joint_names");
  if (excluded != model_.metadata().end() && !excluded->second.empty()) {
    throw std::invalid_argument("Walk ONNX must include all robot joints");
  }
  for (const char* key : {"default_joint_pos", "action_scale", "joint_stiffness", "joint_damping"}) {
    CheckFinite(FloatCsv(model_, key), "Walk", key);
  }
}

void WalkPolicy::ValidateRobotConfig() const {
  const std::set<std::size_t> unique(policy_to_robot_.begin(), policy_to_robot_.end());
  if (unique.size() != kPolicyJointCount ||
      config_.robot.joint_names.size() != kPolicyJointCount) {
    throw std::invalid_argument("Walk ONNX joints do not match the K1 config");
  }
  if (!DefaultPoseMatches(default_joint_pos_, config_.robot, policy_to_robot_)) {
    throw std::invalid_argument("K1 default_joint_pos does not match walk ONNX metadata");
  }
}

void WalkPolicy::Reset() {
  last_action_.assign(kPolicyJointCount, 0.0F);
  history_initialized_ = false;
  active_policy_ = ActivePolicy::kWalk;
  squat_started_ = false;
  squat_complete_ = false;
  return_to_walk_ = false;
  upright_violation_ = false;
  squat_.Reset();
  if (sit_) {
    sit_->Reset();
  }
}

const JointCommand& WalkPolicy::WalkInference(const RobotState& state,
                                              const PolicyCommand& command) {
  float* obs = observation_.data();
  std::copy(state.angular_velocity.begin(), state.angular_velocity.end(), obs);
  std::copy(state.projected_gravity.begin(), state.projected_gravity.end(), obs + 3);
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    obs[6 + i] = state.joint_pos[policy_to_robot_[i]] - default_joint_pos_[i];
    obs[28 + i] = state.joint_vel[policy_to_robot_[i]];
    obs[50 + i] = last_action_[i];
  }
  // The gait was trained with the head state masked; the head slots stay in the
  // action vector but are overridden by the manual head target below.
  for (const std::size_t i : head_policy_indices_) {
    obs[6 + i] = 0.0F;
    obs[28 + i] = 0.0F;
  }

  // History frames are chronological; after a reset every frame is the first
  // observation.
  float* history = model_.input(0).data();
  const std::size_t frame_bytes = sizeof(float) * kWalkObservationSize;
  if (history_initialized_) {
    std::memmove(history, history + kWalkObservationSize, frame_bytes * (kWalkHistoryLength - 1));
    std::memcpy(history + kWalkObservationSize * (kWalkHistoryLength - 1), obs, frame_bytes);
  } else {
    for (std::size_t frame = 0; frame < kWalkHistoryLength; ++frame) {
      std::memcpy(history + kWalkObservationSize * frame, obs, frame_bytes);
    }
    history_initialized_ = true;
  }

  float* instant = model_.input(1).data();
  instant[0] = std::clamp(command.velocity[0], -kMaxTranslationalSpeed, kMaxTranslationalSpeed);
  instant[1] = std::clamp(command.velocity[1], -kMaxTranslationalSpeed, kMaxTranslationalSpeed);
  instant[2] = std::clamp(command.velocity[2], -kMaxYawRate, kMaxYawRate);

  model_.Run();
  const std::vector<float>& action = model_.output(0);

  upright_violation_ = config_.enable_safety_fallback &&
                       -state.projected_gravity[2] < config_.min_upright_projection;

  std::copy(action.begin(), action.end(), last_action_.begin());
  command_.position = config_.robot.default_joint_pos;
  for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
    command_.position[policy_to_robot_[i]] = default_joint_pos_[i] + action_scale_[i] * action[i];
  }
  command_.position[head_indices_[0]] = command.head_target[0];
  command_.position[head_indices_[1]] = command.head_target[1];
  return command_;
}

void WalkPolicy::StartPose(PosePolicy pose) {
  if (pose == PosePolicy::kSit) {
    if (!sit_) {
      throw std::invalid_argument("Sit was commanded but no sit model is configured");
    }
    active_policy_ = ActivePolicy::kSit;
    sit_->Reset();
  } else {
    active_policy_ = ActivePolicy::kSquat;
    squat_.Reset();
  }
  squat_started_ = false;
  squat_complete_ = false;
  return_to_walk_ = false;
}

void WalkPolicy::ResumeWalk() {
  active_policy_ = ActivePolicy::kWalk;
  std::fill(last_action_.begin(), last_action_.end(), 0.0F);
  history_initialized_ = false;
  return_to_walk_ = false;
  squat_started_ = false;
}

const JointCommand& WalkPolicy::Step(const RobotState& state, const PolicyCommand& command) {
  if (return_to_walk_) {
    ResumeWalk();
  }
  if (active_policy_ == ActivePolicy::kWalk) {
    if (!command.squat) {
      return WalkInference(state, command);
    }
    // The pose is latched until standing completes and walking resumes.
    StartPose(command.pose);
  }

  const JointCommand* targets = nullptr;
  if (active_policy_ == ActivePolicy::kSit) {
    targets = &sit_->Step(state, command.squat);
    upright_violation_ = false;
  } else {
    targets = &squat_.Step(state, command.squat);
    upright_violation_ = squat_.upright_violation();
  }
  const bool standing = IsStandingPose(state);
  if (!standing) {
    squat_started_ = true;
  }
  if (!command.squat && standing) {
    squat_complete_ = true;
    return_to_walk_ = true;
  }
  return *targets;
}

bool WalkPolicy::IsStandingPose(const RobotState& state) const {
  switch (active_policy_) {
    case ActivePolicy::kWalk:
      return true;
    case ActivePolicy::kSit:
      return sit_->IsStandingPose();
    case ActivePolicy::kSquat:
      break;
  }
  return squat_.IsStandingPose(state);
}

const JointCommand& WalkPolicy::sit_gains() const {
  if (!sit_) {
    throw std::logic_error("No sit model is configured");
  }
  return sit_->gains();
}

PolicyController::PolicyController(PolicyConfig config) : config_(std::move(config)) {
  const auto& robot = config_.robot;
  const std::size_t joints = robot.joint_names.size();
  if (joints != kPolicyJointCount || robot.default_joint_pos.size() != joints ||
      robot.joint_stiffness.size() != joints || robot.joint_damping.size() != joints) {
    throw std::invalid_argument(
        "Robot config must provide 22 joint names, default positions, stiffness, and damping");
  }
  if (config_.mode == PolicyMode::kWalk) {
    walk_ = std::make_unique<WalkPolicy>(config_);
  } else {
    squat_ = std::make_unique<SquatPolicy>(config_);
  }
  Reset();
}

void PolicyController::Reset() {
  if (walk_) {
    walk_->Reset();
  } else {
    squat_->Reset();
  }
  previous_squat_command_ = false;
  squat_started_ = false;
  standing_pose_complete_ = false;
  upright_fault_ = false;
}

void PolicyController::ValidateState(const RobotState& state) const {
  if (state.joint_pos.size() != kPolicyJointCount || state.joint_vel.size() != kPolicyJointCount) {
    throw std::invalid_argument("Robot state must contain 22 joint positions and velocities");
  }
}

const JointCommand& PolicyController::Step(const RobotState& state,
                                           const PolicyCommand& command) {
  ValidateState(state);
  // A new crouch request starts a fresh squat cycle.
  if (command.squat && !previous_squat_command_) {
    squat_started_ = false;
    standing_pose_complete_ = false;
  }
  previous_squat_command_ = command.squat;

  bool policy_started = false;
  bool policy_complete = false;
  bool standing = false;
  const JointCommand* targets = nullptr;
  if (walk_) {
    targets = &walk_->Step(state, command);
    policy_started = walk_->squat_started();
    policy_complete = walk_->squat_complete();
    standing = walk_->IsStandingPose(state);
    upright_fault_ = upright_fault_ || walk_->upright_violation();
  } else {
    targets = &squat_->Step(state, command.squat);
    standing = squat_->IsStandingPose(state);
    upright_fault_ = upright_fault_ || squat_->upright_violation();
  }

  if (policy_complete) {
    standing_pose_complete_ = true;
  } else if (policy_started || (command.squat && !standing)) {
    squat_started_ = true;
    standing_pose_complete_ = false;
  } else if (!command.squat && squat_started_ && standing) {
    standing_pose_complete_ = true;
  }
  return *targets;
}

ActivePolicy PolicyController::active_policy() const {
  return walk_ ? walk_->active_policy() : ActivePolicy::kSquat;
}

const WalkPolicy& PolicyController::walk() const {
  if (!walk_) {
    throw std::logic_error("The walk policy is only available in walk mode");
  }
  return *walk_;
}

}  // namespace booster_policy
