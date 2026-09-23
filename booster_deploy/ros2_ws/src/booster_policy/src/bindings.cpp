#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "booster_policy/policy.hpp"

namespace py = pybind11;
using namespace booster_policy;

namespace {

using FloatArray = py::array_t<float, py::array::c_style | py::array::forcecast>;

std::vector<float> ToVector(const FloatArray& array, std::size_t size, const char* name) {
  if (static_cast<std::size_t>(array.size()) != size) {
    throw std::invalid_argument(std::string(name) + " must contain " + std::to_string(size) +
                                " values");
  }
  return std::vector<float>(array.data(), array.data() + size);
}

template <std::size_t N>
std::array<float, N> ToArray(const FloatArray& array, const char* name) {
  const auto values = ToVector(array, N, name);
  std::array<float, N> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

py::array_t<float> ToNumpy(const std::vector<float>& values) {
  py::array_t<float> array(static_cast<py::ssize_t>(values.size()));
  std::copy(values.begin(), values.end(), array.mutable_data());
  return array;
}

}  // namespace

PYBIND11_MODULE(booster_policy_core, m) {
  m.doc() = "C++ walk, squat and sit policy inference shared with the booster_policy ROS node";
  m.attr("POLICY_JOINT_COUNT") = kPolicyJointCount;
  m.attr("WALK_HISTORY_LENGTH") = kWalkHistoryLength;
  m.attr("WALK_OBSERVATION_SIZE") = kWalkObservationSize;
  m.attr("MAX_TRANSLATIONAL_SPEED") = kMaxTranslationalSpeed;
  m.attr("MAX_YAW_RATE") = kMaxYawRate;

  py::enum_<PolicyMode>(m, "PolicyMode")
      .value("WALK", PolicyMode::kWalk)
      .value("SQUAT", PolicyMode::kSquat);
  py::enum_<ActivePolicy>(m, "ActivePolicy")
      .value("WALK", ActivePolicy::kWalk)
      .value("SQUAT", ActivePolicy::kSquat)
      .value("SIT", ActivePolicy::kSit);
  py::enum_<PosePolicy>(m, "PosePolicy")
      .value("SQUAT", PosePolicy::kSquat)
      .value("SIT", PosePolicy::kSit);

  py::class_<RobotConfig>(m, "RobotConfig")
      .def(py::init<>())
      .def_readwrite("joint_names", &RobotConfig::joint_names)
      .def_readwrite("default_joint_pos", &RobotConfig::default_joint_pos)
      .def_readwrite("joint_stiffness", &RobotConfig::joint_stiffness)
      .def_readwrite("joint_damping", &RobotConfig::joint_damping);

  py::class_<PolicyConfig>(m, "PolicyConfig")
      .def(py::init<>())
      .def_readwrite("mode", &PolicyConfig::mode)
      .def_readwrite("walk_model_path", &PolicyConfig::walk_model_path)
      .def_readwrite("walk_gain_overrides_path", &PolicyConfig::walk_gain_overrides_path)
      .def_readwrite("squat_model_path", &PolicyConfig::squat_model_path)
      .def_readwrite("squat_gain_overrides_path", &PolicyConfig::squat_gain_overrides_path)
      .def_readwrite("sit_model_path", &PolicyConfig::sit_model_path)
      .def_readwrite("enable_safety_fallback", &PolicyConfig::enable_safety_fallback)
      .def_readwrite("min_upright_projection", &PolicyConfig::min_upright_projection)
      .def_readwrite("standing_joint_pos_tolerance",
                     &PolicyConfig::standing_joint_pos_tolerance)
      .def_readwrite("onnx_threads", &PolicyConfig::onnx_threads)
      .def_readwrite("robot", &PolicyConfig::robot);

  py::class_<PolicyController>(m, "PolicyController")
      .def(py::init<PolicyConfig>(), py::arg("config"))
      .def("reset", &PolicyController::Reset)
      .def(
          "step",
          [](PolicyController& controller, const FloatArray& angular_velocity,
             const FloatArray& projected_gravity, const FloatArray& joint_pos,
             const FloatArray& joint_vel, const FloatArray& velocity_command,
             const FloatArray& head_target, bool squat, PosePolicy pose,
             const FloatArray& root_quat) {
            RobotState state;
            state.angular_velocity = ToArray<3>(angular_velocity, "angular_velocity");
            state.projected_gravity = ToArray<3>(projected_gravity, "projected_gravity");
            state.root_quat = ToArray<4>(root_quat, "root_quat");
            state.joint_pos = ToVector(joint_pos, kPolicyJointCount, "joint_pos");
            state.joint_vel = ToVector(joint_vel, kPolicyJointCount, "joint_vel");
            PolicyCommand command;
            command.velocity = ToArray<3>(velocity_command, "velocity_command");
            command.head_target = ToArray<2>(head_target, "head_target");
            command.squat = squat;
            command.pose = pose;
            JointCommand result;
            {
              py::gil_scoped_release release;
              result = controller.Step(state, command);
            }
            return py::make_tuple(ToNumpy(result.position), ToNumpy(result.stiffness),
                                  ToNumpy(result.damping));
          },
          py::arg("angular_velocity"), py::arg("projected_gravity"), py::arg("joint_pos"),
          py::arg("joint_vel"), py::arg("velocity_command"), py::arg("head_target"),
          py::arg("squat"), py::arg("pose") = PosePolicy::kSquat,
          py::arg("root_quat") = std::array<float, 4>{1.0F, 0.0F, 0.0F, 0.0F},
          "Run one policy step; returns (joint targets, stiffness, damping) in robot order.\n\n"
          "`root_quat` is the (w, x, y, z) world-from-base orientation; only sit uses it.")
      .def_property_readonly("active_policy", &PolicyController::active_policy)
      .def_property_readonly("squat_started", &PolicyController::squat_started)
      .def_property_readonly("standing_pose_complete", &PolicyController::standing_pose_complete)
      .def_property_readonly("upright_fault", &PolicyController::upright_fault)
      .def_property_readonly(
          "walk_history",
          [](const PolicyController& controller) {
            return ToNumpy(controller.walk().history())
                .reshape({static_cast<py::ssize_t>(kWalkHistoryLength),
                          static_cast<py::ssize_t>(kWalkObservationSize)});
          })
      .def_property_readonly(
          "walk_command_input",
          [](const PolicyController& controller) {
            return ToNumpy(controller.walk().command_input());
          })
      .def_property_readonly(
          "walk_gains",
          [](const PolicyController& controller) {
            const auto& gains = controller.walk().walk_gains();
            return py::make_tuple(ToNumpy(gains.stiffness), ToNumpy(gains.damping));
          })
      .def_property_readonly("squat_gains",
                             [](const PolicyController& controller) {
                               const auto& gains = controller.walk().squat_gains();
                               return py::make_tuple(ToNumpy(gains.stiffness),
                                                     ToNumpy(gains.damping));
                             })
      .def_property_readonly("sit_gains", [](const PolicyController& controller) {
        const auto& gains = controller.walk().sit_gains();
        return py::make_tuple(ToNumpy(gains.stiffness), ToNumpy(gains.damping));
      });

  m.def("quaternion_from_rpy", &QuaternionFromRpy, py::arg("roll"), py::arg("pitch"),
        py::arg("yaw"));
  m.def("projected_gravity_from_rpy", &ProjectedGravityFromRpy, py::arg("roll"),
        py::arg("pitch"), py::arg("yaw"));
  m.def("projected_gravity_from_quaternion", &ProjectedGravityFromQuaternion, py::arg("w"),
        py::arg("x"), py::arg("y"), py::arg("z"));
}
