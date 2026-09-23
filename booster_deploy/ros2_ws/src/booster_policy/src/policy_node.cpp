#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <booster_interface/msg/low_cmd.hpp>
#include <booster_interface/msg/low_state.hpp>
#include <booster_interface/msg/motor_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "booster_policy/msg/policy_command.hpp"
#include "booster_policy/msg/policy_status.hpp"
#include "booster_policy/policy.hpp"
#include "booster_policy/srv/start_policy.hpp"

namespace booster_policy {
namespace {

using Clock = std::chrono::steady_clock;
using booster_interface::msg::LowCmd;
using booster_interface::msg::LowState;
using booster_interface::msg::MotorCmd;
using StatusMsg = booster_policy::msg::PolicyStatus;
using CommandMsg = booster_policy::msg::PolicyCommand;

std::vector<float> ToFloats(const std::vector<double>& values) {
  return std::vector<float>(values.begin(), values.end());
}

uint8_t ToStatusPolicy(ActivePolicy policy) {
  switch (policy) {
    case ActivePolicy::kSquat:
      return StatusMsg::POLICY_SQUAT;
    case ActivePolicy::kSit:
      return StatusMsg::POLICY_SIT;
    case ActivePolicy::kWalk:
      break;
  }
  return StatusMsg::POLICY_WALK;
}

// Runs the policies at the policy rate and publishes `joint_ctrl`.
//
// Subscriptions and services run on the executor; inference runs on a
// dedicated control thread so it is not delayed by other callbacks. Wall-clock
// timing is the default; with `use_low_state_clock` one policy step runs per
// `policy_dt / low_state_dt` /low_state messages, matching simulators that
// publish state at simulated time.
class PolicyNode : public rclcpp::Node {
 public:
  explicit PolicyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("booster_policy", options) {
    PolicyConfig config;
    const std::string mode = declare_parameter<std::string>("mode", "walk");
    if (mode == "walk") {
      config.mode = PolicyMode::kWalk;
    } else if (mode == "squat") {
      config.mode = PolicyMode::kSquat;
    } else {
      throw std::invalid_argument("Parameter 'mode' must be 'walk' or 'squat', got '" + mode + "'");
    }
    config.walk_model_path = declare_parameter<std::string>("walk_model_path", "");
    config.walk_gain_overrides_path = declare_parameter<std::string>("walk_gain_overrides_path", "");
    config.squat_model_path = declare_parameter<std::string>("squat_model_path", "");
    config.squat_gain_overrides_path =
        declare_parameter<std::string>("squat_gain_overrides_path", "");
    config.sit_model_path = declare_parameter<std::string>("sit_model_path", "");
    config.enable_safety_fallback = declare_parameter<bool>("enable_safety_fallback", true);
    config.min_upright_projection =
        static_cast<float>(declare_parameter<double>("min_upright_projection", 0.5));
    config.standing_joint_pos_tolerance =
        static_cast<float>(declare_parameter<double>("standing_joint_pos_tolerance", 0.3));
    config.onnx_threads = static_cast<int>(declare_parameter<int64_t>("onnx_threads", 1));
    config.robot.joint_names =
        declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    config.robot.default_joint_pos = ToFloats(
        declare_parameter<std::vector<double>>("default_joint_pos", std::vector<double>{}));
    config.robot.joint_stiffness = ToFloats(
        declare_parameter<std::vector<double>>("joint_stiffness", std::vector<double>{}));
    config.robot.joint_damping = ToFloats(
        declare_parameter<std::vector<double>>("joint_damping", std::vector<double>{}));

    const double policy_dt = declare_parameter<double>("policy_dt", 0.02);
    const double low_state_dt = declare_parameter<double>("low_state_dt", 0.002);
    use_low_state_clock_ = declare_parameter<bool>("use_low_state_clock", false);
    command_timeout_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(declare_parameter<double>("command_timeout", 0.5)));
    if (policy_dt <= 0.0 || low_state_dt <= 0.0) {
      throw std::invalid_argument("policy_dt and low_state_dt must be positive");
    }
    policy_period_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(policy_dt));
    low_state_decimation_ =
        std::max<uint64_t>(1, static_cast<uint64_t>(std::llround(policy_dt / low_state_dt)));

    controller_ = std::make_unique<PolicyController>(std::move(config));
    state_.joint_pos.assign(kPolicyJointCount, 0.0F);
    state_.joint_vel.assign(kPolicyJointCount, 0.0F);
    InitLowCmd();

    low_cmd_publisher_ = create_publisher<LowCmd>(
        "joint_ctrl", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    status_publisher_ = create_publisher<StatusMsg>("~/status", rclcpp::QoS(10));
    low_state_subscription_ = create_subscription<LowState>(
        "/low_state", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](LowState::ConstSharedPtr msg) { OnLowState(*msg); });
    command_subscription_ = create_subscription<CommandMsg>(
        "~/command", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](CommandMsg::ConstSharedPtr msg) { OnCommand(*msg); });
    start_service_ = create_service<srv::StartPolicy>(
        "~/start", [this](const std::shared_ptr<srv::StartPolicy::Request>,
                          std::shared_ptr<srv::StartPolicy::Response> response) {
          OnStart(response.get());
        });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        "~/stop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          OnStop(response.get());
        });
    // Running sessions publish status every step; this keeps idle status fresh.
    idle_status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      bool running = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        running = run_state_ == StatusMsg::STATE_RUNNING;
      }
      if (!running) {
        PublishStatus();
      }
    });

    control_thread_ = std::thread([this]() { ControlLoop(); });
    RCLCPP_INFO(get_logger(), "Policy node ready in %s mode (%s timing, %.1f Hz)", mode.c_str(),
                use_low_state_clock_ ? "low state" : "wall clock", 1.0 / policy_dt);
  }

  ~PolicyNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    tick_cv_.notify_all();
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
  }

 private:
  void InitLowCmd() {
    low_cmd_.cmd_type = LowCmd::CMD_TYPE_SERIAL;
    low_cmd_.motor_cmd.resize(kPolicyJointCount);
    for (MotorCmd& motor : low_cmd_.motor_cmd) {
      motor.q = 0.0F;
      motor.dq = 0.0F;
      motor.tau = 0.0F;
      motor.kp = 0.0F;
      motor.kd = 0.0F;
      motor.weight = 0.0F;
    }
  }

  void OnLowState(const LowState& msg) {
    if (msg.motor_state_serial.size() < kPolicyJointCount) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Ignoring /low_state with %zu serial motors; expected %zu",
                           msg.motor_state_serial.size(), kPolicyJointCount);
      return;
    }
    bool tick = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::copy(msg.imu_state.gyro.begin(), msg.imu_state.gyro.end(),
                state_.angular_velocity.begin());
      state_.root_quat =
          QuaternionFromRpy(msg.imu_state.rpy[0], msg.imu_state.rpy[1], msg.imu_state.rpy[2]);
      state_.projected_gravity =
          ProjectedGravityFromRpy(msg.imu_state.rpy[0], msg.imu_state.rpy[1], msg.imu_state.rpy[2]);
      float max_speed = 0.0F;
      for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
        state_.joint_pos[i] = msg.motor_state_serial[i].q;
        state_.joint_vel[i] = msg.motor_state_serial[i].dq;
        max_speed = std::max(max_speed, std::abs(state_.joint_vel[i]));
      }
      max_joint_speed_ = max_speed;
      has_low_state_ = true;
      if (use_low_state_clock_ && ++low_state_count_ % low_state_decimation_ == 0) {
        ++pending_ticks_;
        tick = true;
      }
    }
    if (tick) {
      tick_cv_.notify_all();
    }
  }

  void OnCommand(const CommandMsg& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_.velocity = {msg.vx, msg.vy, msg.yaw_rate};
    command_.head_target = {msg.head_yaw, msg.head_pitch};
    command_.squat = msg.squat;
    if (msg.pose_policy == CommandMsg::POSE_SIT) {
      command_.pose = PosePolicy::kSit;
    } else {
      if (msg.pose_policy != CommandMsg::POSE_SQUAT) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Unknown pose_policy %u; using squat", msg.pose_policy);
      }
      command_.pose = PosePolicy::kSquat;
    }
    last_command_time_ = Clock::now();
  }

  void OnStart(srv::StartPolicy::Response* response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_low_state_) {
      response->success = false;
      response->message = "No /low_state received yet";
      response->session = session_;
      return;
    }
    // The control thread resets the policies before its next step so the
    // controller is only touched from one thread.
    ++session_;
    reset_pending_ = true;
    run_state_ = StatusMsg::STATE_RUNNING;
    ready_ = false;
    fault_.clear();
    step_ = 0;
    // Start from a neutral command until the client publishes its own.
    command_ = PolicyCommand{};
    last_command_time_.reset();
    command_timed_out_ = false;
    active_policy_ = StatusMsg::POLICY_WALK;
    squat_started_ = false;
    standing_pose_complete_ = false;
    response->success = true;
    response->message = "Policy session started";
    response->session = session_;
    RCLCPP_INFO(get_logger(), "Starting policy session %u", session_);
  }

  void OnStop(std_srvs::srv::Trigger::Response* response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (run_state_ == StatusMsg::STATE_RUNNING) {
      RCLCPP_INFO(get_logger(), "Stopping policy session %u", session_);
    }
    if (run_state_ != StatusMsg::STATE_FAULT) {
      run_state_ = StatusMsg::STATE_IDLE;
    }
    ready_ = false;
    response->success = true;
    response->message = "Policy stopped";
  }

  // Waits until the next policy step is due; returns false on shutdown.
  bool WaitForTick(Clock::time_point* next_tick) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (use_low_state_clock_) {
      tick_cv_.wait(lock, [this]() { return shutdown_ || pending_ticks_ > 0; });
      // Drop ticks that queued while a step was running late.
      pending_ticks_ = 0;
    } else {
      tick_cv_.wait_until(lock, *next_tick, [this]() { return shutdown_; });
      const auto now = Clock::now();
      *next_tick += policy_period_;
      if (*next_tick < now) {
        // Skip missed periods instead of bursting to catch up.
        *next_tick = now + policy_period_;
      }
    }
    return !shutdown_ && rclcpp::ok();
  }

  void ControlLoop() {
    auto next_tick = Clock::now() + policy_period_;
    RobotState state;
    PolicyCommand command;
    while (WaitForTick(&next_tick)) {
      bool reset = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (run_state_ != StatusMsg::STATE_RUNNING) {
          continue;
        }
        reset = reset_pending_;
        reset_pending_ = false;
        state = state_;
        command = command_;
        const bool timed_out = !last_command_time_.has_value() ||
                               Clock::now() - *last_command_time_ > command_timeout_;
        if (timed_out && last_command_time_.has_value() && !command_timed_out_) {
          RCLCPP_WARN(get_logger(), "No policy command for %.2f s; zeroing velocity",
                      std::chrono::duration<double>(command_timeout_).count());
        }
        command_timed_out_ = timed_out && last_command_time_.has_value();
        if (timed_out) {
          command.velocity = {0.0F, 0.0F, 0.0F};
        }
      }

      if (reset) {
        controller_->Reset();
      }
      const auto inference_start = Clock::now();
      std::optional<std::string> error;
      try {
        const JointCommand& targets = controller_->Step(state, command);
        for (std::size_t i = 0; i < kPolicyJointCount; ++i) {
          low_cmd_.motor_cmd[i].q = targets.position[i];
          low_cmd_.motor_cmd[i].kp = targets.stiffness[i];
          low_cmd_.motor_cmd[i].kd = targets.damping[i];
        }
      } catch (const std::exception& exc) {
        error = exc.what();
      }
      const float inference_ms =
          std::chrono::duration<float, std::milli>(Clock::now() - inference_start).count();

      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reset_pending_ || run_state_ != StatusMsg::STATE_RUNNING) {
          // A stop or a new session arrived during inference; drop this step.
          continue;
        }
        if (error) {
          run_state_ = StatusMsg::STATE_FAULT;
          fault_ = "Policy step failed: " + *error;
          ready_ = false;
          RCLCPP_ERROR(get_logger(), "%s", fault_.c_str());
        } else {
          low_cmd_publisher_->publish(low_cmd_);
          ready_ = low_cmd_publisher_->get_subscription_count() > 0;
          inference_ms_ = inference_ms;
          ++step_;
          active_policy_ = ToStatusPolicy(controller_->active_policy());
          squat_started_ = controller_->squat_started();
          standing_pose_complete_ = controller_->standing_pose_complete();
          if (controller_->upright_fault()) {
            run_state_ = StatusMsg::STATE_FAULT;
            fault_ = "Large orientation error detected; policy stopped";
            ready_ = false;
            RCLCPP_ERROR(get_logger(), "%s", fault_.c_str());
          }
        }
      }
      PublishStatus();
    }
  }

  void PublishStatus() {
    StatusMsg status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status.state = run_state_;
      status.session = session_;
      status.ready = ready_;
      status.squat_commanded = command_.squat;
      status.command_timed_out = command_timed_out_;
      status.max_joint_speed =
          has_low_state_ ? max_joint_speed_ : std::numeric_limits<float>::quiet_NaN();
      status.inference_ms = inference_ms_;
      status.step = step_;
      status.fault = fault_;
      status.active_policy = active_policy_;
      status.squat_started = squat_started_;
      status.standing_pose_complete = standing_pose_complete_;
    }
    status.stamp = now();
    status_publisher_->publish(status);
  }

  // Only touched by the control thread after construction.
  std::unique_ptr<PolicyController> controller_;
  bool use_low_state_clock_ = false;
  Clock::duration policy_period_{};
  Clock::duration command_timeout_{};
  uint64_t low_state_decimation_ = 1;

  rclcpp::Publisher<LowCmd>::SharedPtr low_cmd_publisher_;
  rclcpp::Publisher<StatusMsg>::SharedPtr status_publisher_;
  rclcpp::Subscription<LowState>::SharedPtr low_state_subscription_;
  rclcpp::Subscription<CommandMsg>::SharedPtr command_subscription_;
  rclcpp::Service<srv::StartPolicy>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr idle_status_timer_;
  LowCmd low_cmd_;

  std::mutex mutex_;
  std::condition_variable tick_cv_;
  std::thread control_thread_;
  bool shutdown_ = false;
  RobotState state_;
  float max_joint_speed_ = 0.0F;
  bool has_low_state_ = false;
  uint64_t low_state_count_ = 0;
  uint64_t pending_ticks_ = 0;
  PolicyCommand command_;
  std::optional<Clock::time_point> last_command_time_;
  bool command_timed_out_ = false;
  uint32_t session_ = 0;
  uint8_t run_state_ = StatusMsg::STATE_IDLE;
  bool reset_pending_ = false;
  bool ready_ = false;
  float inference_ms_ = 0.0F;
  uint64_t step_ = 0;
  // Controller results, copied under the lock after each step.
  uint8_t active_policy_ = StatusMsg::POLICY_WALK;
  bool squat_started_ = false;
  bool standing_pose_complete_ = false;
  std::string fault_;
};

}  // namespace
}  // namespace booster_policy

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<booster_policy::PolicyNode>());
  } catch (const std::exception& exc) {
    fprintf(stderr, "booster_policy node failed: %s\n", exc.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
