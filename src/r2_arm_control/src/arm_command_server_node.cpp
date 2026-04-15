#include "r2_arm_control/angle_mapping.hpp"
#include "r2_arm_control/ik_solver.hpp"
#include "r2_arm_control/msg/arm_cartesian_target.hpp"
#include "r2_arm_control/msg/arm_motion_state.hpp"
#include "r2_arm_control/srv/move_to_xz.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{

double advance_axis(
  double current,
  double & velocity,
  double target,
  double max_velocity,
  double max_acceleration,
  double dt)
{
  const double safe_dt = std::max(1e-4, dt);
  const double limited_max_velocity = std::max(1e-4, max_velocity);
  const double limited_max_accel = std::max(1e-4, max_acceleration);
  const double error = target - current;
  const double desired_velocity = std::clamp(error / safe_dt, -limited_max_velocity, limited_max_velocity);
  const double max_delta_velocity = limited_max_accel * safe_dt;
  const double delta_velocity = std::clamp(desired_velocity - velocity, -max_delta_velocity, max_delta_velocity);

  velocity = std::clamp(velocity + delta_velocity, -limited_max_velocity, limited_max_velocity);

  const double step = velocity * safe_dt;
  if (std::fabs(step) >= std::fabs(error)) {
    velocity = 0.0;
    return target;
  }

  current += step;
  if (std::fabs(target - current) < 1e-6 && std::fabs(velocity) < max_delta_velocity) {
    velocity = 0.0;
    return target;
  }

  return current;
}

}  // namespace

class ArmCommandServerNode : public rclcpp::Node
{
public:
  ArmCommandServerNode()
  : Node("arm_command_server_node"), solver_(params_)
  {
    declare_parameter("d1", params_.d1);
    declare_parameter("d2", params_.d2);
    declare_parameter("q1_min", params_.q1_min);
    declare_parameter("q1_max", params_.q1_max);
    declare_parameter("q2_min", params_.q2_min);
    declare_parameter("q2_max", params_.q2_max);
    declare_parameter("kp_joint0", 15.0);
    declare_parameter("kp_joint1", 15.0);
    declare_parameter("kd_joint0", 1.0);
    declare_parameter("kd_joint1", 1.0);
    declare_parameter("move_duration", 2.0);
    declare_parameter("elbow_up", true);
    declare_parameter("x_offset", 0.0);
    declare_parameter("z_offset", 0.0);
    declare_parameter("move_service_name", "/r2/arm/move_to_xz");
    declare_parameter("target_topic", "/r2/arm/target_xz");
    declare_parameter("state_topic", "/r2/arm/state");
    declare_parameter("joint_state_topic", "/joint_states");
    declare_parameter("feedback_status_topic", "/r2/arm/feedback_status");
    declare_parameter("position_command_topic", "/arm_position_des_controller/commands");
    declare_parameter("velocity_command_topic", "/arm_velocity_des_controller/commands");
    declare_parameter("kp_command_topic", "/arm_kp_controller/commands");
    declare_parameter("kd_command_topic", "/arm_kd_controller/commands");
    declare_parameter("feedforward_command_topic", "/arm_feedforward_controller/commands");
    declare_parameter("control_rate_hz", 200.0);
    declare_parameter("target_timeout_sec", 0.10);
    declare_parameter("max_cart_vel_x", 0.60);
    declare_parameter("max_cart_vel_z", 0.60);
    declare_parameter("max_cart_accel", 2.50);
    declare_parameter("joint_vel_limit", 2.0);
    declare_parameter("settle_tolerance_x", 0.008);
    declare_parameter("settle_tolerance_z", 0.008);
    declare_parameter("settle_cycles", 8);
    declare_parameter("feedback_timeout_sec", 0.05);
    declare_parameter("feedback_hard_timeout_sec", 0.30);
    declare_parameter("feedback_startup_grace_sec", 8.0);

    params_.d1 = get_parameter("d1").as_double();
    params_.d2 = get_parameter("d2").as_double();
    params_.q1_min = get_parameter("q1_min").as_double();
    params_.q1_max = get_parameter("q1_max").as_double();
    params_.q2_min = get_parameter("q2_min").as_double();
    params_.q2_max = get_parameter("q2_max").as_double();

    kp0_ = get_parameter("kp_joint0").as_double();
    kp1_ = get_parameter("kp_joint1").as_double();
    kd0_ = get_parameter("kd_joint0").as_double();
    kd1_ = get_parameter("kd_joint1").as_double();
    move_duration_ignored_ = get_parameter("move_duration").as_double();
    elbow_up_ = get_parameter("elbow_up").as_bool();

    x_offset_ = get_parameter("x_offset").as_double();
    z_offset_ = get_parameter("z_offset").as_double();

    control_rate_hz_ = std::max(20.0, get_parameter("control_rate_hz").as_double());
    target_timeout_sec_ = std::max(0.01, get_parameter("target_timeout_sec").as_double());
    max_cart_vel_x_ = std::max(0.05, get_parameter("max_cart_vel_x").as_double());
    max_cart_vel_z_ = std::max(0.05, get_parameter("max_cart_vel_z").as_double());
    max_cart_accel_ = std::max(0.10, get_parameter("max_cart_accel").as_double());
    joint_vel_limit_ = std::max(0.10, get_parameter("joint_vel_limit").as_double());
    settle_tolerance_x_ = std::max(1e-4, get_parameter("settle_tolerance_x").as_double());
    settle_tolerance_z_ = std::max(1e-4, get_parameter("settle_tolerance_z").as_double());
    settle_cycles_ = std::max<int>(1, static_cast<int>(get_parameter("settle_cycles").as_int()));
    feedback_timeout_sec_ = std::max(0.01, get_parameter("feedback_timeout_sec").as_double());
    feedback_hard_timeout_sec_ = std::max(
      feedback_timeout_sec_, get_parameter("feedback_hard_timeout_sec").as_double());
    feedback_startup_grace_sec_ = std::max(
      0.0, get_parameter("feedback_startup_grace_sec").as_double());

    const std::string move_service_name = get_parameter("move_service_name").as_string();
    const std::string target_topic = get_parameter("target_topic").as_string();
    const std::string state_topic = get_parameter("state_topic").as_string();
    const std::string joint_state_topic = get_parameter("joint_state_topic").as_string();
    const std::string feedback_status_topic = get_parameter("feedback_status_topic").as_string();
    const std::string position_command_topic = get_parameter("position_command_topic").as_string();
    const std::string velocity_command_topic = get_parameter("velocity_command_topic").as_string();
    const std::string kp_command_topic = get_parameter("kp_command_topic").as_string();
    const std::string kd_command_topic = get_parameter("kd_command_topic").as_string();
    const std::string feedforward_command_topic =
      get_parameter("feedforward_command_topic").as_string();

    solver_ = r2_arm_control::IkSolver(params_);

    pos_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(position_command_topic, 10);
    vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(velocity_command_topic, 10);
    kp_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(kp_command_topic, 10);
    kd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(kd_command_topic, 10);
    ff_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(feedforward_command_topic, 10);
    state_pub_ = create_publisher<r2_arm_control::msg::ArmMotionState>(state_topic, 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic,
      10,
      std::bind(&ArmCommandServerNode::jointStateCallback, this, std::placeholders::_1));

    target_sub_ = create_subscription<r2_arm_control::msg::ArmCartesianTarget>(
      target_topic,
      10,
      std::bind(&ArmCommandServerNode::targetCallback, this, std::placeholders::_1));

    feedback_status_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      feedback_status_topic,
      10,
      std::bind(&ArmCommandServerNode::feedbackStatusCallback, this, std::placeholders::_1));

    move_service_ = create_service<r2_arm_control::srv::MoveToXZ>(
      move_service_name,
      std::bind(
        &ArmCommandServerNode::handleMoveRequest,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&ArmCommandServerNode::timerCallback, this));

    startup_time_ = now();
    last_control_time_ = startup_time_;
    hold_q1_ = 0.0;
    hold_q2_ = 0.0;
    current_state_ = "IDLE";
    status_message_ = "waiting for command";

    RCLCPP_DEBUG(get_logger(), "Arm command server ready");
    RCLCPP_DEBUG(get_logger(), "Service: %s", move_service_name.c_str());
    RCLCPP_DEBUG(get_logger(), "Target topic: %s", target_topic.c_str());
    RCLCPP_DEBUG(get_logger(), "State topic: %s", state_topic.c_str());
    RCLCPP_DEBUG(
      get_logger(),
      "Stream tracking enabled at %.1f Hz (move_duration kept only for compatibility: %.3f s)",
      control_rate_hz_,
      move_duration_ignored_);
  }

private:
  struct CartesianPoint
  {
    double x {0.0};
    double z {0.0};
  };

  enum class TargetSource
  {
    None,
    Stream,
    Service,
  };

  struct FeedbackHealth
  {
    bool feedback_ok {true};
    bool hard_fault {false};
    std::uint8_t stale_motor_count {0};
  };

  struct TimerOutputs
  {
    std::vector<double> position_command;
    std::vector<double> velocity_command;
    std::vector<double> kp_command;
    std::vector<double> kd_command;
    std::vector<double> feedforward_command;
    r2_arm_control::msg::ArmMotionState state_msg;
  };

  bool validateTarget(const CartesianPoint & target, std::string & message) const
  {
    if (!std::isfinite(target.x) || !std::isfinite(target.z)) {
      message = "Target rejected: x and z must both be finite numbers.";
      return false;
    }

    const auto ik = solver_.solve(target.x + x_offset_, target.z + z_offset_, elbow_up_);
    if (!ik.success) {
      std::ostringstream oss;
      oss << "Target rejected: x=" << target.x << ", z=" << target.z
          << " is outside the safe workspace or joint limits.";
      message = oss.str();
      return false;
    }

    message = "target accepted";
    return true;
  }

  CartesianPoint currentCartesianLocked() const
  {
    const double state_q1 = have_joint_state_ ? current_q1_ : hold_q1_;
    const double state_q2 = have_joint_state_ ? current_q2_ : hold_q2_;
    const auto fk = solver_.forward(state_q1, state_q2);
    return {fk.x - x_offset_, fk.z - z_offset_};
  }

  void initializeTrackerLocked(const CartesianPoint & current_point)
  {
    smoothed_target_ = current_point;
    smoothed_velocity_x_ = 0.0;
    smoothed_velocity_z_ = 0.0;
    last_command_q1_ = have_joint_state_ ? current_q1_ : hold_q1_;
    last_command_q2_ = have_joint_state_ ? current_q2_ : hold_q2_;
    hold_q1_ = last_command_q1_;
    hold_q2_ = last_command_q2_;
    tracker_initialized_ = true;
  }

  bool solveUserTarget(
    const CartesianPoint & target,
    double & q1,
    double & q2,
    std::string & message) const
  {
    const auto ik = solver_.solve(target.x + x_offset_, target.z + z_offset_, elbow_up_);
    if (!ik.success) {
      std::ostringstream oss;
      oss << "IK failed for x=" << target.x << ", z=" << target.z;
      message = oss.str();
      return false;
    }

    q1 = ik.q1;
    q2 = ik.q2;
    message = "ik ok";
    return true;
  }

  std::optional<CartesianPoint> activeTargetLocked(
    const rclcpp::Time & now_time,
    TargetSource & source) const
  {
    const bool has_fresh_stream_target =
      stream_target_.has_value() && last_stream_time_.nanoseconds() > 0 &&
      (now_time - last_stream_time_).seconds() <= target_timeout_sec_;

    if (has_fresh_stream_target) {
      source = TargetSource::Stream;
      return stream_target_;
    }

    if (service_target_) {
      source = TargetSource::Service;
      return service_target_;
    }

    source = TargetSource::None;
    return std::nullopt;
  }

  FeedbackHealth computeFeedbackHealthLocked(const rclcpp::Time & now_time)
  {
    FeedbackHealth result;
    result.feedback_ok = true;

    bool soft_fault = false;

    if (feedback_status_received_) {
      const double status_age = (now_time - last_feedback_status_time_).seconds();
      result.stale_motor_count = latest_stale_motor_count_;
      soft_fault = !latest_feedback_ok_ || latest_stale_motor_count_ > 0 || status_age > feedback_timeout_sec_;

      if (status_age > feedback_timeout_sec_) {
        result.stale_motor_count = std::max<std::uint8_t>(result.stale_motor_count, 1);
      }
    } else if (
      !have_joint_state_ &&
      (now_time - startup_time_).seconds() > feedback_startup_grace_sec_)
    {
      soft_fault = true;
      result.stale_motor_count = 2;
    }

    if (soft_fault) {
      if (!feedback_fault_active_) {
        feedback_fault_active_ = true;
        feedback_fault_started_at_ = now_time;
      }
      result.feedback_ok = false;
      result.hard_fault =
        feedback_fault_active_ &&
        (now_time - feedback_fault_started_at_).seconds() >= feedback_hard_timeout_sec_;
    } else {
      feedback_fault_active_ = false;
      result.feedback_ok = true;
      result.hard_fault = false;
      result.stale_motor_count = 0;
    }

    if (last_feedback_ok_state_ != result.feedback_ok && result.feedback_ok) {
      tracker_initialized_ = false;
    }
    last_feedback_ok_state_ = result.feedback_ok;

    return result;
  }

  void updateTrackingStateLocked(
    const FeedbackHealth & feedback_health,
    const CartesianPoint & current_point,
    const std::optional<CartesianPoint> & active_target,
    TargetSource source)
  {
    if (feedback_health.hard_fault) {
      current_state_ = "ERROR";
      status_message_ = "feedback timeout, motion halted";
      settle_counter_ = 0;
      return;
    }

    if (!feedback_health.feedback_ok) {
      current_state_ = "HOLD";
      status_message_ = "feedback stale, holding last safe command";
      settle_counter_ = 0;
      return;
    }

    if (!active_target) {
      current_state_ = "IDLE";
      status_message_ = "waiting for command";
      settle_counter_ = 0;
      return;
    }

    const double error_x = active_target->x - current_point.x;
    const double error_z = active_target->z - current_point.z;
    const bool within_tolerance =
      std::fabs(error_x) <= settle_tolerance_x_ && std::fabs(error_z) <= settle_tolerance_z_;

    if (within_tolerance) {
      settle_counter_ = std::min(settle_counter_ + 1, settle_cycles_);
    } else {
      settle_counter_ = 0;
    }

    if (source == TargetSource::Service && settle_counter_ >= settle_cycles_) {
      current_state_ = "REACHED";
      status_message_ = "service target reached";
      service_target_.reset();
      settle_counter_ = 0;
      return;
    }

    current_state_ = "MOVING";
    status_message_ = source == TargetSource::Stream ? "tracking stream target" : "tracking service target";
  }

  r2_arm_control::msg::ArmMotionState buildStateSnapshotLocked(
    const CartesianPoint & current_point,
    const CartesianPoint & display_target,
    const FeedbackHealth & feedback_health) const
  {
    r2_arm_control::msg::ArmMotionState msg;
    msg.state = current_state_;
    msg.current_x = current_point.x;
    msg.current_z = current_point.z;
    msg.target_x = display_target.x;
    msg.target_z = display_target.z;
    msg.feedback_ok = feedback_health.feedback_ok;
    msg.stale_motor_count = feedback_health.stale_motor_count;
    msg.message = status_message_;
    return msg;
  }

  void publishArray(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr & publisher,
    const std::vector<double> & values)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = values;
    publisher->publish(msg);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    bool saw_shoulder_joint = false;
    bool saw_elbow_joint = false;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) {
        continue;
      }

      if (msg->name[i] == "shoulder_joint") {
        current_q1_ = r2_arm_control::urdfToPhysical(msg->position[i]);
        saw_shoulder_joint = true;
      } else if (msg->name[i] == "elbow_joint") {
        current_q2_ = r2_arm_control::urdfToPhysical(msg->position[i]);
        saw_elbow_joint = true;
      }
    }

    have_shoulder_joint_ = have_shoulder_joint_ || saw_shoulder_joint;
    have_elbow_joint_ = have_elbow_joint_ || saw_elbow_joint;
    have_joint_state_ = have_shoulder_joint_ && have_elbow_joint_;
  }

  void targetCallback(const r2_arm_control::msg::ArmCartesianTarget::SharedPtr msg)
  {
    const CartesianPoint requested_target {msg->x, msg->z};
    std::string validation_message;
    if (!validateTarget(requested_target, validation_message)) {
      const auto now_time = now();
      if ((now_time - last_invalid_target_log_time_).seconds() >= 0.5) {
        RCLCPP_WARN(get_logger(), "%s", validation_message.c_str());
        last_invalid_target_log_time_ = now_time;
      }
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    stream_target_ = requested_target;
    service_target_.reset();
    last_stream_time_ = msg->stamp.sec == 0 && msg->stamp.nanosec == 0 ?
      now() :
      rclcpp::Time(msg->stamp);
    last_target_x_ = requested_target.x;
    last_target_z_ = requested_target.z;
  }

  void feedbackStatusCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_feedback_ok_ = msg->data[0] != 0;
    latest_stale_motor_count_ = msg->data[1];
    last_feedback_status_time_ = now();
    feedback_status_received_ = true;
  }

  void handleMoveRequest(
    const std::shared_ptr<r2_arm_control::srv::MoveToXZ::Request> request,
    std::shared_ptr<r2_arm_control::srv::MoveToXZ::Response> response)
  {
    const CartesianPoint requested_target {request->x, request->z};
    std::string validation_message;
    const bool valid = validateTarget(requested_target, validation_message);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_target_x_ = request->x;
      last_target_z_ = request->z;

      if (valid) {
        service_target_ = requested_target;
        response->accepted = true;
        response->message = "Target accepted and latched for stream tracker.";
        status_message_ = "service target accepted";
      } else {
        response->accepted = false;
        response->message = validation_message;
        current_state_ = "ERROR";
        status_message_ = validation_message;
      }
    }

    if (valid) {
      RCLCPP_DEBUG(get_logger(), "Accepted service target x=%.3f z=%.3f", request->x, request->z);
    } else {
      RCLCPP_WARN(get_logger(), "%s", validation_message.c_str());
    }
  }

  void timerCallback()
  {
    TimerOutputs outputs;
    const auto now_time = now();

    {
      std::lock_guard<std::mutex> lock(mutex_);

      const double dt = std::clamp((now_time - last_control_time_).seconds(), 1e-4, 0.1);
      last_control_time_ = now_time;

      const FeedbackHealth feedback_health = computeFeedbackHealthLocked(now_time);
      const CartesianPoint current_point = currentCartesianLocked();

      if (!tracker_initialized_) {
        initializeTrackerLocked(current_point);
      }

      TargetSource target_source = TargetSource::None;
      const auto active_target = activeTargetLocked(now_time, target_source);
      const CartesianPoint display_target = active_target.value_or(CartesianPoint{last_target_x_, last_target_z_});

      double cmd_q1 = hold_q1_;
      double cmd_q2 = hold_q2_;
      double cmd_vel_q1 = 0.0;
      double cmd_vel_q2 = 0.0;

      if (feedback_health.feedback_ok) {
        const CartesianPoint desired_target =
          active_target.value_or(CartesianPoint{smoothed_target_.x, smoothed_target_.z});

        smoothed_target_.x = advance_axis(
          smoothed_target_.x,
          smoothed_velocity_x_,
          desired_target.x,
          max_cart_vel_x_,
          max_cart_accel_,
          dt);
        smoothed_target_.z = advance_axis(
          smoothed_target_.z,
          smoothed_velocity_z_,
          desired_target.z,
          max_cart_vel_z_,
          max_cart_accel_,
          dt);

        std::string ik_message;
        if (solveUserTarget(smoothed_target_, cmd_q1, cmd_q2, ik_message)) {
          cmd_vel_q1 = std::clamp((cmd_q1 - last_command_q1_) / dt, -joint_vel_limit_, joint_vel_limit_);
          cmd_vel_q2 = std::clamp((cmd_q2 - last_command_q2_) / dt, -joint_vel_limit_, joint_vel_limit_);

          hold_q1_ = cmd_q1;
          hold_q2_ = cmd_q2;
          last_command_q1_ = cmd_q1;
          last_command_q2_ = cmd_q2;
          updateTrackingStateLocked(feedback_health, current_point, active_target, target_source);
        } else {
          smoothed_velocity_x_ = 0.0;
          smoothed_velocity_z_ = 0.0;
          current_state_ = "ERROR";
          status_message_ = ik_message;
          cmd_q1 = hold_q1_;
          cmd_q2 = hold_q2_;
          cmd_vel_q1 = 0.0;
          cmd_vel_q2 = 0.0;
        }
      } else {
        smoothed_velocity_x_ = 0.0;
        smoothed_velocity_z_ = 0.0;
        updateTrackingStateLocked(feedback_health, current_point, active_target, target_source);
      }

      outputs.position_command = {
        r2_arm_control::physicalToUrdf(cmd_q1),
        r2_arm_control::physicalToUrdf(cmd_q2)};
      outputs.velocity_command = {cmd_vel_q1, cmd_vel_q2};
      outputs.kp_command = {kp0_, kp1_};
      outputs.kd_command = {kd0_, kd1_};
      outputs.feedforward_command = {0.0, 0.0};
      outputs.state_msg = buildStateSnapshotLocked(current_point, display_target, feedback_health);
    }

    publishArray(pos_pub_, outputs.position_command);
    publishArray(vel_pub_, outputs.velocity_command);
    publishArray(kp_pub_, outputs.kp_command);
    publishArray(kd_pub_, outputs.kd_command);
    publishArray(ff_pub_, outputs.feedforward_command);
    state_pub_->publish(outputs.state_msg);
  }

  r2_arm_control::ArmParams params_;
  r2_arm_control::IkSolver solver_;

  double kp0_ {15.0};
  double kp1_ {15.0};
  double kd0_ {1.0};
  double kd1_ {1.0};
  double move_duration_ignored_ {2.0};
  bool elbow_up_ {true};
  double x_offset_ {0.0};
  double z_offset_ {0.0};
  double control_rate_hz_ {200.0};
  double target_timeout_sec_ {0.10};
  double max_cart_vel_x_ {0.60};
  double max_cart_vel_z_ {0.60};
  double max_cart_accel_ {2.50};
  double joint_vel_limit_ {2.0};
  double settle_tolerance_x_ {0.008};
  double settle_tolerance_z_ {0.008};
  int settle_cycles_ {8};
  double feedback_timeout_sec_ {0.05};
  double feedback_hard_timeout_sec_ {0.30};
  double feedback_startup_grace_sec_ {8.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kp_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ff_pub_;
  rclcpp::Publisher<r2_arm_control::msg::ArmMotionState>::SharedPtr state_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<r2_arm_control::msg::ArmCartesianTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr feedback_status_sub_;
  rclcpp::Service<r2_arm_control::srv::MoveToXZ>::SharedPtr move_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;

  bool have_shoulder_joint_ {false};
  bool have_elbow_joint_ {false};
  bool have_joint_state_ {false};
  bool tracker_initialized_ {false};
  bool feedback_status_received_ {false};
  bool latest_feedback_ok_ {true};
  bool feedback_fault_active_ {false};
  bool last_feedback_ok_state_ {true};

  double current_q1_ {0.0};
  double current_q2_ {0.0};
  double hold_q1_ {0.0};
  double hold_q2_ {0.0};
  double last_command_q1_ {0.0};
  double last_command_q2_ {0.0};
  double last_target_x_ {0.0};
  double last_target_z_ {0.0};
  double smoothed_velocity_x_ {0.0};
  double smoothed_velocity_z_ {0.0};

  CartesianPoint smoothed_target_ {};
  std::optional<CartesianPoint> stream_target_;
  std::optional<CartesianPoint> service_target_;
  std::uint8_t latest_stale_motor_count_ {0};
  int settle_counter_ {0};

  std::string current_state_ {"IDLE"};
  std::string status_message_ {"waiting for command"};

  rclcpp::Time startup_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Time last_stream_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_feedback_status_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time feedback_fault_started_at_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_invalid_target_log_time_ {0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmCommandServerNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
