#include "r2_arm_control/ik_solver.hpp"
#include "r2_arm_control/angle_mapping.hpp"
#include "r2_arm_control/msg/arm_motion_state.hpp"
#include "r2_arm_control/msg/mit_joint_command.hpp"
#include "r2_arm_control/srv/move_to_xz.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

class ArmCommandServerNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  ArmCommandServerNode()
  : Node("arm_command_server_node"),
    solver_(ik_params_)
  {
    declare_parameter("x_offset", 0.0);
    declare_parameter("z_offset", 0.0);
    declare_parameter("d1", 0.30);
    declare_parameter("d2", 0.30);
    declare_parameter("forearm_mount_offset_rad", 0.0);
    declare_parameter("q1_min", -1.80);
    declare_parameter("q1_max", 1.80);
    declare_parameter("q2_min", -1.80);
    declare_parameter("q2_max", 1.80);
    declare_parameter("elbow_rel_min", -1.3962634);
    declare_parameter("elbow_rel_max", 1.3962634);
    declare_parameter("elbow_up", true);
    declare_parameter("move_service_name", "/r2/arm/move_to_xz");
    declare_parameter("state_topic", "/r2/arm/state");
    declare_parameter("feedback_status_topic", "/r2/arm/feedback_status");
    declare_parameter("planning_group", "arm");
    declare_parameter("base_frame", "base_link");
    declare_parameter("end_effector_link", "tool_link");
    declare_parameter("shoulder_joint_name", "shoulder_joint");
    declare_parameter("elbow_joint_name", "elbow_joint");
    declare_parameter("joint_states_topic", "/joint_states");
    declare_parameter("joint_state_timeout_sec", 0.50);
    declare_parameter("executor_mode", "mit_native_5var");
    declare_parameter("trajectory_controller_action_name", "/arm_trajectory_controller/follow_joint_trajectory");
    declare_parameter("joint_vel_limit", 2.0);
    declare_parameter("joint_acc_limit", 2.0);
    declare_parameter("min_motion_duration_sec", 0.40);
    declare_parameter("max_motion_duration_sec", 4.00);
    declare_parameter("trajectory_sample_period_sec", 0.02);
    declare_parameter("trajectory_start_delay_sec", 0.05);
    declare_parameter("trajectory_execution_timeout_margin_sec", 1.50);
    declare_parameter("direct_goal_tolerance_rad", 0.02);
    declare_parameter("cartesian_goal_tolerance_m", 0.01);
    declare_parameter("ee_linear_speed_limit", 0.08);
    declare_parameter("ee_linear_acc_limit", 0.20);
    declare_parameter("joint_velocity_smoothing", 0.35);
    declare_parameter("jacobian_damping", 0.02);
    declare_parameter("cartesian_correction_gain", 0.80);
    declare_parameter("cartesian_correction_max_step_m", 0.015);
    declare_parameter("cartesian_correction_max_attempts", 2);
    declare_parameter("cartesian_correction_interval_sec", 0.15);
    declare_parameter("shoulder_kp_extension_gain", 0.0);
    declare_parameter("shoulder_kd_extension_gain", 0.0);
    declare_parameter("elbow_kp_extension_gain", 0.0);
    declare_parameter("elbow_kd_extension_gain", 0.0);
    declare_parameter("shoulder_coulomb_friction", 0.0);
    declare_parameter("shoulder_viscous_friction", 0.0);
    declare_parameter("shoulder_friction_velocity_scale", 0.05);
    declare_parameter("elbow_coulomb_friction", 0.0);
    declare_parameter("elbow_viscous_friction", 0.0);
    declare_parameter("elbow_friction_velocity_scale", 0.05);
    declare_parameter("kp_joint0", 40.0);
    declare_parameter("kp_joint1", 40.0);
    declare_parameter("kd_joint0", 2.0);
    declare_parameter("kd_joint1", 2.0);
    declare_parameter("feedforward_joint0", 0.0);
    declare_parameter("feedforward_joint1", 0.0);
    declare_parameter("shoulder_gravity_comp_gain", 0.0);
    declare_parameter("forearm_gravity_comp_gain", 0.0);
    declare_parameter("elbow_gravity_comp_gain", 0.0);
    declare_parameter("mit_settle_timeout_sec", 0.60);

    x_offset_ = get_parameter("x_offset").as_double();
    z_offset_ = get_parameter("z_offset").as_double();
    d1_ = get_parameter("d1").as_double();
    d2_ = get_parameter("d2").as_double();
    ik_params_.d1 = d1_;
    ik_params_.d2 = d2_;
    ik_params_.forearm_mount_offset_rad = get_parameter("forearm_mount_offset_rad").as_double();
    ik_params_.q1_min = get_parameter("q1_min").as_double();
    ik_params_.q1_max = get_parameter("q1_max").as_double();
    ik_params_.q2_min = get_parameter("q2_min").as_double();
    ik_params_.q2_max = get_parameter("q2_max").as_double();
    ik_params_.elbow_rel_min = get_parameter("elbow_rel_min").as_double();
    ik_params_.elbow_rel_max = get_parameter("elbow_rel_max").as_double();
    elbow_up_ = get_parameter("elbow_up").as_bool();
    solver_ = r2_arm_control::IkSolver(ik_params_);

    move_service_name_ = get_parameter("move_service_name").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    feedback_status_topic_ = get_parameter("feedback_status_topic").as_string();
    planning_group_ = get_parameter("planning_group").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    end_effector_link_ = get_parameter("end_effector_link").as_string();
    shoulder_joint_name_ = get_parameter("shoulder_joint_name").as_string();
    elbow_joint_name_ = get_parameter("elbow_joint_name").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    joint_state_timeout_sec_ = std::max(0.05, get_parameter("joint_state_timeout_sec").as_double());
    executor_mode_ = get_parameter("executor_mode").as_string();
    trajectory_action_name_ = get_parameter("trajectory_controller_action_name").as_string();
    joint_vel_limit_ = std::max(0.05, get_parameter("joint_vel_limit").as_double());
    joint_acc_limit_ = std::max(0.05, get_parameter("joint_acc_limit").as_double());
    min_motion_duration_sec_ =
      std::max(0.10, get_parameter("min_motion_duration_sec").as_double());
    max_motion_duration_sec_ =
      std::max(min_motion_duration_sec_, get_parameter("max_motion_duration_sec").as_double());
    trajectory_sample_period_sec_ =
      std::clamp(get_parameter("trajectory_sample_period_sec").as_double(), 0.01, 0.20);
    trajectory_start_delay_sec_ =
      std::clamp(get_parameter("trajectory_start_delay_sec").as_double(), 0.0, 0.50);
    trajectory_execution_timeout_margin_sec_ =
      std::max(0.20, get_parameter("trajectory_execution_timeout_margin_sec").as_double());
    direct_goal_tolerance_rad_ =
      std::clamp(get_parameter("direct_goal_tolerance_rad").as_double(), 1e-4, 0.20);
    cartesian_goal_tolerance_m_ =
      std::clamp(get_parameter("cartesian_goal_tolerance_m").as_double(), 0.001, 0.10);
    ee_linear_speed_limit_ =
      std::max(0.005, get_parameter("ee_linear_speed_limit").as_double());
    ee_linear_acc_limit_ =
      std::max(0.01, get_parameter("ee_linear_acc_limit").as_double());
    joint_velocity_smoothing_ =
      std::clamp(get_parameter("joint_velocity_smoothing").as_double(), 0.0, 0.95);
    jacobian_damping_ =
      std::clamp(get_parameter("jacobian_damping").as_double(), 1e-5, 0.20);
    cartesian_correction_gain_ =
      std::clamp(get_parameter("cartesian_correction_gain").as_double(), 0.0, 2.0);
    cartesian_correction_max_step_m_ =
      std::clamp(get_parameter("cartesian_correction_max_step_m").as_double(), 0.0, 0.10);
    cartesian_correction_max_attempts_ =
      std::max<int>(0, get_parameter("cartesian_correction_max_attempts").as_int());
    cartesian_correction_interval_sec_ =
      std::clamp(get_parameter("cartesian_correction_interval_sec").as_double(), 0.02, 1.0);
    shoulder_kp_extension_gain_ =
      std::max(0.0, get_parameter("shoulder_kp_extension_gain").as_double());
    shoulder_kd_extension_gain_ =
      std::max(0.0, get_parameter("shoulder_kd_extension_gain").as_double());
    elbow_kp_extension_gain_ =
      std::max(0.0, get_parameter("elbow_kp_extension_gain").as_double());
    elbow_kd_extension_gain_ =
      std::max(0.0, get_parameter("elbow_kd_extension_gain").as_double());
    shoulder_coulomb_friction_ = get_parameter("shoulder_coulomb_friction").as_double();
    shoulder_viscous_friction_ = get_parameter("shoulder_viscous_friction").as_double();
    shoulder_friction_velocity_scale_ =
      std::clamp(get_parameter("shoulder_friction_velocity_scale").as_double(), 1e-4, 1.0);
    elbow_coulomb_friction_ = get_parameter("elbow_coulomb_friction").as_double();
    elbow_viscous_friction_ = get_parameter("elbow_viscous_friction").as_double();
    elbow_friction_velocity_scale_ =
      std::clamp(get_parameter("elbow_friction_velocity_scale").as_double(), 1e-4, 1.0);
    kp_cmd_ = {
      std::max(0.0, get_parameter("kp_joint0").as_double()),
      std::max(0.0, get_parameter("kp_joint1").as_double())
    };
    kd_cmd_ = {
      std::max(0.0, get_parameter("kd_joint0").as_double()),
      std::max(0.0, get_parameter("kd_joint1").as_double())
    };
    feedforward_cmd_ = {
      get_parameter("feedforward_joint0").as_double(),
      get_parameter("feedforward_joint1").as_double()
    };
    shoulder_gravity_comp_gain_ = get_parameter("shoulder_gravity_comp_gain").as_double();
    forearm_gravity_comp_gain_ = get_parameter("forearm_gravity_comp_gain").as_double();
    elbow_gravity_comp_gain_ = get_parameter("elbow_gravity_comp_gain").as_double();
    mit_settle_timeout_sec_ =
      std::clamp(get_parameter("mit_settle_timeout_sec").as_double(), 0.10, 3.0);

    state_pub_ = create_publisher<r2_arm_control::msg::ArmMotionState>(state_topic_, 10);
    mit_cmd_pub_ = create_publisher<r2_arm_control::msg::MitJointCommand>(
      "/arm_mit_controller/commands", 10);
    move_service_ = create_service<r2_arm_control::srv::MoveToXZ>(
      move_service_name_,
      std::bind(
        &ArmCommandServerNode::handleMoveRequest,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      20,
      std::bind(&ArmCommandServerNode::jointStateCallback, this, std::placeholders::_1));
    feedback_status_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      feedback_status_topic_,
      10,
      std::bind(&ArmCommandServerNode::feedbackStatusCallback, this, std::placeholders::_1));

    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, trajectory_action_name_);

    publishState(
      "WAITING",
      "Arm executor is waiting for controller feedback.",
      0.0,
      0.0,
      nullptr);
  }

  void initialize()
  {
    if (isMitNativeExecutor()) {
      publishState(
        "READY",
        "MIT native 5-variable executor is ready.",
        last_target_x_,
        last_target_z_,
        nullptr);
      RCLCPP_INFO(
        get_logger(),
        "MIT native executor ready. joint_states=%s executor_mode=%s kp=[%.3f, %.3f] kd=[%.3f, %.3f]",
        joint_states_topic_.c_str(),
        executor_mode_.c_str(),
        kp_cmd_[0], kp_cmd_[1], kd_cmd_[0], kd_cmd_[1]);
      return;
    }

    if (trajectory_client_->wait_for_action_server(std::chrono::seconds(5))) {
      publishState(
        "READY",
        "Direct palletizing executor connected to joint trajectory controller.",
        last_target_x_,
        last_target_z_,
        nullptr);
      RCLCPP_INFO(
        get_logger(),
        "Direct palletizing executor ready. action=%s joint_states=%s executor_mode=%s",
        trajectory_action_name_.c_str(),
        joint_states_topic_.c_str(),
        executor_mode_.c_str());
    } else {
      publishState(
        "WAITING",
        "Joint trajectory action server not ready yet.",
        last_target_x_,
        last_target_z_,
        nullptr);
      RCLCPP_WARN(
        get_logger(),
        "Joint trajectory action server %s not ready yet. Service will keep waiting on demand.",
        trajectory_action_name_.c_str());
    }
  }

private:
  struct CartesianPoint
  {
    double x {std::numeric_limits<double>::quiet_NaN()};
    double z {std::numeric_limits<double>::quiet_NaN()};
  };

  struct CartesianMotionProfile
  {
    double length {0.0};
    double accel {0.0};
    double cruise_speed {0.0};
    double peak_speed {0.0};
    double accel_time {0.0};
    double cruise_time {0.0};
    double total_time {0.0};
  };

  struct JointGoal
  {
    double shoulder {0.0};
    double forearm {0.0};
    bool used_elbow_up {true};
  };

  struct MeasuredState
  {
    bool valid {false};
    rclcpp::Time received_at {0, 0, RCL_ROS_TIME};
    double shoulder_abs {0.0};
    double elbow_rel {0.0};
    double forearm_abs {0.0};
    double x {std::numeric_limits<double>::quiet_NaN()};
    double z {std::numeric_limits<double>::quiet_NaN()};
  };

  struct DirectExecutionResult
  {
    bool success {false};
    std::string message;
  };

  static double absoluteToRelativeElbow(double shoulder_abs, double forearm_abs)
  {
    return r2_arm_control::normalizeAngle(forearm_abs - shoulder_abs);
  }

  static double radToDeg(double angle_rad)
  {
    return angle_rad * 180.0 / M_PI;
  }

  static double shortestAngularDistance(double from, double to)
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  static builtin_interfaces::msg::Duration toBuiltinDuration(double seconds)
  {
    const auto nanoseconds = static_cast<int64_t>(std::llround(std::max(0.0, seconds) * 1e9));
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return duration;
  }

  static double jointGoalDistanceSquared(
    const JointGoal & candidate,
    double reference_shoulder,
    double reference_forearm)
  {
    const double shoulder_error =
      shortestAngularDistance(reference_shoulder, candidate.shoulder);
    const double forearm_error =
      shortestAngularDistance(reference_forearm, candidate.forearm);
    return shoulder_error * shoulder_error + forearm_error * forearm_error;
  }

  static double cartesianDistance(const CartesianPoint & from, const CartesianPoint & to)
  {
    return std::hypot(to.x - from.x, to.z - from.z);
  }

  static CartesianPoint cartesianError(
    const CartesianPoint & measured,
    const CartesianPoint & target)
  {
    return {target.x - measured.x, target.z - measured.z};
  }

  static std::string formatAngleTriplet(
    const std::string & prefix,
    double shoulder_abs,
    double forearm_abs,
    double elbow_rel)
  {
    std::ostringstream oss;
    oss << prefix
        << " shoulder_abs=" << radToDeg(shoulder_abs)
        << " deg, forearm_abs=" << radToDeg(forearm_abs)
        << " deg, elbow_rel=" << radToDeg(elbow_rel) << " deg";
    return oss.str();
  }

  double measuredAgeSecLocked() const
  {
    if (!measured_state_.valid) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, (now() - measured_state_.received_at).seconds());
  }

  bool measuredStateFreshLocked() const
  {
    return measured_state_.valid && measuredAgeSecLocked() <= joint_state_timeout_sec_;
  }

  bool feedbackHealthyLocked() const
  {
    const bool measurement_fresh = measuredStateFreshLocked();
    if (have_feedback_status_) {
      return measurement_fresh && feedback_ok_ && !hard_timeout_active_;
    }
    return measurement_fresh && !hard_timeout_active_;
  }

  CartesianPoint currentCartesianLocked() const
  {
    if (measured_state_.valid) {
      return {measured_state_.x - x_offset_, measured_state_.z - z_offset_};
    }
    return {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
    };
  }

  void populateResponseObservability(
    const JointGoal * target_goal,
    const MeasuredState & measured,
    r2_arm_control::srv::MoveToXZ::Response & response) const
  {
    response.feedback_ok = feedbackHealthyLocked();
    response.hard_timeout_active = hard_timeout_active_;
    response.measured_joint_state_age_sec = measured.valid
      ? measuredAgeSecLocked()
      : std::numeric_limits<double>::infinity();
    response.executor_mode = executor_mode_;

    const double target_elbow_rel = target_goal
      ? absoluteToRelativeElbow(target_goal->shoulder, target_goal->forearm)
      : std::numeric_limits<double>::quiet_NaN();
    response.target_shoulder_abs_deg = target_goal
      ? radToDeg(target_goal->shoulder)
      : std::numeric_limits<double>::quiet_NaN();
    response.target_forearm_abs_deg = target_goal
      ? radToDeg(target_goal->forearm)
      : std::numeric_limits<double>::quiet_NaN();
    response.target_elbow_rel_deg = std::isfinite(target_elbow_rel)
      ? radToDeg(target_elbow_rel)
      : std::numeric_limits<double>::quiet_NaN();

    response.measured_shoulder_abs_deg = measured.valid
      ? radToDeg(measured.shoulder_abs)
      : std::numeric_limits<double>::quiet_NaN();
    response.measured_forearm_abs_deg = measured.valid
      ? radToDeg(measured.forearm_abs)
      : std::numeric_limits<double>::quiet_NaN();
    response.measured_elbow_rel_deg = measured.valid
      ? radToDeg(measured.elbow_rel)
      : std::numeric_limits<double>::quiet_NaN();
    response.measured_x = measured.valid
      ? measured.x - x_offset_
      : std::numeric_limits<double>::quiet_NaN();
    response.measured_z = measured.valid
      ? measured.z - z_offset_
      : std::numeric_limits<double>::quiet_NaN();
  }

  void publishState(
    const std::string & state,
    const std::string & message,
    double target_x,
    double target_z,
    const JointGoal * target_goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishStateLocked(state, message, target_x, target_z, target_goal);
  }

  void publishStateLocked(
    const std::string & state,
    const std::string & message,
    double target_x,
    double target_z,
    const JointGoal * target_goal)
  {
    const CartesianPoint current = currentCartesianLocked();
    last_target_x_ = target_x;
    last_target_z_ = target_z;

    const double target_elbow_rel = target_goal
      ? absoluteToRelativeElbow(target_goal->shoulder, target_goal->forearm)
      : std::numeric_limits<double>::quiet_NaN();

    r2_arm_control::msg::ArmMotionState msg;
    msg.state = state;
    msg.current_x = current.x;
    msg.current_z = current.z;
    msg.target_x = target_x;
    msg.target_z = target_z;
    msg.feedback_ok = feedbackHealthyLocked();
    msg.stale_motor_count = have_feedback_status_
      ? stale_motor_count_
      : static_cast<uint8_t>(measuredStateFreshLocked() ? 0 : 1);
    msg.hard_timeout_active = hard_timeout_active_;
    msg.measured_joint_state_valid = measured_state_.valid;
    msg.measured_joint_state_age_sec = measured_state_.valid
      ? measuredAgeSecLocked()
      : std::numeric_limits<double>::infinity();
    msg.target_shoulder_abs_deg = target_goal
      ? radToDeg(target_goal->shoulder)
      : std::numeric_limits<double>::quiet_NaN();
    msg.target_forearm_abs_deg = target_goal
      ? radToDeg(target_goal->forearm)
      : std::numeric_limits<double>::quiet_NaN();
    msg.target_elbow_rel_deg = std::isfinite(target_elbow_rel)
      ? radToDeg(target_elbow_rel)
      : std::numeric_limits<double>::quiet_NaN();
    msg.measured_shoulder_abs_deg = measured_state_.valid
      ? radToDeg(measured_state_.shoulder_abs)
      : std::numeric_limits<double>::quiet_NaN();
    msg.measured_forearm_abs_deg = measured_state_.valid
      ? radToDeg(measured_state_.forearm_abs)
      : std::numeric_limits<double>::quiet_NaN();
    msg.measured_elbow_rel_deg = measured_state_.valid
      ? radToDeg(measured_state_.elbow_rel)
      : std::numeric_limits<double>::quiet_NaN();
    msg.executor_mode = executor_mode_;
    msg.message = message;
    state_pub_->publish(msg);
  }

  bool validateTarget(double requested_x, double requested_z, std::string & message) const
  {
    const double shifted_x = requested_x + x_offset_;
    const double shifted_z = requested_z + z_offset_;

    if (!std::isfinite(shifted_x) || !std::isfinite(shifted_z)) {
      message = "Target rejected: x and z must be finite numbers.";
      return false;
    }

    if (!solver_.isReachable(shifted_x, shifted_z)) {
      const double radius = std::hypot(shifted_x, shifted_z);
      const double min_radius = std::fabs(d1_ - d2_);
      const double max_radius = d1_ + d2_;
      std::ostringstream oss;
      oss << "Target rejected: x=" << requested_x << ", z=" << requested_z
          << " is outside the palletizing workspace. Reach radius=" << radius
          << " m, allowed range=[" << min_radius << ", " << max_radius << "] m.";
      message = oss.str();
      return false;
    }

    message = "target accepted";
    return true;
  }

  bool solveJointGoal(double requested_x, double requested_z, JointGoal & goal, std::string & message) const
  {
    const double shifted_x = requested_x + x_offset_;
    const double shifted_z = requested_z + z_offset_;

    const auto preferred = solver_.solve(shifted_x, shifted_z, elbow_up_);
    const auto alternate = solver_.solve(shifted_x, shifted_z, !elbow_up_);

    if (preferred.success && alternate.success) {
      const JointGoal preferred_goal {preferred.q1, preferred.q2, elbow_up_};
      const JointGoal alternate_goal {alternate.q1, alternate.q2, !elbow_up_};

      if (has_last_joint_goal_) {
        const double preferred_distance =
          jointGoalDistanceSquared(preferred_goal, last_goal_shoulder_, last_goal_forearm_);
        const double alternate_distance =
          jointGoalDistanceSquared(alternate_goal, last_goal_shoulder_, last_goal_forearm_);

        goal = preferred_distance <= alternate_distance ? preferred_goal : alternate_goal;
        message =
          goal.used_elbow_up == elbow_up_
          ? "Solved with preferred branch and continuity check."
          : "Solved with alternate branch to keep joint motion continuous.";
        return true;
      }

      goal = preferred_goal;
      message = elbow_up_ ? "Solved with elbow_up branch." : "Solved with elbow_down branch.";
      return true;
    }

    if (preferred.success) {
      goal = JointGoal {preferred.q1, preferred.q2, elbow_up_};
      message = elbow_up_ ? "Solved with elbow_up branch." : "Solved with elbow_down branch.";
      return true;
    }

    if (alternate.success) {
      goal = JointGoal {alternate.q1, alternate.q2, !elbow_up_};
      message =
        elbow_up_ ? "Preferred elbow_up branch failed, using elbow_down."
                  : "Preferred elbow_down branch failed, using elbow_up.";
      return true;
    }

    std::ostringstream oss;
    oss << "Target rejected: x=" << requested_x << ", z=" << requested_z
        << " is reachable in radius, but no valid IK solution satisfies joint limits.";
    message = oss.str();
    return false;
  }

  bool solveContinuousJointGoal(
    double x,
    double z,
    double reference_shoulder,
    double reference_forearm,
    JointGoal & goal) const
  {
    const auto preferred = solver_.solve(x, z, elbow_up_);
    const auto alternate = solver_.solve(x, z, !elbow_up_);

    if (!preferred.success && !alternate.success) {
      return false;
    }
    if (preferred.success && !alternate.success) {
      goal = JointGoal {preferred.q1, preferred.q2, elbow_up_};
      return true;
    }
    if (!preferred.success && alternate.success) {
      goal = JointGoal {alternate.q1, alternate.q2, !elbow_up_};
      return true;
    }

    const JointGoal preferred_goal {preferred.q1, preferred.q2, elbow_up_};
    const JointGoal alternate_goal {alternate.q1, alternate.q2, !elbow_up_};
    const double preferred_distance =
      jointGoalDistanceSquared(preferred_goal, reference_shoulder, reference_forearm);
    const double alternate_distance =
      jointGoalDistanceSquared(alternate_goal, reference_shoulder, reference_forearm);

    goal = preferred_distance <= alternate_distance ? preferred_goal : alternate_goal;
    return true;
  }

  bool validateCartesianPath(
    const CartesianPoint & start,
    const CartesianPoint & target,
    const MeasuredState & measured,
    std::string & message) const
  {
    const double length = cartesianDistance(start, target);
    if (length < 1e-6) {
      message = "Cartesian path length is negligible.";
      return true;
    }

    const std::size_t sample_count = std::max<std::size_t>(
      10,
      static_cast<std::size_t>(std::ceil(length / 0.005)));
    double reference_shoulder = measured.shoulder_abs;
    double reference_forearm = measured.forearm_abs;

    for (std::size_t i = 1; i <= sample_count; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(sample_count);
      const double x = start.x + (target.x - start.x) * alpha;
      const double z = start.z + (target.z - start.z) * alpha;

      JointGoal sample_goal;
      if (!solveContinuousJointGoal(x, z, reference_shoulder, reference_forearm, sample_goal)) {
        std::ostringstream oss;
        oss << "Cartesian straight-line path leaves the reachable IK workspace at x="
            << (x - x_offset_) << ", z=" << (z - z_offset_) << ".";
        message = oss.str();
        return false;
      }

      reference_shoulder = sample_goal.shoulder;
      reference_forearm = sample_goal.forearm;
    }

    message = "cartesian path accepted";
    return true;
  }

  CartesianMotionProfile buildCartesianMotionProfile(
    const CartesianPoint & start,
    const CartesianPoint & target) const
  {
    CartesianMotionProfile profile;
    profile.length = cartesianDistance(start, target);
    profile.accel = ee_linear_acc_limit_;
    profile.cruise_speed = ee_linear_speed_limit_;

    if (profile.length < 1e-6) {
      return profile;
    }

    const double accel_distance =
      (profile.cruise_speed * profile.cruise_speed) / (2.0 * profile.accel);
    if (2.0 * accel_distance >= profile.length) {
      profile.peak_speed = std::sqrt(std::max(0.0, profile.length * profile.accel));
      profile.accel_time = profile.peak_speed / profile.accel;
      profile.cruise_time = 0.0;
      profile.total_time = 2.0 * profile.accel_time;
      return profile;
    }

    profile.peak_speed = profile.cruise_speed;
    profile.accel_time = profile.cruise_speed / profile.accel;
    profile.cruise_time =
      (profile.length - 2.0 * accel_distance) / profile.cruise_speed;
    profile.total_time = 2.0 * profile.accel_time + profile.cruise_time;
    return profile;
  }

  std::pair<double, double> sampleCartesianMotionProfile(
    const CartesianMotionProfile & profile,
    double time_sec) const
  {
    if (profile.length < 1e-6 || profile.total_time <= 0.0) {
      return {0.0, 0.0};
    }

    const double clamped_time = std::clamp(time_sec, 0.0, profile.total_time);
    const double accel_distance =
      0.5 * profile.accel * profile.accel_time * profile.accel_time;

    if (clamped_time <= profile.accel_time) {
      return {
        0.5 * profile.accel * clamped_time * clamped_time,
        profile.accel * clamped_time
      };
    }

    if (clamped_time <= profile.accel_time + profile.cruise_time) {
      const double cruise_elapsed = clamped_time - profile.accel_time;
      return {
        accel_distance + profile.peak_speed * cruise_elapsed,
        profile.peak_speed
      };
    }

    const double decel_elapsed = clamped_time - profile.accel_time - profile.cruise_time;
    const double traveled =
      accel_distance +
      profile.peak_speed * profile.cruise_time +
      profile.peak_speed * decel_elapsed -
      0.5 * profile.accel * decel_elapsed * decel_elapsed;
    const double velocity =
      std::max(0.0, profile.peak_speed - profile.accel * decel_elapsed);
    return {std::min(profile.length, traveled), velocity};
  }

  bool solveJointVelocityForCartesian(
    double shoulder_abs,
    double forearm_abs,
    double cartesian_vx,
    double cartesian_vz,
    double & shoulder_vel,
    double & forearm_vel) const
  {
    const double forearm_world = forearm_abs + ik_params_.forearm_mount_offset_rad;
    const double j11 = d1_ * std::cos(shoulder_abs);
    const double j12 = -d2_ * std::sin(forearm_world);
    const double j21 = -d1_ * std::sin(shoulder_abs);
    const double j22 = -d2_ * std::cos(forearm_world);

    const double lambda2 = jacobian_damping_ * jacobian_damping_;
    const double a11 = j11 * j11 + j12 * j12 + lambda2;
    const double a12 = j11 * j21 + j12 * j22;
    const double a22 = j21 * j21 + j22 * j22 + lambda2;
    const double det = a11 * a22 - a12 * a12;
    if (std::fabs(det) < 1e-9) {
      return false;
    }

    const double inv_a11 = a22 / det;
    const double inv_a12 = -a12 / det;
    const double inv_a22 = a11 / det;
    const double w1 = inv_a11 * cartesian_vx + inv_a12 * cartesian_vz;
    const double w2 = inv_a12 * cartesian_vx + inv_a22 * cartesian_vz;

    shoulder_vel = j11 * w1 + j21 * w2;
    forearm_vel = j12 * w1 + j22 * w2;
    return std::isfinite(shoulder_vel) && std::isfinite(forearm_vel);
  }

  bool computeCorrectedJointGoal(
    const MeasuredState & measured,
    const CartesianPoint & nominal_target,
    JointGoal & corrected_goal,
    CartesianPoint & corrected_target) const
  {
    const CartesianPoint measured_cartesian {measured.x, measured.z};
    const CartesianPoint error = cartesianError(measured_cartesian, nominal_target);
    const double error_norm = std::hypot(error.x, error.z);
    if (error_norm <= cartesian_goal_tolerance_m_ || cartesian_correction_gain_ <= 0.0) {
      return false;
    }

    double correction_scale = cartesian_correction_gain_;
    if (cartesian_correction_max_step_m_ > 0.0) {
      const double requested_step = correction_scale * error_norm;
      if (requested_step > cartesian_correction_max_step_m_) {
        correction_scale = cartesian_correction_max_step_m_ / error_norm;
      }
    }

    for (int attempt = 0; attempt < 4; ++attempt) {
      corrected_target.x = nominal_target.x + error.x * correction_scale;
      corrected_target.z = nominal_target.z + error.z * correction_scale;
      if (solveContinuousJointGoal(
            corrected_target.x,
            corrected_target.z,
            measured.shoulder_abs,
            measured.forearm_abs,
            corrected_goal))
      {
        return true;
      }
      correction_scale *= 0.5;
    }

    return false;
  }

  double computeMotionDuration(const MeasuredState & start, const JointGoal & goal) const
  {
    const double shoulder_delta = std::fabs(shortestAngularDistance(start.shoulder_abs, goal.shoulder));
    const double forearm_delta = std::fabs(shortestAngularDistance(start.forearm_abs, goal.forearm));
    const double max_delta = std::max(shoulder_delta, forearm_delta);
    if (max_delta < direct_goal_tolerance_rad_) {
      return 0.0;
    }

    const double velocity_duration = 2.0 * max_delta / joint_vel_limit_;
    const double acceleration_duration = 2.5 * std::sqrt(max_delta / joint_acc_limit_);
    return std::clamp(
      std::max({min_motion_duration_sec_, velocity_duration, acceleration_duration}),
      min_motion_duration_sec_,
      max_motion_duration_sec_);
  }

  trajectory_msgs::msg::JointTrajectory buildDirectTrajectory(
    const MeasuredState & start,
    const JointGoal & goal,
    double duration_sec) const
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {shoulder_joint_name_, elbow_joint_name_};
    trajectory.header.stamp = now() + rclcpp::Duration::from_seconds(trajectory_start_delay_sec_);

    const double shoulder_delta = shortestAngularDistance(start.shoulder_abs, goal.shoulder);
    const double forearm_delta = shortestAngularDistance(start.forearm_abs, goal.forearm);
    const std::size_t segment_count = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(std::ceil(duration_sec / trajectory_sample_period_sec_)));

    for (std::size_t i = 0; i <= segment_count; ++i) {
      const double tau = segment_count == 0 ? 1.0 : static_cast<double>(i) / segment_count;
      const double tau2 = tau * tau;
      const double tau3 = tau2 * tau;
      const double tau4 = tau3 * tau;
      const double tau5 = tau4 * tau;

      const double s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
      const double ds_dt = duration_sec > 0.0
        ? (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / duration_sec
        : 0.0;

      const double shoulder_abs = start.shoulder_abs + shoulder_delta * s;
      const double forearm_abs = start.forearm_abs + forearm_delta * s;
      const double shoulder_vel = shoulder_delta * ds_dt;
      const double forearm_vel = forearm_delta * ds_dt;
      const double elbow_rel = absoluteToRelativeElbow(shoulder_abs, forearm_abs);
      const double elbow_rel_vel = forearm_vel - shoulder_vel;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {
        r2_arm_control::physicalToUrdf(shoulder_abs),
        r2_arm_control::physicalToUrdf(elbow_rel),
      };
      point.velocities = {shoulder_vel, elbow_rel_vel};
      point.time_from_start = toBuiltinDuration(duration_sec * tau);
      trajectory.points.push_back(point);
    }

    return trajectory;
  }

  DirectExecutionResult executeDirectTrajectory(
    const MeasuredState & start,
    const JointGoal & goal)
  {
    DirectExecutionResult execution;

    const double duration_sec = computeMotionDuration(start, goal);
    if (duration_sec <= 0.0) {
      execution.success = true;
      execution.message = "Already within direct goal tolerance.";
      return execution;
    }

    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(2))) {
      execution.message = "Direct executor could not reach the trajectory action server.";
      return execution;
    }

    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = buildDirectTrajectory(start, goal, duration_sec);
    goal_msg.goal_time_tolerance = toBuiltinDuration(trajectory_execution_timeout_margin_sec_);

    const auto goal_handle_future = trajectory_client_->async_send_goal(goal_msg);
    const auto send_status = goal_handle_future.wait_for(std::chrono::seconds(2));
    if (send_status != std::future_status::ready) {
      execution.message = "Timed out while sending the palletizing trajectory goal.";
      return execution;
    }

    const auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      execution.message = "Trajectory controller rejected the palletizing goal.";
      return execution;
    }

    const auto result_future = trajectory_client_->async_get_result(goal_handle);
    const auto timeout = std::chrono::duration<double>(
      duration_sec + trajectory_execution_timeout_margin_sec_ + trajectory_start_delay_sec_ + 1.0);
    const auto result_status = result_future.wait_for(timeout);
    if (result_status != std::future_status::ready) {
      trajectory_client_->async_cancel_goal(goal_handle);
      execution.message = "Timed out while waiting for the palletizing trajectory result.";
      return execution;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      execution.success = true;
      execution.message = "Direct palletizing trajectory succeeded.";
      return execution;
    }

    std::ostringstream oss;
    oss << "Direct palletizing trajectory failed";
    if (wrapped_result.result) {
      oss << " (error_code=" << wrapped_result.result->error_code;
      if (!wrapped_result.result->error_string.empty()) {
        oss << ", error_string=" << wrapped_result.result->error_string;
      }
      oss << ")";
    }
    execution.message = oss.str();
    return execution;
  }

  bool isMitNativeExecutor() const
  {
    return executor_mode_ == "mit_native_5var";
  }

  double computeExtensionRatio(double shoulder_abs, double forearm_abs) const
  {
    const auto fk = solver_.forward(shoulder_abs, forearm_abs);
    const double max_reach = std::max(1e-6, d1_ + d2_);
    return std::clamp(std::hypot(fk.x, fk.z) / max_reach, 0.0, 1.0);
  }

  std::array<double, 2> computeScheduledMitGains(double shoulder_abs, double forearm_abs) const
  {
    const double extension_ratio = computeExtensionRatio(shoulder_abs, forearm_abs);
    return {
      kp_cmd_[0] + shoulder_kp_extension_gain_ * extension_ratio,
      kp_cmd_[1] + elbow_kp_extension_gain_ * extension_ratio,
    };
  }

  std::array<double, 2> computeScheduledMitDamping(double shoulder_abs, double forearm_abs) const
  {
    const double extension_ratio = computeExtensionRatio(shoulder_abs, forearm_abs);
    return {
      kd_cmd_[0] + shoulder_kd_extension_gain_ * extension_ratio,
      kd_cmd_[1] + elbow_kd_extension_gain_ * extension_ratio,
    };
  }

  static double smoothFrictionTerm(double velocity, double coulomb, double viscous, double scale)
  {
    return coulomb * std::tanh(velocity / std::max(1e-6, scale)) + viscous * velocity;
  }

  std::vector<double> computeMitFeedforward(
    double shoulder_abs,
    double forearm_abs,
    double shoulder_vel,
    double forearm_vel) const
  {
    return {
      feedforward_cmd_[0] +
        shoulder_gravity_comp_gain_ * std::cos(shoulder_abs) +
        forearm_gravity_comp_gain_ * std::cos(forearm_abs) +
        smoothFrictionTerm(
          shoulder_vel,
          shoulder_coulomb_friction_,
          shoulder_viscous_friction_,
          shoulder_friction_velocity_scale_),
      feedforward_cmd_[1] +
        elbow_gravity_comp_gain_ * std::cos(forearm_abs) +
        smoothFrictionTerm(
          forearm_vel,
          elbow_coulomb_friction_,
          elbow_viscous_friction_,
          elbow_friction_velocity_scale_),
    };
  }

  void publishMitCommand(
    double shoulder_abs,
    double forearm_abs,
    double shoulder_vel,
    double forearm_vel)
  {
    const double elbow_rel = absoluteToRelativeElbow(shoulder_abs, forearm_abs);
    const double elbow_rel_vel = forearm_vel - shoulder_vel;
    const auto effort = computeMitFeedforward(
      shoulder_abs, forearm_abs, shoulder_vel, forearm_vel);
    const auto scheduled_kp = computeScheduledMitGains(shoulder_abs, forearm_abs);
    const auto scheduled_kd = computeScheduledMitDamping(shoulder_abs, forearm_abs);
    r2_arm_control::msg::MitJointCommand msg;
    msg.joint_names = {shoulder_joint_name_, elbow_joint_name_};
    msg.position = {
      r2_arm_control::physicalToUrdf(shoulder_abs),
      r2_arm_control::physicalToUrdf(elbow_rel),
    };
    msg.velocity = {shoulder_vel, elbow_rel_vel};
    msg.effort = effort;
    msg.kp = {scheduled_kp[0], scheduled_kp[1]};
    msg.kd = {scheduled_kd[0], scheduled_kd[1]};
    mit_cmd_pub_->publish(msg);
  }

  DirectExecutionResult executeMitNativeTrajectory(
    const MeasuredState & start,
    const JointGoal & goal)
  {
    DirectExecutionResult execution;
    const CartesianPoint start_cartesian {start.x, start.z};
    const auto target_fk = solver_.forward(goal.shoulder, goal.forearm);
    const CartesianPoint target_cartesian {target_fk.x, target_fk.z};
    const auto profile = buildCartesianMotionProfile(start_cartesian, target_cartesian);

    if (profile.length < 1e-6 || profile.total_time <= 0.0) {
      publishMitCommand(goal.shoulder, goal.forearm, 0.0, 0.0);
      execution.success = true;
      execution.message = "Already within MIT native Cartesian goal tolerance.";
      return execution;
    }

    std::string path_validation_message;
    if (!validateCartesianPath(start_cartesian, target_cartesian, start, path_validation_message)) {
      execution.message = path_validation_message;
      return execution;
    }

    const auto motion_start = std::chrono::steady_clock::now();
    const auto sample_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(trajectory_sample_period_sec_));
    const auto motion_end = motion_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(profile.total_time));
    auto next_tick = motion_start;
    double previous_shoulder = start.shoulder_abs;
    double previous_forearm = start.forearm_abs;
    double previous_shoulder_vel = 0.0;
    double previous_forearm_vel = 0.0;
    bool lost_fresh_feedback_during_motion = false;

    while (std::chrono::steady_clock::now() <= motion_end) {
      bool hard_timeout = false;
      bool stale_feedback = false;
      bool feedback_status_unhealthy = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        hard_timeout = hard_timeout_active_;
        stale_feedback = !measuredStateFreshLocked();
        feedback_status_unhealthy = have_feedback_status_ && !feedback_ok_;
      }

      if (hard_timeout) {
        execution.message = "MIT native execution aborted because hardware entered hard-timeout safe hold.";
        return execution;
      }
      if (feedback_status_unhealthy) {
        execution.message = "MIT native execution aborted because motor feedback status became unhealthy.";
        return execution;
      }
      if (stale_feedback) {
        lost_fresh_feedback_during_motion = true;
      }

      const double elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - motion_start).count();
      const auto profile_sample = sampleCartesianMotionProfile(profile, elapsed_sec);
      const double travel_distance = profile_sample.first;
      const double cartesian_speed = profile_sample.second;
      const double alpha = profile.length > 1e-9 ? travel_distance / profile.length : 1.0;
      const double cartesian_x =
        start_cartesian.x + (target_cartesian.x - start_cartesian.x) * alpha;
      const double cartesian_z =
        start_cartesian.z + (target_cartesian.z - start_cartesian.z) * alpha;

      JointGoal sample_goal;
      if (!solveContinuousJointGoal(
            cartesian_x, cartesian_z, previous_shoulder, previous_forearm, sample_goal))
      {
        std::ostringstream oss;
        oss << "MIT native Cartesian execution lost IK continuity at x="
            << (cartesian_x - x_offset_) << ", z=" << (cartesian_z - z_offset_) << ".";
        execution.message = oss.str();
        return execution;
      }

      const double direction_x =
        profile.length > 1e-9 ? (target_cartesian.x - start_cartesian.x) / profile.length : 0.0;
      const double direction_z =
        profile.length > 1e-9 ? (target_cartesian.z - start_cartesian.z) / profile.length : 0.0;
      const double cartesian_vx = direction_x * cartesian_speed;
      const double cartesian_vz = direction_z * cartesian_speed;

      double shoulder_vel = 0.0;
      double forearm_vel = 0.0;
      if (!solveJointVelocityForCartesian(
            sample_goal.shoulder,
            sample_goal.forearm,
            cartesian_vx,
            cartesian_vz,
            shoulder_vel,
            forearm_vel))
      {
        shoulder_vel = previous_shoulder_vel;
        forearm_vel = previous_forearm_vel;
      }

      const double smoothing_blend = 1.0 - joint_velocity_smoothing_;
      shoulder_vel =
        previous_shoulder_vel + smoothing_blend * (shoulder_vel - previous_shoulder_vel);
      forearm_vel =
        previous_forearm_vel + smoothing_blend * (forearm_vel - previous_forearm_vel);

      publishMitCommand(
        sample_goal.shoulder, sample_goal.forearm, shoulder_vel, forearm_vel);

      previous_shoulder = sample_goal.shoulder;
      previous_forearm = sample_goal.forearm;
      previous_shoulder_vel = shoulder_vel;
      previous_forearm_vel = forearm_vel;

      next_tick += sample_period;
      std::this_thread::sleep_until(next_tick);
    }

    const auto settle_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(mit_settle_timeout_sec_));
    const auto correction_interval =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(cartesian_correction_interval_sec_));
    bool stale_during_settle = false;
    JointGoal settle_goal = goal;
    int correction_attempts = 0;
    auto next_correction_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() <= settle_deadline) {
      MeasuredState measured_snapshot;
      bool hard_timeout = false;
      bool stale_feedback = false;
      bool feedback_status_unhealthy = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        measured_snapshot = measured_state_;
        hard_timeout = hard_timeout_active_;
        stale_feedback = !measuredStateFreshLocked();
        feedback_status_unhealthy = have_feedback_status_ && !feedback_ok_;
      }

      if (hard_timeout) {
        execution.message = "MIT native execution entered hard-timeout safe hold during settle.";
        return execution;
      }
      if (feedback_status_unhealthy) {
        execution.message = "MIT native execution aborted because motor feedback status became unhealthy during settle.";
        return execution;
      }

      publishMitCommand(settle_goal.shoulder, settle_goal.forearm, 0.0, 0.0);

      if (stale_feedback) {
        stale_during_settle = true;
        std::this_thread::sleep_for(sample_period);
        continue;
      }

      const double shoulder_error =
        std::fabs(shortestAngularDistance(measured_snapshot.shoulder_abs, settle_goal.shoulder));
      const double forearm_error =
        std::fabs(shortestAngularDistance(measured_snapshot.forearm_abs, settle_goal.forearm));
      const CartesianPoint measured_cartesian {measured_snapshot.x, measured_snapshot.z};
      const CartesianPoint cartesian_error =
        cartesianError(measured_cartesian, target_cartesian);
      const double cartesian_error_norm = std::hypot(cartesian_error.x, cartesian_error.z);
      if (cartesian_error_norm <= cartesian_goal_tolerance_m_) {
        execution.success = true;
        std::ostringstream oss;
        oss << (lost_fresh_feedback_during_motion
            ? "MIT native trajectory stream succeeded after temporary stale feedback recovered."
            : "MIT native trajectory succeeded.")
            << " Final Cartesian error=" << cartesian_error_norm
            << " m, settle shoulder error=" << shoulder_error
            << " rad, settle forearm error=" << forearm_error << " rad.";
        execution.message = oss.str();
        return execution;
      }

      if (
        correction_attempts < cartesian_correction_max_attempts_ &&
        cartesian_error_norm > cartesian_goal_tolerance_m_ &&
        std::chrono::steady_clock::now() >= next_correction_time)
      {
        JointGoal corrected_goal;
        CartesianPoint corrected_target;
        if (computeCorrectedJointGoal(
              measured_snapshot, target_cartesian, corrected_goal, corrected_target))
        {
          settle_goal = corrected_goal;
          ++correction_attempts;
          next_correction_time = std::chrono::steady_clock::now() + correction_interval;
          RCLCPP_INFO(
            get_logger(),
            "Applying Cartesian settle correction %d/%d: err=%.4f m -> corrected x=%.4f z=%.4f",
            correction_attempts,
            cartesian_correction_max_attempts_,
            cartesian_error_norm,
            corrected_target.x - x_offset_,
            corrected_target.z - z_offset_);
        }
      }

      std::this_thread::sleep_for(sample_period);
    }

    if (stale_during_settle || lost_fresh_feedback_during_motion) {
      execution.success = false;
      execution.message =
        "MIT native trajectory stream completed, but final settle could not be confirmed because joint feedback was stale.";
      return execution;
    }

    std::ostringstream oss;
    oss << "MIT native execution reached the hold phase but did not settle inside tolerance. "
        << "Cartesian tolerance=" << cartesian_goal_tolerance_m_
        << " m, joint tolerance=" << direct_goal_tolerance_rad_ << " rad.";
    execution.message = oss.str();
    return execution;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double shoulder_urdf = 0.0;
    double elbow_urdf = 0.0;
    bool have_shoulder = false;
    bool have_elbow = false;

    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      if (msg->name[i] == shoulder_joint_name_) {
        shoulder_urdf = msg->position[i];
        have_shoulder = true;
      } else if (msg->name[i] == elbow_joint_name_) {
        elbow_urdf = msg->position[i];
        have_elbow = true;
      }
    }

    if (!have_shoulder || !have_elbow) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for %s and %s in %s",
        shoulder_joint_name_.c_str(),
        elbow_joint_name_.c_str(),
        joint_states_topic_.c_str());
      return;
    }

    const double shoulder_abs = r2_arm_control::urdfToPhysical(shoulder_urdf);
    const double elbow_rel = r2_arm_control::urdfToPhysical(elbow_urdf);
    const double forearm_abs = shoulder_abs + elbow_rel;
    const auto fk = solver_.forward(shoulder_abs, forearm_abs);

    std::lock_guard<std::mutex> lock(mutex_);
    measured_state_.valid = true;
    measured_state_.received_at = now();
    measured_state_.shoulder_abs = shoulder_abs;
    measured_state_.elbow_rel = elbow_rel;
    measured_state_.forearm_abs = forearm_abs;
    measured_state_.x = fk.x;
    measured_state_.z = fk.z;
  }

  void feedbackStatusCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_feedback_status_ = true;
    feedback_ok_ = !msg->data.empty() && msg->data[0] != 0;
    stale_motor_count_ = msg->data.size() > 1 ? msg->data[1] : 0;
    hard_timeout_active_ = msg->data.size() > 2 && msg->data[2] != 0;
    startup_grace_active_ = msg->data.size() > 3 && msg->data[3] != 0;
  }

  void handleMoveRequest(
    const std::shared_ptr<r2_arm_control::srv::MoveToXZ::Request> request,
    std::shared_ptr<r2_arm_control::srv::MoveToXZ::Response> response)
  {
    const double requested_x = request->x;
    const double requested_z = request->z;
    std::string validation_message;
    JointGoal joint_goal;

    if (!validateTarget(requested_x, requested_z, validation_message)) {
      response->accepted = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        populateResponseObservability(nullptr, measured_state_, *response);
        publishStateLocked("ERROR", validation_message, requested_x, requested_z, nullptr);
      }
      response->message = validation_message;
      return;
    }

    if (!solveJointGoal(requested_x, requested_z, joint_goal, validation_message)) {
      response->accepted = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        populateResponseObservability(nullptr, measured_state_, *response);
        publishStateLocked("ERROR", validation_message, requested_x, requested_z, nullptr);
      }
      response->message = validation_message;
      return;
    }

    MeasuredState measured_snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      measured_snapshot = measured_state_;
      populateResponseObservability(&joint_goal, measured_snapshot, *response);
      if (hard_timeout_active_) {
        response->accepted = false;
        response->message = "Direct executor rejected the target because hardware is in hard-timeout safe hold.";
        publishStateLocked("ERROR", response->message, requested_x, requested_z, &joint_goal);
        return;
      }
      if (!measuredStateFreshLocked()) {
        response->accepted = false;
        response->message = "Direct executor rejected the target because measured joint feedback is stale.";
        publishStateLocked("ERROR", response->message, requested_x, requested_z, &joint_goal);
        return;
      }
      if (have_feedback_status_ && !feedback_ok_) {
        response->accepted = false;
        std::ostringstream oss;
        oss << "Direct executor rejected the target because motor feedback status is unhealthy"
            << " (stale_motor_count=" << static_cast<int>(stale_motor_count_)
            << ", startup_grace=" << (startup_grace_active_ ? "true" : "false") << ").";
        response->message = oss.str();
        publishStateLocked("ERROR", response->message, requested_x, requested_z, &joint_goal);
        return;
      }
      publishStateLocked(
        "EXECUTING",
        "Executing direct palletizing joint trajectory.",
        requested_x,
        requested_z,
        &joint_goal);
    }

    const auto execution = isMitNativeExecutor()
      ? executeMitNativeTrajectory(measured_snapshot, joint_goal)
      : executeDirectTrajectory(measured_snapshot, joint_goal);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      populateResponseObservability(&joint_goal, measured_state_, *response);
      if (!execution.success) {
        response->accepted = false;
        response->message = execution.message;
        publishStateLocked("ERROR", response->message, requested_x, requested_z, &joint_goal);
        return;
      }

      response->accepted = true;
      const double target_elbow_rel =
        absoluteToRelativeElbow(joint_goal.shoulder, joint_goal.forearm);
      std::ostringstream oss;
      oss << "Direct palletizing execution succeeded. "
          << formatAngleTriplet(
        "target",
        joint_goal.shoulder,
        joint_goal.forearm,
        target_elbow_rel);
      if (measured_state_.valid) {
        oss << " | "
            << formatAngleTriplet(
          "measured",
          measured_state_.shoulder_abs,
          measured_state_.forearm_abs,
          measured_state_.elbow_rel)
            << ", measured_x=" << (measured_state_.x - x_offset_)
            << " m, measured_z=" << (measured_state_.z - z_offset_) << " m";
      }
      oss << ", branch=" << (joint_goal.used_elbow_up ? "elbow_up" : "elbow_down") << ".";
      response->message = oss.str();
      has_last_joint_goal_ = true;
      last_goal_shoulder_ = joint_goal.shoulder;
      last_goal_forearm_ = joint_goal.forearm;
      publishStateLocked("REACHED", response->message, requested_x, requested_z, &joint_goal);
    }
  }

  double x_offset_ {0.0};
  double z_offset_ {0.0};
  double d1_ {0.30};
  double d2_ {0.30};
  bool elbow_up_ {true};
  double joint_state_timeout_sec_ {0.50};
  double joint_vel_limit_ {2.0};
  double joint_acc_limit_ {2.0};
  double min_motion_duration_sec_ {0.40};
  double max_motion_duration_sec_ {4.0};
  double trajectory_sample_period_sec_ {0.02};
  double trajectory_start_delay_sec_ {0.05};
  double trajectory_execution_timeout_margin_sec_ {1.50};
  double direct_goal_tolerance_rad_ {0.02};
  double cartesian_goal_tolerance_m_ {0.01};
  double ee_linear_speed_limit_ {0.08};
  double ee_linear_acc_limit_ {0.20};
  double joint_velocity_smoothing_ {0.35};
  double jacobian_damping_ {0.02};
  double cartesian_correction_gain_ {0.80};
  double cartesian_correction_max_step_m_ {0.015};
  int cartesian_correction_max_attempts_ {2};
  double cartesian_correction_interval_sec_ {0.15};
  double shoulder_kp_extension_gain_ {0.0};
  double shoulder_kd_extension_gain_ {0.0};
  double elbow_kp_extension_gain_ {0.0};
  double elbow_kd_extension_gain_ {0.0};
  double shoulder_gravity_comp_gain_ {0.0};
  double forearm_gravity_comp_gain_ {0.0};
  double elbow_gravity_comp_gain_ {0.0};
  double shoulder_coulomb_friction_ {0.0};
  double shoulder_viscous_friction_ {0.0};
  double shoulder_friction_velocity_scale_ {0.05};
  double elbow_coulomb_friction_ {0.0};
  double elbow_viscous_friction_ {0.0};
  double elbow_friction_velocity_scale_ {0.05};
  double mit_settle_timeout_sec_ {0.60};

  std::string move_service_name_ {"/r2/arm/move_to_xz"};
  std::string state_topic_ {"/r2/arm/state"};
  std::string feedback_status_topic_ {"/r2/arm/feedback_status"};
  std::string planning_group_ {"arm"};
  std::string base_frame_ {"base_link"};
  std::string end_effector_link_ {"tool_link"};
  std::string shoulder_joint_name_ {"shoulder_joint"};
  std::string elbow_joint_name_ {"elbow_joint"};
  std::string joint_states_topic_ {"/joint_states"};
  std::string executor_mode_ {"mit_native_5var"};
  std::string trajectory_action_name_ {"/arm_trajectory_controller/follow_joint_trajectory"};

  mutable std::mutex mutex_;
  bool feedback_ok_ {false};
  bool have_feedback_status_ {false};
  bool hard_timeout_active_ {false};
  bool startup_grace_active_ {false};
  uint8_t stale_motor_count_ {0};
  double last_target_x_ {0.0};
  double last_target_z_ {0.0};
  bool has_last_joint_goal_ {false};
  double last_goal_shoulder_ {0.0};
  double last_goal_forearm_ {0.0};
  MeasuredState measured_state_ {};
  r2_arm_control::ArmParams ik_params_ {};
  r2_arm_control::IkSolver solver_;
  std::array<double, 2> kp_cmd_ {40.0, 40.0};
  std::array<double, 2> kd_cmd_ {2.0, 2.0};
  std::array<double, 2> feedforward_cmd_ {0.0, 0.0};

  rclcpp::Publisher<r2_arm_control::msg::ArmMotionState>::SharedPtr state_pub_;
  rclcpp::Publisher<r2_arm_control::msg::MitJointCommand>::SharedPtr mit_cmd_pub_;
  rclcpp::Service<r2_arm_control::srv::MoveToXZ>::SharedPtr move_service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr feedback_status_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmCommandServerNode>();
  node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);

  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
