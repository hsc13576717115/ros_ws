#include "r2_arm_control/ik_solver.hpp"
#include "r2_arm_control/msg/arm_motion_state.hpp"
#include "r2_arm_control/srv/move_to_xz.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#ifdef R2_ARM_CONTROL_HAS_MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#endif

class ArmCommandServerNode : public rclcpp::Node
{
public:
  ArmCommandServerNode()
  : Node("arm_command_server_node"),
    solver_(ik_params_)
  {
    declare_parameter("x_offset", 0.0);
    declare_parameter("z_offset", 0.0);
    declare_parameter("d1", 0.30);
    declare_parameter("d2", 0.30);
    declare_parameter("q1_min", -1.80);
    declare_parameter("q1_max", 1.80);
    declare_parameter("q2_min", -1.80);
    declare_parameter("q2_max", 1.80);
    declare_parameter("elbow_up", true);
    declare_parameter("move_service_name", "/r2/arm/move_to_xz");
    declare_parameter("state_topic", "/r2/arm/state");
    declare_parameter("planning_group", "arm");
    declare_parameter("base_frame", "base_link");
    declare_parameter("end_effector_link", "tool_link");
    declare_parameter("shoulder_joint_name", "shoulder_joint");
    declare_parameter("elbow_joint_name", "elbow_joint");
    declare_parameter("planning_time", 2.0);
    declare_parameter("num_planning_attempts", 5);
    declare_parameter("goal_position_tolerance", 0.005);
    declare_parameter("goal_orientation_tolerance", 3.14159);
    declare_parameter("max_velocity_scaling_factor", 0.5);
    declare_parameter("max_acceleration_scaling_factor", 0.5);

    x_offset_ = get_parameter("x_offset").as_double();
    z_offset_ = get_parameter("z_offset").as_double();
    d1_ = get_parameter("d1").as_double();
    d2_ = get_parameter("d2").as_double();
    ik_params_.d1 = d1_;
    ik_params_.d2 = d2_;
    ik_params_.q1_min = get_parameter("q1_min").as_double();
    ik_params_.q1_max = get_parameter("q1_max").as_double();
    ik_params_.q2_min = get_parameter("q2_min").as_double();
    ik_params_.q2_max = get_parameter("q2_max").as_double();
    elbow_up_ = get_parameter("elbow_up").as_bool();
    solver_ = r2_arm_control::IkSolver(ik_params_);
    move_service_name_ = get_parameter("move_service_name").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    planning_group_ = get_parameter("planning_group").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    end_effector_link_ = get_parameter("end_effector_link").as_string();
    shoulder_joint_name_ = get_parameter("shoulder_joint_name").as_string();
    elbow_joint_name_ = get_parameter("elbow_joint_name").as_string();
    planning_time_ = get_parameter("planning_time").as_double();
    num_planning_attempts_ = static_cast<int>(get_parameter("num_planning_attempts").as_int());
    goal_position_tolerance_ = get_parameter("goal_position_tolerance").as_double();
    goal_orientation_tolerance_ = get_parameter("goal_orientation_tolerance").as_double();
    max_velocity_scaling_factor_ = get_parameter("max_velocity_scaling_factor").as_double();
    max_acceleration_scaling_factor_ = get_parameter("max_acceleration_scaling_factor").as_double();

    state_pub_ = create_publisher<r2_arm_control::msg::ArmMotionState>(state_topic_, 10);
    move_service_ = create_service<r2_arm_control::srv::MoveToXZ>(
      move_service_name_,
      std::bind(
        &ArmCommandServerNode::handleMoveRequest,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    publishState("IDLE", "MoveIt command server is waiting for move_group.", 0.0, 0.0, false);
  }

  void initialize(const rclcpp::Node::SharedPtr & self)
  {
#ifdef R2_ARM_CONTROL_HAS_MOVEIT
    std::lock_guard<std::mutex> lock(mutex_);

    try {
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        self, planning_group_);
      move_group_->setPoseReferenceFrame(base_frame_);
      move_group_->setEndEffectorLink(end_effector_link_);
      move_group_->setPlanningTime(planning_time_);
      move_group_->setNumPlanningAttempts(num_planning_attempts_);
      move_group_->setGoalPositionTolerance(goal_position_tolerance_);
      move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_);
      move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
      move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
      publishStateLocked(
        "READY", "MoveIt command server connected to move_group.", last_target_x_, last_target_z_, true);
      move_group_ready_ = true;
      RCLCPP_INFO(
        get_logger(),
        "MoveIt command server ready. group=%s ee_link=%s base_frame=%s",
        planning_group_.c_str(),
        end_effector_link_.c_str(),
        base_frame_.c_str());
    } catch (const std::exception & e) {
      move_group_ready_ = false;
      publishStateLocked(
        "ERROR", std::string("Failed to initialize MoveIt: ") + e.what(), 0.0, 0.0, false);
      RCLCPP_ERROR(get_logger(), "Failed to initialize MoveIt: %s", e.what());
    }
#else
    (void)self;
    publishState(
      "ERROR",
      "MoveIt support was not built. Install moveit_ros_planning_interface and rebuild.",
      0.0,
      0.0,
      false);
    RCLCPP_ERROR(
      get_logger(),
      "MoveIt support was not built. Install moveit_ros_planning_interface and rebuild.");
#endif
  }

private:
  struct CartesianPoint
  {
    double x {0.0};
    double z {0.0};
  };

  struct JointGoal
  {
    double shoulder {0.0};
    double elbow {0.0};
    bool used_elbow_up {true};
  };

  static double shortestAngularDistance(double from, double to)
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  static double jointGoalDistanceSquared(
    const JointGoal & candidate,
    double reference_shoulder,
    double reference_elbow)
  {
    const double shoulder_error =
      shortestAngularDistance(reference_shoulder, candidate.shoulder);
    const double elbow_error =
      shortestAngularDistance(reference_elbow, candidate.elbow);
    return shoulder_error * shoulder_error + elbow_error * elbow_error;
  }

  CartesianPoint currentCartesianLocked() const
  {
#ifdef R2_ARM_CONTROL_HAS_MOVEIT
    if (move_group_ && move_group_ready_) {
      const auto current_pose = move_group_->getCurrentPose(end_effector_link_);
      return {
        current_pose.pose.position.x - x_offset_,
        current_pose.pose.position.z - z_offset_,
      };
    }
#endif
    return {last_target_x_, last_target_z_};
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
          << " is outside the 2R workspace. Reach radius=" << radius
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
          jointGoalDistanceSquared(
          preferred_goal, last_goal_shoulder_, last_goal_elbow_);
        const double alternate_distance =
          jointGoalDistanceSquared(
          alternate_goal, last_goal_shoulder_, last_goal_elbow_);

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

  void publishState(
    const std::string & state,
    const std::string & message,
    double target_x,
    double target_z,
    bool feedback_ok)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishStateLocked(state, message, target_x, target_z, feedback_ok);
  }

  void publishStateLocked(
    const std::string & state,
    const std::string & message,
    double target_x,
    double target_z,
    bool feedback_ok)
  {
    const CartesianPoint current = currentCartesianLocked();
    last_target_x_ = target_x;
    last_target_z_ = target_z;

    r2_arm_control::msg::ArmMotionState msg;
    msg.state = state;
    msg.current_x = current.x;
    msg.current_z = current.z;
    msg.target_x = target_x;
    msg.target_z = target_z;
    msg.feedback_ok = feedback_ok;
    msg.stale_motor_count = feedback_ok ? 0 : 1;
    msg.message = message;
    state_pub_->publish(msg);
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
      response->message = validation_message;
      publishState("ERROR", response->message, requested_x, requested_z, false);
      return;
    }

    if (!solveJointGoal(requested_x, requested_z, joint_goal, validation_message)) {
      response->accepted = false;
      response->message = validation_message;
      publishState("ERROR", response->message, requested_x, requested_z, false);
      return;
    }

#ifdef R2_ARM_CONTROL_HAS_MOVEIT
    std::lock_guard<std::mutex> lock(mutex_);

    if (!move_group_ || !move_group_ready_) {
      response->accepted = false;
      response->message = "MoveIt command server is not ready.";
      publishStateLocked("ERROR", response->message, requested_x, requested_z, false);
      return;
    }

    publishStateLocked("PLANNING", "Planning MoveIt joint trajectory.", requested_x, requested_z, true);

    move_group_->stop();
    move_group_->clearPoseTargets();

    const bool accepted_target = move_group_->setJointValueTarget(
      std::map<std::string, double> {
        {shoulder_joint_name_, joint_goal.shoulder},
        {elbow_joint_name_, joint_goal.elbow},
      });
    if (!accepted_target) {
      response->accepted = false;
      response->message = "MoveIt rejected the requested joint target.";
      publishStateLocked("ERROR", response->message, requested_x, requested_z, false);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    if (!planned) {
      response->accepted = false;
      response->message = "MoveIt planning failed.";
      publishStateLocked("ERROR", response->message, requested_x, requested_z, false);
      return;
    }

    publishStateLocked("EXECUTING", "Executing MoveIt trajectory.", requested_x, requested_z, true);

    const bool executed = static_cast<bool>(move_group_->execute(plan));
    move_group_->stop();
    move_group_->clearPoseTargets();

    if (!executed) {
      response->accepted = false;
      response->message = "MoveIt execution failed.";
      publishStateLocked("ERROR", response->message, requested_x, requested_z, false);
      return;
    }

    response->accepted = true;
    std::ostringstream oss;
    oss << "MoveIt planning and execution succeeded. shoulder=" << joint_goal.shoulder
        << " rad, elbow=" << joint_goal.elbow << " rad, branch="
        << (joint_goal.used_elbow_up ? "elbow_up" : "elbow_down") << ".";
    response->message = oss.str();
    has_last_joint_goal_ = true;
    last_goal_shoulder_ = joint_goal.shoulder;
    last_goal_elbow_ = joint_goal.elbow;
    publishStateLocked("REACHED", response->message, requested_x, requested_z, true);
#else
    (void)request;
    response->accepted = false;
    response->message =
      "MoveIt support was not built. Install moveit_ros_planning_interface and rebuild.";
    publishState("ERROR", response->message, requested_x, requested_z, false);
#endif
  }

  double x_offset_ {0.0};
  double z_offset_ {0.0};
  double d1_ {0.30};
  double d2_ {0.30};
  bool elbow_up_ {true};
  double planning_time_ {2.0};
  double goal_position_tolerance_ {0.005};
  double goal_orientation_tolerance_ {3.14159};
  double max_velocity_scaling_factor_ {0.5};
  double max_acceleration_scaling_factor_ {0.5};
  int num_planning_attempts_ {5};

  std::string move_service_name_ {"/r2/arm/move_to_xz"};
  std::string state_topic_ {"/r2/arm/state"};
  std::string planning_group_ {"arm"};
  std::string base_frame_ {"base_link"};
  std::string end_effector_link_ {"tool_link"};
  std::string shoulder_joint_name_ {"shoulder_joint"};
  std::string elbow_joint_name_ {"elbow_joint"};

  mutable std::mutex mutex_;
  bool move_group_ready_ {false};
  double last_target_x_ {0.0};
  double last_target_z_ {0.0};
  bool has_last_joint_goal_ {false};
  double last_goal_shoulder_ {0.0};
  double last_goal_elbow_ {0.0};
  r2_arm_control::ArmParams ik_params_ {};
  r2_arm_control::IkSolver solver_;

  rclcpp::Publisher<r2_arm_control::msg::ArmMotionState>::SharedPtr state_pub_;
  rclcpp::Service<r2_arm_control::srv::MoveToXZ>::SharedPtr move_service_;

#ifdef R2_ARM_CONTROL_HAS_MOVEIT
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
#endif
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmCommandServerNode>();
  node->initialize(node);
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
