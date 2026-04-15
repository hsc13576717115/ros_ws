#include "r2_arm_control/angle_mapping.hpp"
#include "r2_arm_control/ik_solver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

}  // namespace

class ArmPoseTestNode : public rclcpp::Node
{
public:
  ArmPoseTestNode()
  : Node("arm_pose_test_node")
  {
    declare_parameter("d1", 0.30);
    declare_parameter("d2", 0.30);
    declare_parameter("q1_min", -1.80);
    declare_parameter("q1_max", 1.80);
    declare_parameter("q2_min", -1.80);
    declare_parameter("q2_max", 1.80);
    declare_parameter("q1_phys_deg", 0.0);
    declare_parameter("q2_phys_deg", 0.0);
    declare_parameter("use_target_ik", false);
    declare_parameter("target_x", 0.30);
    declare_parameter("target_z", 0.30);
    declare_parameter("x_offset", 0.0);
    declare_parameter("z_offset", 0.0);
    declare_parameter("elbow_up", true);
    declare_parameter("publish_rate_hz", 10.0);

    params_.d1 = get_parameter("d1").as_double();
    params_.d2 = get_parameter("d2").as_double();
    params_.q1_min = get_parameter("q1_min").as_double();
    params_.q1_max = get_parameter("q1_max").as_double();
    params_.q2_min = get_parameter("q2_min").as_double();
    params_.q2_max = get_parameter("q2_max").as_double();

    const double q1_phys_deg = get_parameter("q1_phys_deg").as_double();
    const double q2_phys_deg = get_parameter("q2_phys_deg").as_double();
    const bool use_target_ik = get_parameter("use_target_ik").as_bool();
    const double target_x = get_parameter("target_x").as_double();
    const double target_z = get_parameter("target_z").as_double();
    const double x_offset = get_parameter("x_offset").as_double();
    const double z_offset = get_parameter("z_offset").as_double();
    const bool elbow_up = get_parameter("elbow_up").as_bool();
    const double publish_rate_hz = std::max(1.0, get_parameter("publish_rate_hz").as_double());

    solver_ = r2_arm_control::IkSolver(params_);

    if (use_target_ik) {
      const auto ik = solver_.solve(target_x + x_offset, target_z + z_offset, elbow_up);
      if (!ik.success) {
        throw std::runtime_error("arm_pose_test_node failed to solve the requested target.");
      }
      q1_urdf_ = r2_arm_control::physicalToUrdf(ik.q1);
      q2_urdf_ = r2_arm_control::physicalToUrdf(ik.q2);
      const auto fk = solver_.forward(ik.q1, ik.q2);
      RCLCPP_INFO(
        get_logger(),
        "Publishing IK pose for target x=%.3f z=%.3f -> q1_phys=%.2f deg q2_phys=%.2f deg | FK x=%.3f z=%.3f",
        target_x,
        target_z,
        ik.q1 / kDegToRad,
        ik.q2 / kDegToRad,
        fk.x - x_offset,
        fk.z - z_offset);
    } else {
      const double q1_phys = q1_phys_deg * kDegToRad;
      const double q2_phys = q2_phys_deg * kDegToRad;
      q1_urdf_ = r2_arm_control::physicalToUrdf(q1_phys);
      q2_urdf_ = r2_arm_control::physicalToUrdf(q2_phys);
      const auto fk = solver_.forward(q1_phys, q2_phys);
      RCLCPP_INFO(
        get_logger(),
        "Publishing fixed pose q1_phys=%.2f deg q2_phys=%.2f deg -> FK x=%.3f z=%.3f",
        q1_phys_deg,
        q2_phys_deg,
        fk.x,
        fk.z);
    }

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz)),
      std::bind(&ArmPoseTestNode::publishJointState, this));
  }

private:
  void publishJointState()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {"shoulder_joint", "elbow_joint"};
    msg.position = {q1_urdf_, q2_urdf_};
    msg.velocity = {0.0, 0.0};
    msg.effort = {0.0, 0.0};
    joint_state_pub_->publish(msg);
  }

  r2_arm_control::ArmParams params_ {};
  r2_arm_control::IkSolver solver_ {params_};
  double q1_urdf_ {0.0};
  double q2_urdf_ {0.0};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmPoseTestNode>();
  rclcpp::spin(node);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
