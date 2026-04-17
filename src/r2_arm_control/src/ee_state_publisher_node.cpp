#include "r2_arm_control/angle_mapping.hpp"
#include "r2_arm_control/ik_solver.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <string>

class EndEffectorStatePublisherNode : public rclcpp::Node
{
public:
  EndEffectorStatePublisherNode()
  : Node("ee_state_publisher_node"),
    solver_(params_)
  {
    declare_parameter("d1", 0.30);
    declare_parameter("d2", 0.30);
    declare_parameter("x_offset", 0.0);
    declare_parameter("z_offset", 0.0);
    declare_parameter("base_frame", "base_link");
    declare_parameter("joint_states_topic", "/joint_states");
    declare_parameter("end_effector_point_topic", "/r2/arm/end_effector_point");
    declare_parameter("log_end_effector_state", true);
    declare_parameter("end_effector_log_interval_ms", 500);

    params_.d1 = get_parameter("d1").as_double();
    params_.d2 = get_parameter("d2").as_double();
    x_offset_ = get_parameter("x_offset").as_double();
    z_offset_ = get_parameter("z_offset").as_double();
    base_frame_ = get_parameter("base_frame").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    point_topic_ = get_parameter("end_effector_point_topic").as_string();
    log_to_console_ = get_parameter("log_end_effector_state").as_bool();
    log_interval_ms_ = std::max<int64_t>(
      100,
      get_parameter("end_effector_log_interval_ms").as_int());

    solver_ = r2_arm_control::IkSolver(params_);

    point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(point_topic_, 10);
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      20,
      std::bind(&EndEffectorStatePublisherNode::jointStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Publishing end-effector point on %s from %s",
      point_topic_.c_str(),
      joint_states_topic_.c_str());
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double shoulder_urdf = 0.0;
    double elbow_urdf = 0.0;
    bool have_shoulder = false;
    bool have_elbow = false;

    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      if (msg->name[i] == "shoulder_joint") {
        shoulder_urdf = msg->position[i];
        have_shoulder = true;
      } else if (msg->name[i] == "elbow_joint") {
        elbow_urdf = msg->position[i];
        have_elbow = true;
      }
    }

    if (!have_shoulder || !have_elbow) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for shoulder_joint and elbow_joint in %s",
        joint_states_topic_.c_str());
      return;
    }

    const double shoulder_phys = r2_arm_control::urdfToPhysical(shoulder_urdf);
    const double elbow_phys = r2_arm_control::urdfToPhysical(elbow_urdf);
    const auto fk = solver_.forward(shoulder_phys, elbow_phys);

    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header = msg->header;
    if (point_msg.header.frame_id.empty()) {
      point_msg.header.frame_id = base_frame_;
    }
    point_msg.point.x = fk.x;
    point_msg.point.y = 0.0;
    point_msg.point.z = fk.z;
    point_pub_->publish(point_msg);

    if (log_to_console_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        log_interval_ms_,
        "EE base(%s): x=%.3f z=%.3f | command frame: x=%.3f z=%.3f",
        point_msg.header.frame_id.c_str(),
        fk.x,
        fk.z,
        fk.x - x_offset_,
        fk.z - z_offset_);
    }
  }

  r2_arm_control::ArmParams params_ {};
  r2_arm_control::IkSolver solver_;
  double x_offset_ {0.0};
  double z_offset_ {0.0};
  bool log_to_console_ {true};
  int64_t log_interval_ms_ {500};
  std::string base_frame_ {"base_link"};
  std::string joint_states_topic_ {"/joint_states"};
  std::string point_topic_ {"/r2/arm/end_effector_point"};

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EndEffectorStatePublisherNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
