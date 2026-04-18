#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>

class JointStateTimestampBridgeNode : public rclcpp::Node
{
public:
  JointStateTimestampBridgeNode()
  : Node("joint_state_timestamp_bridge_node")
  {
    declare_parameter("input_topic", "/joint_states");
    declare_parameter("output_topic", "/joint_states_moveit");
    declare_parameter("always_restamp", true);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    always_restamp_ = get_parameter("always_restamp").as_bool();

    pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, 20);
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic_,
      20,
      std::bind(&JointStateTimestampBridgeNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Restamping joint states from %s to %s (always_restamp=%s)",
      input_topic_.c_str(),
      output_topic_.c_str(),
      always_restamp_ ? "true" : "false");
  }

private:
  void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    sensor_msgs::msg::JointState out = *msg;
    const bool has_zero_stamp =
      out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0;
    if (always_restamp_ || has_zero_stamp) {
      out.header.stamp = now();
    }
    pub_->publish(out);
  }

  std::string input_topic_ {"/joint_states"};
  std::string output_topic_ {"/joint_states_moveit"};
  bool always_restamp_ {true};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateTimestampBridgeNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
