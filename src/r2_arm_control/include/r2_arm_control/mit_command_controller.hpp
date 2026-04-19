#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "r2_arm_control/msg/mit_joint_command.hpp"
#include "rclcpp/subscription.hpp"

namespace r2_arm_control
{

class MitCommandController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct MitCommandData
  {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
    std::vector<double> kp;
    std::vector<double> kd;
  };

  void command_callback(const msg::MitJointCommand::SharedPtr msg);
  bool normalize_command_message(const msg::MitJointCommand & msg, MitCommandData & normalized) const;
  static bool vector_has_size(
    const std::vector<double> & values, std::size_t expected_size, const char * field_name,
    const rclcpp::Logger & logger);

  std::vector<std::string> joint_names_;
  realtime_tools::RealtimeBuffer<MitCommandData> rt_command_;
  MitCommandData active_command_;
  rclcpp::Subscription<msg::MitJointCommand>::SharedPtr command_subscriber_;
};

}  // namespace r2_arm_control
