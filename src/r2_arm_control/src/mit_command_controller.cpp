#include "r2_arm_control/mit_command_controller.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace r2_arm_control
{

controller_interface::CallbackReturn MitCommandController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MitCommandController::command_interface_configuration() const
{
  if (joint_names_.empty()) {
    return {
      controller_interface::interface_configuration_type::NONE,
      {}
    };
  }

  std::vector<std::string> interface_names;
  interface_names.reserve(joint_names_.size() * 5);
  for (const auto & joint_name : joint_names_) {
    interface_names.push_back(joint_name + "/position");
    interface_names.push_back(joint_name + "/velocity");
    interface_names.push_back(joint_name + "/effort");
    interface_names.push_back(joint_name + "/kp");
    interface_names.push_back(joint_name + "/kd");
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    interface_names
  };
}

controller_interface::InterfaceConfiguration
MitCommandController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE,
    {}
  };
}

controller_interface::CallbackReturn MitCommandController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_subscriber_ = get_node()->create_subscription<msg::MitJointCommand>(
    "~/commands",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MitCommandController::command_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitCommandController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const std::size_t expected_interfaces = joint_names_.size() * 5;
  if (command_interfaces_.size() != expected_interfaces) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu MIT command interfaces but received %zu.",
      expected_interfaces,
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  active_command_.position.resize(joint_names_.size(), 0.0);
  active_command_.velocity.resize(joint_names_.size(), 0.0);
  active_command_.effort.resize(joint_names_.size(), 0.0);
  active_command_.kp.resize(joint_names_.size(), 0.0);
  active_command_.kd.resize(joint_names_.size(), 0.0);

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const std::size_t offset = i * 5;
    active_command_.position[i] = command_interfaces_[offset + 0].get_value();
    active_command_.velocity[i] = command_interfaces_[offset + 1].get_value();
    active_command_.effort[i] = command_interfaces_[offset + 2].get_value();
    active_command_.kp[i] = command_interfaces_[offset + 3].get_value();
    active_command_.kd[i] = command_interfaces_[offset + 4].get_value();
  }

  rt_command_.initRT(active_command_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitCommandController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MitCommandController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  const MitCommandData & buffered_command = *rt_command_.readFromRT();
  if (buffered_command.position.size() != joint_names_.size()) {
    return controller_interface::return_type::OK;
  }

  active_command_ = buffered_command;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const std::size_t offset = i * 5;
    command_interfaces_[offset + 0].set_value(active_command_.position[i]);
    command_interfaces_[offset + 1].set_value(active_command_.velocity[i]);
    command_interfaces_[offset + 2].set_value(active_command_.effort[i]);
    command_interfaces_[offset + 3].set_value(active_command_.kp[i]);
    command_interfaces_[offset + 4].set_value(active_command_.kd[i]);
  }

  return controller_interface::return_type::OK;
}

void MitCommandController::command_callback(const msg::MitJointCommand::SharedPtr msg)
{
  MitCommandData normalized;
  if (!normalize_command_message(*msg, normalized)) {
    return;
  }
  rt_command_.writeFromNonRT(normalized);
}

bool MitCommandController::normalize_command_message(
  const msg::MitJointCommand & msg,
  MitCommandData & normalized) const
{
  const auto logger = get_node()->get_logger();
  const std::size_t joint_count = joint_names_.size();
  if (!vector_has_size(msg.position, joint_count, "position", logger) ||
    !vector_has_size(msg.velocity, joint_count, "velocity", logger) ||
    !vector_has_size(msg.effort, joint_count, "effort", logger) ||
    !vector_has_size(msg.kp, joint_count, "kp", logger) ||
    !vector_has_size(msg.kd, joint_count, "kd", logger))
  {
    return false;
  }

  normalized.position.resize(joint_count, 0.0);
  normalized.velocity.resize(joint_count, 0.0);
  normalized.effort.resize(joint_count, 0.0);
  normalized.kp.resize(joint_count, 0.0);
  normalized.kd.resize(joint_count, 0.0);

  if (msg.joint_names.empty()) {
    normalized.position = msg.position;
    normalized.velocity = msg.velocity;
    normalized.effort = msg.effort;
    normalized.kp = msg.kp;
    normalized.kd = msg.kd;
    return true;
  }

  if (msg.joint_names.size() != joint_count) {
    RCLCPP_WARN(
      logger,
      "Rejected MIT command because joint_names has size %zu but controller expects %zu joints.",
      msg.joint_names.size(),
      joint_count);
    return false;
  }

  for (std::size_t target_index = 0; target_index < joint_count; ++target_index) {
    const auto match_it = std::find(
      msg.joint_names.begin(), msg.joint_names.end(), joint_names_[target_index]);
    if (match_it == msg.joint_names.end()) {
      RCLCPP_WARN(
        logger,
        "Rejected MIT command because joint '%s' was not found in message joint_names.",
        joint_names_[target_index].c_str());
      return false;
    }

    const auto source_index = static_cast<std::size_t>(
      std::distance(msg.joint_names.begin(), match_it));
    normalized.position[target_index] = msg.position[source_index];
    normalized.velocity[target_index] = msg.velocity[source_index];
    normalized.effort[target_index] = msg.effort[source_index];
    normalized.kp[target_index] = msg.kp[source_index];
    normalized.kd[target_index] = msg.kd[source_index];
  }

  return true;
}

bool MitCommandController::vector_has_size(
  const std::vector<double> & values,
  std::size_t expected_size,
  const char * field_name,
  const rclcpp::Logger & logger)
{
  if (values.size() == expected_size) {
    return true;
  }

  RCLCPP_WARN(
    logger,
    "Rejected MIT command because field '%s' has size %zu but expected %zu.",
    field_name,
    values.size(),
    expected_size);
  return false;
}

}  // namespace r2_arm_control

PLUGINLIB_EXPORT_CLASS(
  r2_arm_control::MitCommandController, controller_interface::ControllerInterface)
