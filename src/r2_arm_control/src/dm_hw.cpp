#include "r2_arm_control/dm_hw.hpp"

#include <algorithm>
#include <exception>
#include <sstream>
#include <string>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace r2_arm_control
{

namespace
{

DM_Motor_Type stringToMotorType(const std::string & type_str)
{
  if (type_str == "DM4310") {
    return DM4310;
  }
  if (type_str == "DM4340") {
    return DM4340;
  }
  if (type_str == "DM8006") {
    return DM8006;
  }

  RCLCPP_WARN(
    rclcpp::get_logger("DmHW"),
    "Unknown motor type '%s' specified. Defaulting to DM4310.",
    type_str.c_str());
  return DM4310;
}

template<typename T>
T getHardwareParamOr(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const T & fallback);

template<>
double getHardwareParamOr<double>(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const double & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }

  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      rclcpp::get_logger("DmHW"),
      "Invalid hardware parameter '%s'='%s'. Falling back to %.3f.",
      key.c_str(), it->second.c_str(), fallback);
    return fallback;
  }
}

template<>
std::string getHardwareParamOr<std::string>(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end() || it->second.empty()) {
    return fallback;
  }
  return it->second;
}

template<typename T>
T getJointParamOr(
  const hardware_interface::ComponentInfo & joint,
  const std::string & key,
  const T & fallback);

template<>
double getJointParamOr<double>(
  const hardware_interface::ComponentInfo & joint,
  const std::string & key,
  const double & fallback)
{
  const auto it = joint.parameters.find(key);
  if (it == joint.parameters.end()) {
    return fallback;
  }

  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      rclcpp::get_logger("DmHW"),
      "Invalid joint parameter '%s'='%s' on '%s'. Falling back to %.3f.",
      key.c_str(), it->second.c_str(), joint.name.c_str(), fallback);
    return fallback;
  }
}

}  // namespace

hardware_interface::CallbackReturn DmHW::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Initializing R2 arm hardware interface");
  hw_actuator_data_.resize(info_.joints.size());
  joint_mappings_.resize(info_.joints.size());

  feedback_timeout_sec_ = std::max(
    0.01,
    getHardwareParamOr<double>(info_, "feedback_timeout_sec", feedback_timeout_sec_));
  feedback_hard_timeout_sec_ = std::max(
    feedback_timeout_sec_,
    getHardwareParamOr<double>(
      info_, "feedback_hard_timeout_sec", feedback_hard_timeout_sec_));
  read_error_log_interval_sec_ = std::max(
    0.05,
    getHardwareParamOr<double>(
      info_, "read_error_log_interval_sec", read_error_log_interval_sec_));
  feedback_status_topic_ = getHardwareParamOr<std::string>(
    info_, "feedback_status_topic", feedback_status_topic_);

  feedback_node_ = rclcpp::Node::make_shared("r2_arm_feedback_status_node");
  feedback_status_pub_ = feedback_node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
    feedback_status_topic_, 10);

  std::unordered_map<std::string, int> port_to_baud_rate;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];

    const std::string serial_port = joint.parameters.at("serial_port");
    const int baud_rate = std::stoi(joint.parameters.at("baud_rate"));
    const int can_id = std::stoi(joint.parameters.at("can_id"));
    const int mst_id = std::stoi(joint.parameters.at("mst_id"));
    const std::string motor_type_str = joint.parameters.at("motor_type");
    const double motor_sign = sanitizeMotorSign(
      getJointParamOr<double>(joint, "motor_sign", 1.0));
    const double zero_offset_rad = getJointParamOr<double>(joint, "zero_offset_rad", 0.0);

    DmActData data;
    data.name = joint.name;
    data.can_id = can_id;
    data.mst_id = mst_id;
    data.motorType = stringToMotorType(motor_type_str);
    data.motor_sign = motor_sign;
    data.zero_offset_rad = zero_offset_rad;

    port_to_motors_config_[serial_port][can_id] = data;
    if (port_to_baud_rate.find(serial_port) == port_to_baud_rate.end()) {
      port_to_baud_rate[serial_port] = baud_rate;
    }

    hw_actuator_data_[i] = data;
    joint_mappings_[i] = JointAngleMapping {joint.name, motor_sign, zero_offset_rad};

    RCLCPP_INFO(
      rclcpp::get_logger("DmHW"),
      "Joint '%s': motor_sign=%.3f zero_offset_rad=%.6f",
      joint.name.c_str(), motor_sign, zero_offset_rad);
  }

  for (auto & pair : port_to_motors_config_) {
    const std::string & port_name = pair.first;
    auto * motor_config_map_ptr = &pair.second;
    const int baud_rate_for_port = port_to_baud_rate.at(port_name);

    try {
      motor_controls_[port_name] = std::make_unique<Motor_Control>(
        port_name,
        baud_rate_for_port,
        motor_config_map_ptr,
        read_error_log_interval_sec_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DmHW"),
        "Failed to create Motor_Control for port '%s': %s",
        port_name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DmHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hw_actuator_data_[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_actuator_data_[i].pos);
    state_interfaces.emplace_back(
      hw_actuator_data_[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_actuator_data_[i].vel);
    state_interfaces.emplace_back(
      hw_actuator_data_[i].name,
      hardware_interface::HW_IF_EFFORT,
      &hw_actuator_data_[i].effort);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DmHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hw_actuator_data_[i].name, "position_des", &hw_actuator_data_[i].cmd_pos);
    command_interfaces.emplace_back(
      hw_actuator_data_[i].name, "velocity_des", &hw_actuator_data_[i].cmd_vel);
    command_interfaces.emplace_back(
      hw_actuator_data_[i].name, "kp", &hw_actuator_data_[i].kp);
    command_interfaces.emplace_back(
      hw_actuator_data_[i].name, "kd", &hw_actuator_data_[i].kd);
    command_interfaces.emplace_back(
      hw_actuator_data_[i].name, "feedforward", &hw_actuator_data_[i].cmd_effort);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DmHW::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Activating hardware");

  for (const auto & [port_name, driver] : motor_controls_) {
    RCLCPP_INFO(
      rclcpp::get_logger("DmHW"),
      "Enabling motors on port: %s",
      port_name.c_str());
    driver->enable();
    usleep(1000000);

    bool switched_ok = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (driver->switch_mode_all(MIT_MODE)) {
        switched_ok = true;
        break;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("DmHW"),
        "Switch to MIT_MODE failed on port %s, attempt %d/3",
        port_name.c_str(),
        attempt + 1);
      usleep(300000);
    }

    if (!switched_ok) {
      RCLCPP_ERROR(
        rclcpp::get_logger("DmHW"),
        "Failed to switch all motors on port %s to MIT_MODE after retries. "
        "Keeping ros2_control alive; arm state may stay stale until communication recovers.",
        port_name.c_str());
    }

    try {
      driver->start_receive_thread();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("DmHW"),
        "Failed to start receive thread on port %s: %s",
        port_name.c_str(), e.what());
    } catch (...) {
      RCLCPP_ERROR(
        rclcpp::get_logger("DmHW"),
        "Failed to start receive thread on port %s: unknown exception",
        port_name.c_str());
    }
    usleep(300000);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmHW::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Deactivating hardware");
  for (const auto & [port_name, driver] : motor_controls_) {
    (void)port_name;
    driver->stop_receive_thread();
    driver->disable();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DmHW::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  std::size_t stale_motor_count = 0;

  for (const auto & [port_name, driver] : motor_controls_) {
    (void)port_name;
    driver->read();
  }

  const auto steady_now = std::chrono::steady_clock::now();

  for (size_t i = 0; i < hw_actuator_data_.size(); ++i) {
    const auto & joint_info = info_.joints[i];
    const std::string & port = joint_info.parameters.at("serial_port");
    const int can_id = std::stoi(joint_info.parameters.at("can_id"));

    const DmActData & updated_data = port_to_motors_config_.at(port).at(can_id);
    const auto & mapping = joint_mappings_[i];
    hw_actuator_data_[i].pos = physicalToUrdf(motorToPhysical(updated_data.pos, mapping));
    hw_actuator_data_[i].vel = motorToPhysicalScalar(updated_data.vel, mapping);
    hw_actuator_data_[i].effort = motorToPhysicalScalar(updated_data.effort, mapping);

    bool motor_is_stale = !updated_data.has_valid_feedback;
    if (updated_data.has_valid_feedback) {
      const double feedback_age_sec =
        std::chrono::duration<double>(steady_now - updated_data.last_valid_feedback_time).count();
      motor_is_stale = feedback_age_sec > feedback_timeout_sec_;
    }
    if (motor_is_stale) {
      ++stale_motor_count;
    }
  }

  publish_feedback_status(stale_motor_count);

  std::ostringstream feedback_stream;
  for (size_t i = 0; i < hw_actuator_data_.size(); ++i) {
    if (i > 0) {
      feedback_stream << " | ";
    }
    feedback_stream
      << hw_actuator_data_[i].name
      << " pos=" << hw_actuator_data_[i].pos
      << " vel=" << hw_actuator_data_[i].vel
      << " effort=" << hw_actuator_data_[i].effort;
  }

  RCLCPP_DEBUG_THROTTLE(
    rclcpp::get_logger("DmHW"),
    steady_clock,
    1000,
    "%s",
    feedback_stream.str().c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DmHW::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < hw_actuator_data_.size(); ++i) {
    const auto & joint_info = info_.joints[i];
    const std::string & port = joint_info.parameters.at("serial_port");
    const int can_id = std::stoi(joint_info.parameters.at("can_id"));

    auto & mapped_data = port_to_motors_config_.at(port).at(can_id);
    const auto & mapping = joint_mappings_[i];
    mapped_data.cmd_pos = physicalToMotor(urdfToPhysical(hw_actuator_data_[i].cmd_pos), mapping);
    mapped_data.cmd_vel = physicalToMotorScalar(hw_actuator_data_[i].cmd_vel, mapping);
    mapped_data.kp = hw_actuator_data_[i].kp;
    mapped_data.kd = hw_actuator_data_[i].kd;
    mapped_data.cmd_effort = physicalToMotorScalar(hw_actuator_data_[i].cmd_effort, mapping);
  }

  for (const auto & [port_name, driver] : motor_controls_) {
    (void)port_name;
    driver->write();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn DmHW::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmHW::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Cleaning up hardware");
  feedback_status_pub_.reset();
  feedback_node_.reset();
  motor_controls_.clear();
  port_to_motors_config_.clear();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmHW::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_FATAL(rclcpp::get_logger("DmHW"), "Hardware has encountered an error");
  on_deactivate(rclcpp_lifecycle::State());
  return hardware_interface::CallbackReturn::SUCCESS;
}

void DmHW::publish_feedback_status(std::size_t stale_motor_count)
{
  if (!feedback_status_pub_) {
    return;
  }

  std_msgs::msg::UInt8MultiArray msg;
  msg.data = {
    static_cast<std::uint8_t>(stale_motor_count == 0 ? 1 : 0),
    static_cast<std::uint8_t>(std::min<std::size_t>(stale_motor_count, 255)),
  };
  feedback_status_pub_->publish(msg);
}

}  // namespace r2_arm_control

PLUGINLIB_EXPORT_CLASS(r2_arm_control::DmHW, hardware_interface::SystemInterface)
