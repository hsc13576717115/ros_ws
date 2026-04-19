#include "r2_arm_control/dm_hw.hpp"

#include <algorithm>
#include <cmath>
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

double blendCommandValue(double previous, double target, double alpha)
{
  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  if (!std::isfinite(previous)) {
    return target;
  }
  if (!std::isfinite(target)) {
    return previous;
  }
  return previous + clamped_alpha * (target - previous);
}

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

template<>
std::string getJointParamOr<std::string>(
  const hardware_interface::ComponentInfo & joint,
  const std::string & key,
  const std::string & fallback)
{
  const auto it = joint.parameters.find(key);
  if (it == joint.parameters.end() || it->second.empty()) {
    return fallback;
  }
  return it->second;
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
  feedback_startup_grace_sec_ = std::max(
    0.0,
    getHardwareParamOr<double>(
      info_, "feedback_startup_grace_sec", feedback_startup_grace_sec_));
  read_error_log_interval_sec_ = std::max(
    0.05,
    getHardwareParamOr<double>(
      info_, "read_error_log_interval_sec", read_error_log_interval_sec_));
  feedback_status_topic_ = getHardwareParamOr<std::string>(
    info_, "feedback_status_topic", feedback_status_topic_);
  mit_position_command_alpha_ = std::clamp(
    getHardwareParamOr<double>(info_, "mit_position_command_alpha", mit_position_command_alpha_),
    0.0, 1.0);
  mit_velocity_command_alpha_ = std::clamp(
    getHardwareParamOr<double>(info_, "mit_velocity_command_alpha", mit_velocity_command_alpha_),
    0.0, 1.0);
  mit_effort_command_alpha_ = std::clamp(
    getHardwareParamOr<double>(info_, "mit_effort_command_alpha", mit_effort_command_alpha_),
    0.0, 1.0);
  mit_gain_command_alpha_ = std::clamp(
    getHardwareParamOr<double>(info_, "mit_gain_command_alpha", mit_gain_command_alpha_),
    0.0, 1.0);

  feedback_node_ = rclcpp::Node::make_shared("r2_arm_feedback_status_node");
  feedback_status_pub_ = feedback_node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
    feedback_status_topic_, 10);

  filtered_physical_cmd_pos_.assign(info_.joints.size(), 0.0);
  filtered_physical_cmd_vel_.assign(info_.joints.size(), 0.0);
  filtered_physical_cmd_effort_.assign(info_.joints.size(), 0.0);
  filtered_cmd_kp_.assign(info_.joints.size(), 0.0);
  filtered_cmd_kd_.assign(info_.joints.size(), 0.0);

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
    const double transmission_ratio = getJointParamOr<double>(joint, "transmission_ratio", 1.0);
    const std::string absolute_reference_joint = getJointParamOr<std::string>(
      joint, "absolute_reference_joint", "");
    const double mit_kp = getJointParamOr<double>(joint, "mit_kp", 0.0);
    const double mit_kd = getJointParamOr<double>(joint, "mit_kd", 0.0);
    const double mit_feedforward = getJointParamOr<double>(joint, "mit_feedforward", 0.0);

    DmActData data;
    data.name = joint.name;
    data.can_id = can_id;
    data.mst_id = mst_id;
    data.motorType = stringToMotorType(motor_type_str);
    data.motor_sign = motor_sign;
    data.zero_offset_rad = zero_offset_rad;
    data.kp = mit_kp;
    data.kd = mit_kd;
    data.cmd_effort = mit_feedforward;

    port_to_motors_config_[serial_port][can_id] = data;
    if (port_to_baud_rate.find(serial_port) == port_to_baud_rate.end()) {
      port_to_baud_rate[serial_port] = baud_rate;
    }

    hw_actuator_data_[i] = data;
    joint_mappings_[i] =
      JointAngleMapping {joint.name, motor_sign, zero_offset_rad, transmission_ratio,
        absolute_reference_joint};
    joint_index_by_name_[joint.name] = i;

    RCLCPP_INFO(
      rclcpp::get_logger("DmHW"),
      "Joint '%s': motor_sign=%.3f zero_offset_rad=%.6f transmission_ratio=%.6f absolute_reference_joint='%s' mit_kp=%.3f mit_kd=%.3f mit_ff=%.3f",
      joint.name.c_str(), motor_sign, zero_offset_rad, transmission_ratio,
      absolute_reference_joint.c_str(), mit_kp, mit_kd, mit_feedforward);
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
      hw_actuator_data_[i].name, kR2PrimaryCommandInterface, &hw_actuator_data_[i].cmd_pos);
    command_interfaces.emplace_back(
      hw_actuator_data_[i].name, kR2SecondaryCommandInterface, &hw_actuator_data_[i].cmd_vel);
    command_interfaces.emplace_back(
      hw_actuator_data_[i].name, kR2EffortCommandInterface, &hw_actuator_data_[i].cmd_effort);
    command_interfaces.emplace_back(
      hw_actuator_data_[i].name, kR2KpCommandInterface, &hw_actuator_data_[i].kp);
    command_interfaces.emplace_back(
      hw_actuator_data_[i].name, kR2KdCommandInterface, &hw_actuator_data_[i].kd);
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

  activated_at_ = std::chrono::steady_clock::now();
  hard_timeout_active_ = false;
  startup_grace_active_ = feedback_startup_grace_sec_ > 0.0;
  read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.0));
  for (auto & actuator : hw_actuator_data_) {
    actuator.cmd_pos = actuator.pos;
    actuator.cmd_vel = 0.0;
  }
  for (size_t i = 0; i < hw_actuator_data_.size(); ++i) {
    filtered_physical_cmd_pos_[i] = urdfToPhysical(hw_actuator_data_[i].pos);
    filtered_physical_cmd_vel_[i] = 0.0;
    filtered_physical_cmd_effort_[i] = hw_actuator_data_[i].cmd_effort;
    filtered_cmd_kp_[i] = hw_actuator_data_[i].kp;
    filtered_cmd_kd_[i] = hw_actuator_data_[i].kd;
  }
  command_filter_initialized_ = true;

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
  std::size_t hard_timeout_motor_count = 0;
  std::vector<double> raw_physical_pos(hw_actuator_data_.size(), 0.0);
  std::vector<double> raw_physical_vel(hw_actuator_data_.size(), 0.0);
  std::vector<double> raw_physical_effort(hw_actuator_data_.size(), 0.0);

  for (const auto & [port_name, driver] : motor_controls_) {
    (void)port_name;
    driver->read();
  }

  const auto steady_now = std::chrono::steady_clock::now();
  startup_grace_active_ =
    feedback_startup_grace_sec_ > 0.0 &&
    std::chrono::duration<double>(steady_now - activated_at_).count() < feedback_startup_grace_sec_;

  for (size_t i = 0; i < hw_actuator_data_.size(); ++i) {
    const auto & joint_info = info_.joints[i];
    const std::string & port = joint_info.parameters.at("serial_port");
    const int can_id = std::stoi(joint_info.parameters.at("can_id"));

    const DmActData & updated_data = port_to_motors_config_.at(port).at(can_id);
    const auto & mapping = joint_mappings_[i];
    raw_physical_pos[i] = motorToPhysical(updated_data.pos, mapping);
    raw_physical_vel[i] = motorToPhysicalScalar(updated_data.vel, mapping);
    raw_physical_effort[i] = motorToPhysicalScalar(updated_data.effort, mapping);

    bool motor_is_stale = !updated_data.has_valid_feedback;
    bool motor_hard_timed_out = !updated_data.has_valid_feedback;
    if (updated_data.has_valid_feedback) {
      const double feedback_age_sec =
        std::chrono::duration<double>(steady_now - updated_data.last_valid_feedback_time).count();
      motor_is_stale = feedback_age_sec > feedback_timeout_sec_;
      motor_hard_timed_out = feedback_age_sec > feedback_hard_timeout_sec_;
    }
    if (motor_is_stale) {
      ++stale_motor_count;
    }
    if (!startup_grace_active_ && motor_hard_timed_out) {
      ++hard_timeout_motor_count;
    }
  }

  hard_timeout_active_ = hard_timeout_motor_count > 0;
  if (hard_timeout_active_) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("DmHW"),
      steady_clock,
      1000,
      "Hard feedback timeout active. Freezing commands at the latest measured pose.");
  }

  for (size_t i = 0; i < hw_actuator_data_.size(); ++i) {
    const auto & mapping = joint_mappings_[i];
    double physical_pos = raw_physical_pos[i];
    double physical_vel = raw_physical_vel[i];

    if (!mapping.absolute_reference_joint_name.empty()) {
      const auto ref_it = joint_index_by_name_.find(mapping.absolute_reference_joint_name);
      if (ref_it != joint_index_by_name_.end()) {
        physical_pos -= raw_physical_pos[ref_it->second];
        physical_vel -= raw_physical_vel[ref_it->second];
      }
    }

    hw_actuator_data_[i].pos = physicalToUrdf(physical_pos);
    hw_actuator_data_[i].vel = physical_vel;
    hw_actuator_data_[i].effort = raw_physical_effort[i];
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
  std::vector<double> desired_physical_pos(hw_actuator_data_.size(), 0.0);
  std::vector<double> desired_physical_vel(hw_actuator_data_.size(), 0.0);
  std::vector<double> desired_physical_effort(hw_actuator_data_.size(), 0.0);
  std::vector<double> desired_kp(hw_actuator_data_.size(), 0.0);
  std::vector<double> desired_kd(hw_actuator_data_.size(), 0.0);

  for (size_t i = 0; i < hw_actuator_data_.size(); ++i) {
    if (hard_timeout_active_) {
      hw_actuator_data_[i].cmd_pos = hw_actuator_data_[i].pos;
      hw_actuator_data_[i].cmd_vel = 0.0;
      desired_physical_pos[i] = urdfToPhysical(hw_actuator_data_[i].pos);
      desired_physical_vel[i] = 0.0;
      desired_physical_effort[i] = 0.0;
      desired_kp[i] = hw_actuator_data_[i].kp;
      desired_kd[i] = hw_actuator_data_[i].kd;
    } else {
      desired_physical_pos[i] = urdfToPhysical(hw_actuator_data_[i].cmd_pos);
      desired_physical_vel[i] = hw_actuator_data_[i].cmd_vel;
      desired_physical_effort[i] = hw_actuator_data_[i].cmd_effort;
      desired_kp[i] = hw_actuator_data_[i].kp;
      desired_kd[i] = hw_actuator_data_[i].kd;
    }

    if (!command_filter_initialized_) {
      filtered_physical_cmd_pos_[i] = desired_physical_pos[i];
      filtered_physical_cmd_vel_[i] = desired_physical_vel[i];
      filtered_physical_cmd_effort_[i] = desired_physical_effort[i];
      filtered_cmd_kp_[i] = desired_kp[i];
      filtered_cmd_kd_[i] = desired_kd[i];
      continue;
    }

    if (hard_timeout_active_) {
      filtered_physical_cmd_pos_[i] = desired_physical_pos[i];
      filtered_physical_cmd_vel_[i] = desired_physical_vel[i];
      filtered_physical_cmd_effort_[i] = 0.0;
      filtered_cmd_kp_[i] = desired_kp[i];
      filtered_cmd_kd_[i] = desired_kd[i];
    } else {
      filtered_physical_cmd_pos_[i] = blendCommandValue(
        filtered_physical_cmd_pos_[i], desired_physical_pos[i], mit_position_command_alpha_);
      filtered_physical_cmd_vel_[i] = blendCommandValue(
        filtered_physical_cmd_vel_[i], desired_physical_vel[i], mit_velocity_command_alpha_);
      filtered_physical_cmd_effort_[i] = blendCommandValue(
        filtered_physical_cmd_effort_[i], desired_physical_effort[i], mit_effort_command_alpha_);
      filtered_cmd_kp_[i] = blendCommandValue(
        filtered_cmd_kp_[i], desired_kp[i], mit_gain_command_alpha_);
      filtered_cmd_kd_[i] = blendCommandValue(
        filtered_cmd_kd_[i], desired_kd[i], mit_gain_command_alpha_);
    }
  }
  command_filter_initialized_ = true;

  for (size_t i = 0; i < hw_actuator_data_.size(); ++i) {
    const auto & joint_info = info_.joints[i];
    const std::string & port = joint_info.parameters.at("serial_port");
    const int can_id = std::stoi(joint_info.parameters.at("can_id"));

    auto & mapped_data = port_to_motors_config_.at(port).at(can_id);
    const auto & mapping = joint_mappings_[i];
    double physical_cmd_pos = filtered_physical_cmd_pos_[i];
    double physical_cmd_vel = filtered_physical_cmd_vel_[i];
    double physical_cmd_effort = filtered_physical_cmd_effort_[i];

    if (!mapping.absolute_reference_joint_name.empty()) {
      const auto ref_it = joint_index_by_name_.find(mapping.absolute_reference_joint_name);
      if (ref_it != joint_index_by_name_.end()) {
        physical_cmd_pos += filtered_physical_cmd_pos_[ref_it->second];
        physical_cmd_vel += filtered_physical_cmd_vel_[ref_it->second];
      }
    }

    mapped_data.cmd_pos = physicalToMotor(physical_cmd_pos, mapping);
    mapped_data.cmd_vel = physicalToMotorScalar(physical_cmd_vel, mapping);
    mapped_data.kp = filtered_cmd_kp_[i];
    mapped_data.kd = filtered_cmd_kd_[i];
    mapped_data.cmd_effort = hard_timeout_active_
      ? 0.0
      : physicalToMotorScalar(physical_cmd_effort, mapping);
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
    static_cast<std::uint8_t>((stale_motor_count == 0 && !hard_timeout_active_) ? 1 : 0),
    static_cast<std::uint8_t>(std::min<std::size_t>(stale_motor_count, 255)),
    static_cast<std::uint8_t>(hard_timeout_active_ ? 1 : 0),
    static_cast<std::uint8_t>(startup_grace_active_ ? 1 : 0),
  };
  feedback_status_pub_->publish(msg);
}

}  // namespace r2_arm_control

PLUGINLIB_EXPORT_CLASS(r2_arm_control::DmHW, hardware_interface::SystemInterface)
