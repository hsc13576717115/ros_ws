#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <array>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "r2_arm_control/angle_mapping.hpp"
#include "r2_arm_control/damiao.hpp"

namespace r2_arm_control
{

inline constexpr const char * kR2PrimaryCommandInterface = "position";
inline constexpr const char * kR2SecondaryCommandInterface = "velocity";
inline constexpr const char * kR2EffortCommandInterface = "effort";
inline constexpr const char * kR2KpCommandInterface = "kp";
inline constexpr const char * kR2KdCommandInterface = "kd";

class DmHW : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void publish_feedback_status(std::size_t stale_motor_count);

  std::vector<DmActData> hw_actuator_data_;
  std::unordered_map<std::string, std::unique_ptr<Motor_Control>> motor_controls_;
  std::unordered_map<std::string, std::unordered_map<int, DmActData>> port_to_motors_config_;
  std::vector<JointAngleMapping> joint_mappings_;
  std::unordered_map<std::string, size_t> joint_index_by_name_;
  rclcpp::Node::SharedPtr feedback_node_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr feedback_status_pub_;
  std::string feedback_status_topic_ {"/r2/arm/feedback_status"};
  double feedback_timeout_sec_ {0.05};
  double feedback_hard_timeout_sec_ {0.30};
  double feedback_startup_grace_sec_ {0.0};
  double read_error_log_interval_sec_ {0.50};
  double mit_position_command_alpha_ {1.0};
  double mit_velocity_command_alpha_ {1.0};
  double mit_effort_command_alpha_ {1.0};
  double mit_gain_command_alpha_ {1.0};
  std::vector<double> filtered_physical_cmd_pos_;
  std::vector<double> filtered_physical_cmd_vel_;
  std::vector<double> filtered_physical_cmd_effort_;
  std::vector<double> filtered_cmd_kp_;
  std::vector<double> filtered_cmd_kd_;
  std::chrono::steady_clock::time_point activated_at_ {};
  bool hard_timeout_active_ {false};
  bool startup_grace_active_ {false};
  bool startup_feedback_ready_ {false};
  bool command_filter_initialized_ {false};
};

RCLCPP_SHARED_PTR_DEFINITIONS(DmHW)

}  // namespace r2_arm_control
