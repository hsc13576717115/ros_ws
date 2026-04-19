#include "r2_arm_control/angle_mapping.hpp"
#include "r2_arm_control/ik_solver.hpp"
#include "r2_arm_control/msg/mit_joint_command.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class IkCommanderNode : public rclcpp::Node
{
public:
  IkCommanderNode()
  : Node("ik_commander_node"),
    params_(),
    solver_(params_)
  {
    declare_parameter("d1", 0.30);
    declare_parameter("d2", 0.30);
    declare_parameter("forearm_mount_offset_rad", 0.0);

    declare_parameter("q1_min", -0.8);
    declare_parameter("q1_max", 1.4);
    declare_parameter("q2_min", -1.2);
    declare_parameter("q2_max", 1.8);

    declare_parameter("kp_joint0", 15.0);
    declare_parameter("kp_joint1", 15.0);
    declare_parameter("kd_joint0", 1.0);
    declare_parameter("kd_joint1", 1.0);

    declare_parameter("move_duration", 2.0);
    declare_parameter("elbow_up", true);

    declare_parameter("x_offset", 0.0);
    declare_parameter("z_offset", 0.0);

    params_.d1 = get_parameter("d1").as_double();
    params_.d2 = get_parameter("d2").as_double();
    params_.forearm_mount_offset_rad = get_parameter("forearm_mount_offset_rad").as_double();
    params_.q1_min = get_parameter("q1_min").as_double();
    params_.q1_max = get_parameter("q1_max").as_double();
    params_.q2_min = get_parameter("q2_min").as_double();
    params_.q2_max = get_parameter("q2_max").as_double();

    kp0_ = get_parameter("kp_joint0").as_double();
    kp1_ = get_parameter("kp_joint1").as_double();
    kd0_ = get_parameter("kd_joint0").as_double();
    kd1_ = get_parameter("kd_joint1").as_double();
    move_duration_ = get_parameter("move_duration").as_double();
    elbow_up_ = get_parameter("elbow_up").as_bool();

    x_offset_ = get_parameter("x_offset").as_double();
    z_offset_ = get_parameter("z_offset").as_double();

    solver_ = r2_arm_control::IkSolver(params_);

    mit_pub_ = create_publisher<r2_arm_control::msg::MitJointCommand>(
      "/arm_mit_controller/commands", 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      10,
      std::bind(&IkCommanderNode::jointStateCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(20ms, std::bind(&IkCommanderNode::timerCallback, this));

    current_q1_ = 0.0;
    current_q2_ = 0.0;
    hold_q1_ = 0.0;
    hold_q2_ = 0.0;

    input_thread_ = std::thread(&IkCommanderNode::inputLoop, this);

    RCLCPP_INFO(get_logger(), "IK commander started.");
    RCLCPP_INFO(get_logger(), "Input target x z in meters, e.g.: 0.00 0.00");
    RCLCPP_INFO(
      get_logger(),
      "x_offset = %.3f, z_offset = %.3f",
      x_offset_, z_offset_);
  }

  ~IkCommanderNode() override
  {
    running_.store(false);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    bool have_shoulder = false;
    double shoulder_abs = current_q1_;
    double elbow_rel = 0.0;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "shoulder_joint" && i < msg->position.size()) {
        shoulder_abs = r2_arm_control::urdfToPhysical(msg->position[i]);
        current_q1_ = shoulder_abs;
        have_shoulder = true;
        have_joint_state_ = true;
      }
      if (msg->name[i] == "elbow_joint" && i < msg->position.size()) {
        elbow_rel = r2_arm_control::urdfToPhysical(msg->position[i]);
        have_joint_state_ = true;
      }
    }

    if (have_shoulder) {
      current_q2_ = shoulder_abs + elbow_rel;
    }
  }

  void inputLoop()
  {
    while (rclcpp::ok() && running_.load()) {
      std::string line;
      std::cout << "\nEnter target x z (meters), or q to quit: ";
      if (!std::getline(std::cin, line)) {
        running_.store(false);
        break;
      }

      if (line == "q" || line == "quit") {
        rclcpp::shutdown();
        break;
      }

      std::stringstream ss(line);
      double x_input = 0.0;
      double z_input = 0.0;
      if (!(ss >> x_input >> z_input)) {
        std::cout << "Invalid input. Example: 0.00 0.00\n";
        continue;
      }

      const double x_real = x_input + x_offset_;
      const double z_real = z_input + z_offset_;

      const auto ik = solver_.solve(x_real, z_real, elbow_up_);
      if (!ik.success) {
        std::cout << "IK failed or target is outside safe workspace/limits.\n";
        std::cout << "Input target: x=" << x_input << ", z=" << z_input
                  << " | shifted target: x=" << x_real << ", z=" << z_real
                  << "\n";
        continue;
      }

      const auto fk = solver_.forward(ik.q1, ik.q2);

      {
        std::lock_guard<std::mutex> lock(mutex_);

        start_q1_ = have_joint_state_ ? current_q1_ : hold_q1_;
        start_q2_ = have_joint_state_ ? current_q2_ : hold_q2_;

        target_q1_ = ik.q1;
        target_q2_ = ik.q2;

        hold_q1_ = target_q1_;
        hold_q2_ = target_q2_;

        move_start_time_ = now();
        moving_ = true;
      }

      std::cout << "Target accepted: input x=" << x_input << ", z=" << z_input
                << " | shifted x=" << x_real << ", z=" << z_real
                << " -> shoulder_abs=" << ik.q1 << ", forearm_abs=" << ik.q2
                << " | FK check: x=" << fk.x << ", z=" << fk.z << "\n";
    }
  }

  void timerCallback()
  {
    double cmd_q1 = hold_q1_;
    double cmd_q2 = hold_q2_;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (moving_) {
        const double elapsed = (now() - move_start_time_).seconds();
        double s = elapsed / move_duration_;
        if (s >= 1.0) {
          s = 1.0;
          moving_ = false;
        }

        cmd_q1 = start_q1_ + s * (target_q1_ - start_q1_);
        cmd_q2 = start_q2_ + s * (target_q2_ - start_q2_);

        hold_q1_ = cmd_q1;
        hold_q2_ = cmd_q2;
      }
    }

    r2_arm_control::msg::MitJointCommand msg;
    msg.joint_names = {"shoulder_joint", "elbow_joint"};
    msg.position = {
      r2_arm_control::physicalToUrdf(cmd_q1),
      r2_arm_control::physicalToUrdf(r2_arm_control::normalizeAngle(cmd_q2 - cmd_q1))
    };
    msg.velocity = {0.0, 0.0};
    msg.kp = {kp0_, kp1_};
    msg.kd = {kd0_, kd1_};
    msg.effort = {0.0, 0.0};
    mit_pub_->publish(msg);
  }

  r2_arm_control::ArmParams params_;
  r2_arm_control::IkSolver solver_;

  double kp0_ {3.0};
  double kp1_ {3.0};
  double kd0_ {0.2};
  double kd1_ {0.2};
  double move_duration_ {2.0};
  bool elbow_up_ {true};

  double x_offset_ {0.30};
  double z_offset_ {0.30};

  rclcpp::Publisher<r2_arm_control::msg::MitJointCommand>::SharedPtr mit_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::thread input_thread_;
  std::atomic<bool> running_ {true};

  std::mutex mutex_;

  bool have_joint_state_ {false};
  bool moving_ {false};

  double current_q1_ {0.0};
  double current_q2_ {0.0};

  double start_q1_ {0.0};
  double start_q2_ {0.0};
  double target_q1_ {0.0};
  double target_q2_ {0.0};
  double hold_q1_ {0.0};
  double hold_q2_ {0.0};

  rclcpp::Time move_start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IkCommanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
