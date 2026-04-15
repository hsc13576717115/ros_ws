#include "foot_controller.h"

#include "r2_arm_control/msg/arm_cartesian_target.hpp"
#include "r2_arm_control/msg/arm_motion_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "std_msgs/msg/bool.hpp"

class Foot_Controller : public rclcpp::Node{
    private:
        float step_length = 0.4;
        bool left = 0;
        bool right = 0;
    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
        rclcpp::Subscription<vmc_quadruped_controller::msg::MoveCmd>::SharedPtr move_cmd_subscription;
        rclcpp::Subscription<yesense_interface::msg::EulerOnly>::SharedPtr euler_subscription;
        rclcpp::Subscription<r2_arm_control::msg::ArmMotionState>::SharedPtr arm_state_subscription;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gpio_state_subscription;
        rclcpp::Publisher<r2_arm_control::msg::ArmCartesianTarget>::SharedPtr arm_target_publisher;
        rclcpp::TimerBase::SharedPtr arm_control_timer;
        rclcpp::TimerBase::SharedPtr status_line_timer;
        bool stand_up_flag = false;
        bool ctrl_by_joy = true;
        const float joy_deadzone = 0.10;
        const double joy_override_timeout_sec = 0.25;
        const double auto_cmd_timeout_sec = 0.50;
        const double arm_control_period_sec = 0.02;
        const double arm_stick_deadzone = 0.10;
        const double arm_target_vel_x = 0.25;
        const double arm_target_vel_z = 0.25;
        const double arm_target_default_x = 0.30;
        const double arm_target_default_z = 0.30;
        const double arm_target_min_x = -0.35;
        const double arm_target_max_x = 0.45;
        const double arm_target_min_z = 0.05;
        const double arm_target_max_z = 0.55;
        bool manual_pause_latched = false;
        bool prev_stand_btn = false;
        bool prev_sit_btn = false;
        std::chrono::steady_clock::time_point last_joy_motion_time;
        std::chrono::steady_clock::time_point last_auto_cmd_time;
        std::chrono::steady_clock::time_point last_arm_control_time;
        bool use_imu_pitch = false;
        double arm_stick_x = 0.0;
        double arm_stick_z = 0.0;
        double arm_target_x = 0.30;
        double arm_target_z = 0.30;
        double arm_current_x = 0.30;
        double arm_current_z = 0.30;
        double arm_report_target_x = 0.30;
        double arm_report_target_z = 0.30;
        bool arm_state_received = false;
        bool arm_target_initialized = false;
        bool gpio_state = false;
        bool gpio_state_received = false;
        float init_pos[4][2]
            ,leg_pos[4][2] // leg_pos {x,y}
            ,period = NORMAL_GAIT_PERIOD
            ,BODY_HEIGHT = NORMAL_GAIT_BODY_HEIGHT
            ,step_x
            ,step_y
            ,imu_pitch=0
            ,imu_yaw=0
            ,target_yaw=360;
        bool inited[4]={0,0,0,0}
            ,running = false
            ,runner_exists = false;
        Cycloid cycloid;
        VMC_Param params[4];
    public: Foot_Controller(): Node("foot_controller"){
        joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy",10,std::bind(&Foot_Controller::joy_callback,this,std::placeholders::_1));
        move_cmd_subscription = this->create_subscription<vmc_quadruped_controller::msg::MoveCmd>("move_cmd",10,std::bind(&Foot_Controller::move_cmd_callback,this,std::placeholders::_1));
        euler_subscription = this->create_subscription<yesense_interface::msg::EulerOnly>("euler_only",10,std::bind(&Foot_Controller::euler_callback,this,std::placeholders::_1));
        arm_state_subscription = this->create_subscription<r2_arm_control::msg::ArmMotionState>(
            "/r2/arm/state", 10, std::bind(&Foot_Controller::arm_state_callback, this, std::placeholders::_1));
        gpio_state_subscription = this->create_subscription<std_msgs::msg::Bool>(
            "/r2/manual/dpad_down_gpio36_state", 10, std::bind(&Foot_Controller::gpio_state_callback, this, std::placeholders::_1));
        arm_target_publisher = this->create_publisher<r2_arm_control::msg::ArmCartesianTarget>("/r2/arm/target_xz", 10);
        arm_control_timer = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(arm_control_period_sec)),
            std::bind(&Foot_Controller::arm_control_timer_callback, this));
        status_line_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Foot_Controller::status_line_timer_callback, this));
        cycloid.Length = 0.05;
        cycloid.Height = NORMAL_GAIT_HEIGHT;
        cycloid.FlightPercent = NORMAL_GAIT_FLIGHT_PERCENT;
        cycloid.BodyHeight = NORMAL_GAIT_BODY_HEIGHT;
        for(int i=0;i<4;i++){
            params[i].kp_x = INIT_KP_X;
            params[i].ki_x = INIT_KI_X;
            params[i].kd_x = INIT_KD_X;
            params[i].kp_y = INIT_KP_Y;
            params[i].ki_y = INIT_KI_Y;
            params[i].kd_y = INIT_KD_Y;
        }
        // params[3].kp_y = INIT_KP_Y_LEG3; // 3号腿输出力矩不足，软件解决
        // params[2].kp_y = INIT_KP_Y_LEG2; // 2号腿输出力矩不足，软件解决
        //     params[2].kd_y = INIT_KD_Y_LEG2; // 2号腿输出力矩不足，软件解决
        //     params[3].kd_y = INIT_KD_Y_LEG3; // 2号腿输出力矩不足，软件解决
            last_joy_motion_time = std::chrono::steady_clock::time_point::min();
            last_auto_cmd_time = std::chrono::steady_clock::time_point::min();
            last_arm_control_time = std::chrono::steady_clock::time_point::min();
            std::thread leg0(&Foot_Controller::control_leg,this,0);
            std::thread leg1(&Foot_Controller::control_leg,this,1);
            std::thread leg2(&Foot_Controller::control_leg,this,2);
            std::thread leg3(&Foot_Controller::control_leg,this,3);
            leg0.detach();
            leg1.detach();
            leg2.detach();
            leg3.detach();
    }
    private: bool is_joy_override_active() const{
        if (last_joy_motion_time == std::chrono::steady_clock::time_point::min()) {
            return false;
        }
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_joy_motion_time).count() < joy_override_timeout_sec;
    }
    private: bool has_recent_auto_cmd() const{
        if (last_auto_cmd_time == std::chrono::steady_clock::time_point::min()) {
            return false;
        }
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_auto_cmd_time).count() < auto_cmd_timeout_sec;
    }
    private: bool is_arm_stick_active() const{
        return std::abs(arm_stick_x) > arm_stick_deadzone || std::abs(arm_stick_z) > arm_stick_deadzone;
    }
    private: double clamp_arm_x(double x) const{
        return std::clamp(x, arm_target_min_x, arm_target_max_x);
    }
    private: double clamp_arm_z(double z) const{
        return std::clamp(z, arm_target_min_z, arm_target_max_z);
    }
    private: void publish_arm_target(){
        r2_arm_control::msg::ArmCartesianTarget arm_target_msg;
        arm_target_msg.stamp = this->now();
        arm_target_msg.x = arm_target_x;
        arm_target_msg.z = arm_target_z;
        arm_target_publisher->publish(arm_target_msg);
    }
    private: void arm_state_callback(const r2_arm_control::msg::ArmMotionState::SharedPtr msg){
        arm_current_x = msg->current_x;
        arm_current_z = msg->current_z;
        arm_report_target_x = msg->target_x;
        arm_report_target_z = msg->target_z;
        arm_state_received = true;
        if(!is_arm_stick_active()){
            arm_target_x = clamp_arm_x(arm_current_x);
            arm_target_z = clamp_arm_z(arm_current_z);
            arm_target_initialized = true;
        }
    }
    private: void gpio_state_callback(const std_msgs::msg::Bool::SharedPtr msg){
        gpio_state = msg->data;
        gpio_state_received = true;
    }
    private: void status_line_timer_callback(){
        const char * gpio_label = gpio_state_received ? (gpio_state ? "ON " : "OFF") : "---";
        if(arm_state_received){
            std::printf(
                "\rARM cur(%.3f, %.3f)  target(%.3f, %.3f)  GPIO36:%s",
                arm_current_x,
                arm_current_z,
                arm_report_target_x,
                arm_report_target_z,
                gpio_label);
        }else{
            std::printf("\rARM cur(--, --)  target(--, --)  GPIO36:%s", gpio_label);
        }
        std::fflush(stdout);
    }
    private: void arm_control_timer_callback(){
        auto now_time = std::chrono::steady_clock::now();
        double dt = arm_control_period_sec;
        if(last_arm_control_time != std::chrono::steady_clock::time_point::min()){
            dt = std::chrono::duration<double>(now_time - last_arm_control_time).count();
        }
        last_arm_control_time = now_time;
        dt = std::clamp(dt, 0.001, 0.1);

        if(!arm_target_initialized){
            if(arm_state_received){
                arm_target_x = clamp_arm_x(arm_current_x);
                arm_target_z = clamp_arm_z(arm_current_z);
            }
            arm_target_initialized = true;
        }

        if(!is_arm_stick_active()){
            if(arm_state_received){
                arm_target_x = clamp_arm_x(arm_current_x);
                arm_target_z = clamp_arm_z(arm_current_z);
            }
            return;
        }

        const double filtered_stick_x = std::abs(arm_stick_x) > arm_stick_deadzone ? arm_stick_x : 0.0;
        const double filtered_stick_z = std::abs(arm_stick_z) > arm_stick_deadzone ? arm_stick_z : 0.0;
        arm_target_x = clamp_arm_x(arm_target_x + filtered_stick_x * arm_target_vel_x * dt);
        arm_target_z = clamp_arm_z(arm_target_z + filtered_stick_z * arm_target_vel_z * dt);
        publish_arm_target();
    }
    private: void apply_motion_cmd(float cmd_step_x, float cmd_step_y){
        step_x = cmd_step_x;
        step_y = cmd_step_y;
        bool can_run = (abs(step_x) > 0 || abs(step_y) > 0);
        if(stand_up_flag && !running && can_run && !runner_exists){
            RCLCPP_DEBUG(this->get_logger(),"start run");
            std::thread runner(&Foot_Controller::run,this);
            runner.detach();
        }
        else if(running && !can_run){
            RCLCPP_DEBUG(this->get_logger(),"end run");
            running = false;
        }
    }
    private: void latch_manual_pause(){
        manual_pause_latched = true;
        last_joy_motion_time = std::chrono::steady_clock::now();
        apply_motion_cmd(0, 0);
    }
    private: void euler_callback(const yesense_interface::msg::EulerOnly::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(),"euler:%.3f %.3f %.3f",msg->euler.pitch,msg->euler.roll,msg->euler.yaw);
        float recvd_pitch = msg->euler.pitch/180.0*M_PI;
        imu_pitch = imu_pitch + (recvd_pitch - imu_pitch) * IMU_PITCH_KD;
        imu_yaw = msg->euler.yaw;
        // RCLCPP_INFO(this->get_logger(),"imu_pitch:%.3f",imu_pitch);
    }
    private: void move_cmd_callback(const vmc_quadruped_controller::msg::MoveCmd::SharedPtr msg){
        last_auto_cmd_time = std::chrono::steady_clock::now();
        if(ctrl_by_joy && (manual_pause_latched || is_joy_override_active())){
            return;
        }
        apply_motion_cmd(msg->step_x, msg->step_y);
    }

    private: void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        arm_stick_x = msg->axes[AXES_RX];
        arm_stick_z = msg->axes[AXES_RY];
        // // 打印轴和按钮状态
        // RCLCPP_INFO(this->get_logger(), "收到Joy消息:");
        
        // // 打印所有轴
        // RCLCPP_INFO(this->get_logger(), "轴:");
        // for (size_t i = 0; i < msg->axes.size(); ++i) {
        // RCLCPP_INFO(this->get_logger(), "  轴[%zu]: %.2f", i, msg->axes[i]);
        // }
        
        // // 打印所有按钮
        // RCLCPP_INFO(this->get_logger(), "按钮:");
        // for (size_t i = 0; i < msg->buttons.size(); ++i) {
        // RCLCPP_INFO(this->get_logger(), "  按钮[%zu]: %d", i, msg->buttons[i]);
        // }
        
        bool stand_btn = msg->buttons[STAND_UP_BTN];
        bool sit_btn = msg->buttons[SIT_DOWN_BTN];

        if(stand_btn && !prev_stand_btn){
            if(!stand_up_flag){
                if(inited[0]&&inited[1]&&inited[2]&&inited[3]){
                    stand_up();
                }
            }else{
                manual_pause_latched = false;
                RCLCPP_DEBUG(this->get_logger(),"manual pause released, auto navigation commands enabled");
            }
        }

        if(sit_btn && !prev_sit_btn){
            latch_manual_pause();
            if(stand_up_flag){
                sit_down();
            }
            RCLCPP_DEBUG(this->get_logger(),"manual emergency pause: stop and crouch");
        }

        prev_stand_btn = stand_btn;
        prev_sit_btn = sit_btn;
        if(msg->buttons[USE_IMU_PITCH_BTN]){
            use_imu_pitch = !use_imu_pitch;
            if(use_imu_pitch)
                RCLCPP_DEBUG(get_logger(),"enable imu pitch");
            else
                RCLCPP_DEBUG(get_logger(),"disable imu pitch");
        }
        if(msg->buttons[NORMAL_GAIT_BTN]){
            BODY_HEIGHT = NORMAL_GAIT_BODY_HEIGHT;
            cycloid.Height = NORMAL_GAIT_HEIGHT;
            cycloid.FlightPercent = NORMAL_GAIT_FLIGHT_PERCENT;
            cycloid.BodyHeight = NORMAL_GAIT_BODY_HEIGHT;
            period = NORMAL_GAIT_PERIOD;
            if(stand_up_flag)
                for(int i=0;i<4;i++){
                    leg_pos[i][0]=0;
                    leg_pos[i][1]=BODY_HEIGHT;
                }
            RCLCPP_DEBUG(this->get_logger(),"switch to normal gait");
        }
        if(msg->buttons[LOWER_GAIT_BTN]){
            BODY_HEIGHT = LOWER_GAIT_BODY_HEIGHT;
            cycloid.Height = LOWER_GAIT_HEIGHT;
            cycloid.FlightPercent = LOWER_GAIT_FLIGHT_PERCENT;
            cycloid.BodyHeight = LOWER_GAIT_BODY_HEIGHT;
            period = LOWER_GAIT_PERIOD;
            if(stand_up_flag)
                for(int i=0;i<4;i++){
                    leg_pos[i][0]=0;
                    leg_pos[i][1]=BODY_HEIGHT;
                }
            RCLCPP_DEBUG(this->get_logger(),"switch to lower gait");
        }
        if(msg->buttons[JUMP_BTN] && stand_up_flag && !running && !runner_exists){
            jump();
        }
        if(msg->buttons[JUMP_HIGH_BTN] && stand_up_flag && !running && !runner_exists){
            jump_high();
        }
        // if(msg->buttons[CTR_TYPE_BTN]){
        //     ctrl_by_joy = !ctrl_by_joy;
        //     if(ctrl_by_joy)
        //         RCLCPP_INFO(get_logger(),"switch control type: joy");
        //     else
        //         RCLCPP_INFO(get_logger(),"switch control type: auto");
        // }
        if(ctrl_by_joy){
            float joy_step_x = msg->axes[AXES_LX] / 2.0;
            float joy_step_y = -msg->axes[AXES_LY];
            bool joy_motion = (abs(joy_step_x) > joy_deadzone || abs(joy_step_y) > joy_deadzone);
            if (joy_motion) {
                manual_pause_latched = true;
                last_joy_motion_time = std::chrono::steady_clock::now();
                apply_motion_cmd(joy_step_x, joy_step_y);
            } else if(manual_pause_latched){
                apply_motion_cmd(0, 0);
            } else if(!has_recent_auto_cmd()){
                apply_motion_cmd(0, 0);
            }
        }
        // LT 是 axes[2]，按下时 < -0.5
        if (msg->axes[2] < -0.5) {
            period = FASTFAST_GAIT_PERIOD;           // 更快步频
            step_length = FAST_GAIT_STEP_LENGTH;      // 更小步长
            RCLCPP_DEBUG(this->get_logger(), "Switched to fast-short gait (triggered by LT)");
        }
        // RT 恢复步长和速度
        if (msg->axes[5] < -0.5) {
            period = NORMAL_GAIT_PERIOD;
            step_length = 0.4;
            RCLCPP_DEBUG(this->get_logger(), "Switched to fast-short gait (triggered by LT)");
        }
        // 十字左键修改左边高度降低
        if (msg->axes[6] > 0.5) {
            leg_offset_y[2] -= 0.01;
            leg_offset_y[3] -= 0.01;
            leg_pos[2][1] -= 0.01;
            leg_pos[3][1] -= 0.01;
            RCLCPP_DEBUG(this->get_logger(), "Leg 2 offset: %.3f, Leg 3 offset: %.3f", leg_offset_y[0], leg_offset_y[1]);
        }
        // 十字右键修改右边高度降低
        if (msg->axes[6] < -0.5) {
            leg_offset_y[0] -= 0.01;
            leg_offset_y[1] -= 0.01;
            leg_pos[0][1] -= 0.01;
            leg_pos[1][1] -= 0.01;
            RCLCPP_DEBUG(this->get_logger(), "Leg 0 offset: %.3f, Leg 1 offset: %.3f", leg_offset_y[0], leg_offset_y[1]);
        }
        // 十字上键恢复水平
        if (msg->axes[7] > 0.5) {
            for(int i=0;i<4;i++){
                leg_offset_y[i] = 0;
                // leg_pos[i][0] = 0;
                // leg_pos[i][1] = BODY_HEIGHT;
            }
        }
        if(msg->buttons[FAST_BIN]){
            period = FAST_GAIT_PERIOD;           // 更快步频
            step_length = FAST_GAIT_STEP_LENGTH;      // 更小步长
            RCLCPP_DEBUG(this->get_logger(), "Switched to fast-short gait (triggered by LT)");
        }
        if(msg->buttons[10]){
            period = BIG_FAST_GAIT_PERIOD;           // 更快步频
            step_length = BIG_GAIT_STEP_LENGTH;      // 更小步长
        }
    }

    private: void jump_high(){
        RCLCPP_DEBUG(this->get_logger(),"start jump");
        // ready for jump
        for(int i=0;i<4;i++){
            leg_pos[i][0] = -0.06;
            leg_pos[i][1] = 0.15;
        }
        float jump_x = -0.18;
        float jump_height = BODY_HEIGHT+0.10;
        // jump
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        params[0].kp_y = 6970*1.1;
        params[0].kd_y = JUMP_HIGH_KD_Y;
        params[0].kp_x = 5000;
        params[0].kd_x = 80;
        leg_pos[0][0] = jump_x;
        leg_pos[0][1] = jump_height + 0.01;

        params[1].kp_y = 5550;
        params[1].kd_y = JUMP_HIGH_KD_Y;
        params[1].kp_x = 5000;
        params[1].kd_x = 80;
        leg_pos[1][0] = jump_x;
        leg_pos[1][1] = jump_height + 0.03;

        params[2].kp_y = 7200;
        params[2].kd_y = JUMP_HIGH_KD_Y + 12;
        params[2].kp_x = 5000;
        params[2].kd_x = 80;
        leg_pos[2][0] = jump_x;
        leg_pos[2][1] = jump_height + 0.03;

        params[3].kp_y = 8300*1.3;
        params[3].kd_y = JUMP_HIGH_KD_Y + 15;
        params[3].kp_x = 5000;
        params[3].kd_x = 80;
        leg_pos[3][0] = jump_x;
        leg_pos[3][1] = jump_height + 0.01;
        // fall down
        std::this_thread::sleep_for(std::chrono::milliseconds(170));
        for(int i=0;i<4;i++){
            
            params[i].kp_y = 500;
            params[i].kd_y = 40;
            params[i].kp_x = 500;
            params[i].kd_x = 40;
            params[i].kp_x = INIT_KP_X;
            leg_pos[i][0] = -0.06;
            leg_pos[i][1] = BODY_HEIGHT - 0.03;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        for(int i=0;i<4;i++){
            
            params[i].kp_y = 500;
            params[i].kd_y = 40;
            params[i].kp_x = 500;
            params[i].kd_x = 40;
            params[i].kp_x = INIT_KP_X;
            leg_pos[i][0] = 0.10;
            leg_pos[i][1] = BODY_HEIGHT;
        }
        // release
        std::this_thread::sleep_for(std::chrono::milliseconds(1300));
        for(int i=0;i<4;i++){
            params[i].kp_y = INIT_KP_Y;
            params[i].kd_y = INIT_KD_Y;
            params[i].kp_x = INIT_KP_X;
            params[i].kd_x = INIT_KD_X;
            leg_pos[i][0] = 0;
            leg_pos[i][1] = BODY_HEIGHT;
        }
        RCLCPP_DEBUG(this->get_logger(),"end jump");
    }

    private: void jump(){
        RCLCPP_DEBUG(this->get_logger(),"start jump");
        // ready for jump
        for(int i=0;i<4;i++){
            leg_pos[i][0] = -0.03;
            leg_pos[i][1] = 0.15;
        }
        float jump_x = -0.08;
        // jump
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        params[0].kp_y = 5000;
        params[0].kd_y = 40;
        leg_pos[0][0] = jump_x;
        leg_pos[0][1] = BODY_HEIGHT;

        params[1].kp_y = 4500;
        params[1].kd_y = 40;
        leg_pos[1][0] = jump_x;
        leg_pos[1][1] = BODY_HEIGHT;

        params[2].kp_y = 4500;
        params[2].kd_y = 40;
        leg_pos[2][0] = jump_x;
        leg_pos[2][1] = BODY_HEIGHT;

        params[3].kp_y = 6000;
        params[3].kd_y = 40;
        leg_pos[3][0] = jump_x;
        leg_pos[3][1] = BODY_HEIGHT;
        // fall down
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        for(int i=0;i<4;i++){
            params[i].kp_y = 800;
            params[i].kd_y = 40;
            params[i].kp_x = 800;
            params[i].kd_x = 40;
            params[i].kp_x = INIT_KP_X;
            leg_pos[i][0] = 0;
        }
        // release
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        for(int i=0;i<4;i++){
            params[i].kp_y = INIT_KP_Y;
            params[i].kd_y = INIT_KD_Y;
            params[i].kp_x = INIT_KP_X;
            params[i].kd_x = INIT_KD_X;
        }
        RCLCPP_DEBUG(this->get_logger(),"end jump");
    }

    private: void stand_up(){
        stand_up_flag = true;
        // // print leg init pos
        // for(int i=0;i<4;i++){
        //     RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",i,leg_pos[i][0],leg_pos[i][1]);
        // }
        uint microstep = 200;
        for(uint i=0;i<microstep && rclcpp::ok();i++){
            for(int j=0;j<4;j++){
                leg_pos[j][0] = init_pos[j][0] + (0.0 - init_pos[j][0])*(i/(float)microstep);
                leg_pos[j][1] = init_pos[j][1] + (BODY_HEIGHT - init_pos[j][1])*(i/(float)microstep);
                // RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",j,leg_pos[j][0],leg_pos[j][1]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        RCLCPP_DEBUG(this->get_logger(),"stand_up");
    }

    private: void sit_down(){
        stand_up_flag = false;
        // // print leg init pos
        // for(int i=0;i<4;i++){
        //     RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",i,leg_pos[i][0],leg_pos[i][1]);
        // }
        uint microstep = 200;
        for(uint i=0;i<microstep && rclcpp::ok();i++){
            for(int j=0;j<4;j++){
                leg_pos[j][0] = init_pos[j][0] + (0.0 - init_pos[j][0])*(1 - i/(float)microstep);
                leg_pos[j][1] = init_pos[j][1] + (BODY_HEIGHT - init_pos[j][1])*(1 - i/(float)microstep);
                // RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",j,leg_pos[j][0],leg_pos[j][1]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_DEBUG(this->get_logger(),"sit_down");
    }

    private: void run(){
        running = true;
        runner_exists = true;
        // step_length = 0.40;
        double intpart;
        float t;
        // bool isFlightPercent[4]={0,0,0,0};
        // float flight = 2000
        //     ,noflight = 2000
        //     ,flight_kd = 80
        //     ,noflight_kd = 80;
        CycloidResult res;
        float local_step_x = 0
            ,local_step_y = 0;
        while(running && rclcpp::ok()){
            t = modf(rclcpp::Clock().now().seconds()/period,&intpart);
            local_step_x = local_step_x + CTRL_X_KD * (step_x - local_step_x);
            local_step_y = local_step_y + CTRL_Y_KD * (step_y - local_step_y);
            cycloid.Length = local_step_y * step_length + local_step_x * step_length;
            res = cycloid.generate(0.25+t);
            // if(res.isFlightPercent != isFlightPercent[2]){
            //     isFlightPercent[2] = res.isFlightPercent;
            //     if(isFlightPercent[2]){
            //         params[2].kp_y=flight;
            //         params[2].kd_y=flight_kd;
            //     }
            //     else{
            //         params[2].kp_y=noflight;
            //         params[2].kd_y=noflight_kd;
            //     }
            // }
            leg_pos[2][0] = res.x;
            leg_pos[2][1] = res.y + leg_offset_y[2];
            // leg_pos[2][1] = res.y;
            res = cycloid.generate(0.75+t);
            // if(res.isFlightPercent != isFlightPercent[3]){
            //     isFlightPercent[3] = res.isFlightPercent;
            //     if(isFlightPercent[3]){
            //         params[3].kp_y=flight;
            //         params[3].kd_y=flight_kd;
            //     }
            //     else{
            //         params[3].kp_y=noflight;
            //         params[3].kd_y=noflight_kd;
            //     }
            // }
            leg_pos[3][0] = res.x;
            leg_pos[3][1] = res.y + leg_offset_y[3];
            // leg_pos[3][1] = res.y;

            cycloid.Length = local_step_y * step_length - local_step_x * step_length;
            res = cycloid.generate(0.25+t);
            // if(res.isFlightPercent != isFlightPercent[0]){
            //     isFlightPercent[0] = res.isFlightPercent;
            //     if(isFlightPercent[0]){
            //         params[0].kp_y=flight;
            //         params[0].kd_y=flight_kd;
            //     }
            //     else{
            //         params[0].kp_y=noflight;
            //         params[0].kd_y=noflight_kd;
            //     }
            // }
            leg_pos[0][0] = res.x;
            leg_pos[0][1] = res.y + leg_offset_y[0];
            // leg_pos[0][1] = res.y;
            res = cycloid.generate(0.75+t);
            // if(res.isFlightPercent != isFlightPercent[1]){
            //     isFlightPercent[1] = res.isFlightPercent;
            //     if(isFlightPercent[1]){
            //         params[1].kp_y=flight;
            //         params[1].kd_y=flight_kd;
            //     }
            //     else{
            //         params[1].kp_y=noflight;
            //         params[1].kd_y=noflight_kd;
            //     }
            // }
            leg_pos[1][0] = res.x;
            leg_pos[1][1] = res.y + leg_offset_y[1];
            // leg_pos[1][1] = res.y;
            
            // for(int i=0;i<4;i++){
            //     RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",i,leg_pos[i][0],leg_pos[i][1]);
            // }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        for(int i=0;i<4;i++){
            leg_pos[i][0]=0;
            leg_pos[i][1]=BODY_HEIGHT + leg_offset_y[i];
            // leg_pos[i][1]=BODY_HEIGHT;
        }
        runner_exists = false;
    }
    private: void control_leg(uint id){
        // // start count
        // auto start = std::chrono::system_clock::now();
        // int count = 0;
        // VMC_Param param;
        // param.kp_x = 4000;
        // param.kd_x = 50;
        // param.kp_y = 1000;
        // param.kd_y = 50;
        // init motor data
        SerialPort* serial;
        switch(id){
            case 0:
                serial = new SerialPort("/dev/ttyS3");
                break;
            case 1:
                serial = new SerialPort("/dev/ttyS7");
                break;
            case 2:
                serial = new SerialPort("/dev/ttyS6");
                break;
            case 3:
                serial = new SerialPort("/dev/ttyS1");
                break;
        }
        MotorData data_outer,data_inner;
        data_outer.motorType = MotorType::GO_M8010_6;
        data_inner.motorType = MotorType::GO_M8010_6;
        // init motor cmd
        MotorCmd cmd_outer,cmd_inner;
        cmd_outer.motorType = MotorType::GO_M8010_6;
        cmd_inner.motorType = MotorType::GO_M8010_6;
        cmd_outer.id = motor_id_for_legs[id][0];
        cmd_outer.mode = 1;
        cmd_outer.tau = 0;
        cmd_outer.kp = 0;
        cmd_outer.kd = 0;
        cmd_outer.q = 0;
        cmd_outer.dq = 0;
        cmd_inner.id = motor_id_for_legs[id][1];
        cmd_inner.mode = 1;
        cmd_inner.tau = 0;
        cmd_inner.kp = 0;
        cmd_inner.kd = 0;
        cmd_inner.q = 0;
        cmd_inner.dq = 0;
        // get gear_ratio
        float gear_ratio = queryGearRatio(MotorType::GO_M8010_6);
        // init values
        int dir_outer = motor_dir[motor_id_for_legs[id][0]]
            ,dir_inner = motor_dir[motor_id_for_legs[id][1]];
        float target_pos_x
            ,target_pos_y
            ,angle_alpha
            ,angle_beta
            ,vec_alpha
            ,vec_beta
            ,outer_tau
            ,inner_tau;
        KinematicResult kineRes;
        VMC_Result vmcRes;
        JacobiResult jocRes;
        // set init pos
        kineRes = Kinematic_Solution(OUTER_MOTOR_OFFEST,INNER_MOTOR_OFFEST,0,0);
        leg_pos[id][0] = kineRes.pos_x;
        leg_pos[id][1] = kineRes.pos_z;
        init_pos[id][0] = kineRes.pos_x;
        init_pos[id][1] = kineRes.pos_z;
        target_pos_x = kineRes.pos_x;
        target_pos_y = kineRes.pos_z;
        // target_pos_x = 0;
        // target_pos_y = 0.223;
        // // test target pos out
        // std::cout << target_pos_x << "  " << target_pos_y << std::endl;
        // return;

        // auto stretch legs
        // stretch
        cmd_outer.tau = 0.1 * dir_outer;
        cmd_inner.tau = -0.1 * dir_inner;
        SendMsg(&data_outer,&cmd_outer,serial);
        SendMsg(&data_inner,&cmd_inner,serial);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // release
        cmd_outer.tau = 0;
        cmd_inner.tau = 0;
        SendMsg(&data_outer,&cmd_outer,serial);
        SendMsg(&data_inner,&cmd_inner,serial);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // save motor offests
        float outer_motor_offest = data_outer.q
            ,inner_motor_offest = data_inner.q;
        inited[id] = true;
        RCLCPP_DEBUG(this->get_logger(), "inited leg:%d",id);
        // init ki
        params[id].start = rclcpp::Clock().now().seconds();
        while(rclcpp::ok()){
            target_pos_x = leg_pos[id][0];
            target_pos_y = leg_pos[id][1];
            angle_alpha = ((data_outer.q - outer_motor_offest) / gear_ratio)*dir_outer + OUTER_MOTOR_OFFEST;
            angle_beta = ((data_inner.q - inner_motor_offest) / gear_ratio)*dir_inner + INNER_MOTOR_OFFEST;
            if(use_imu_pitch){
                angle_alpha -= imu_pitch;
                angle_beta -= imu_pitch;
            }
            vec_alpha = data_outer.dq / gear_ratio * dir_outer;
            vec_beta = data_inner.dq / gear_ratio * dir_inner;
            kineRes = Kinematic_Solution(angle_alpha,angle_beta,vec_alpha,vec_beta);
            params[id].period = rclcpp::Clock().now().seconds() - params[id].start;
            vmcRes = VMC_Calculate(&params[id],target_pos_x,target_pos_y,kineRes.pos_x,kineRes.pos_z,kineRes.vec_x,kineRes.vec_z);
            jocRes = VMC_Jacobi_Matrix(angle_alpha,angle_beta,vmcRes.force_x,vmcRes.force_z);
            outer_tau = jocRes.tau_alpha / gear_ratio * dir_outer;
            inner_tau = jocRes.tau_beta / gear_ratio * dir_inner;
            // RCLCPP_INFO(this->get_logger(),"force_x:%.3f force_y:%.3f",vmcRes.force_x,vmcRes.force_z);
            // outer_tau = clip(outer_tau,0.1);
            // inner_tau = clip(inner_tau,0.1);
            // printf("force_x:%.3f force_y:%.3f outer_tau:%.3f inner_tau:%.3f\r\n",vmcRes.force_x,vmcRes.force_z,outer_tau,inner_tau);
            cmd_outer.tau = outer_tau;
            cmd_inner.tau = inner_tau;
            SendMsg(&data_outer,&cmd_outer,serial);
            SendMsg(&data_inner,&cmd_inner,serial);
            // wait for other thread to send messages
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
            // // count freq
            // count ++ ;
            // auto time_now = std::chrono::system_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - start);
            // if(duration.count()>1000){
            //     std::cout << "id:" << id << " count:" << count << std::endl;
            //     start = time_now;
            //     count = 0;
            // }
        }
        cmd_outer.mode = 0;
        cmd_inner.mode = 0;
        SendMsg(&data_outer,&cmd_outer,serial);
        SendMsg(&data_inner,&cmd_inner,serial);
    }

};

void SendMsg(MotorData* data,MotorCmd* cmd,SerialPort* serial){
    serial->sendRecv(cmd,data);
}

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Foot_Controller>());
	rclcpp::shutdown();
    return 0;
}
