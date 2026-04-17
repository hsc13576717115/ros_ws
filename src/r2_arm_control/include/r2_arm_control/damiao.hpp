#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <libserial/SerialPort.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

namespace r2_arm_control
{

constexpr uint32_t POS_MODE = 0x100;
constexpr uint32_t SPEED_MODE = 0x200;
constexpr uint8_t kMaxRetries = 20;
constexpr unsigned int kRetryIntervalUs = 50000;

using MotorId = uint32_t;

enum DM_Motor_Type
{
  DM4310,
  DM4310_48V,
  DM4340,
  DM4340_48V,
  DM6006,
  DM8006,
  DM8009,
  DM10010L,
  DM10010,
  DMH3510,
  DMH6215,
  DMG6220,
  Num_Of_Motor
};

enum Control_Mode
{
  MIT_MODE = 1,
  POS_VEL_MODE = 2,
  VEL_MODE = 3,
  POS_FORCE_MODE = 4,
};

enum DM_REG
{
  CTRL_MODE = 10,
};

#pragma pack(push, 1)

struct CAN_Receive_Frame
{
  uint8_t FrameHeader;
  uint8_t CMD;
  uint8_t canDataLen : 6;
  uint8_t canIde : 1;
  uint8_t canRtr : 1;
  uint32_t canId;
  uint8_t canData[8];
  uint8_t frameEnd;
};

struct can_send_frame
{
  uint8_t FrameHeader[2] = {0x55, 0xAA};
  uint8_t FrameLen = 0x1e;
  uint8_t CMD = 0x03;
  uint32_t sendTimes = 1;
  uint32_t timeInterval = 10;
  uint8_t IDType = 0;
  uint32_t canId = 0x01;
  uint8_t frameType = 0;
  uint8_t len = 0x08;
  uint8_t idAcc = 0;
  uint8_t dataAcc = 0;
  uint8_t data[8] = {0};
  uint8_t crc = 0;

  void modify(const MotorId id, const uint8_t * send_data)
  {
    canId = id;
    std::copy(send_data, send_data + 8, data);
  }
};

#pragma pack(pop)

struct Limit_param
{
  float Q_MAX;
  float DQ_MAX;
  float TAU_MAX;
};

extern Limit_param limit_param[Num_Of_Motor];

struct MitCommand
{
  float kp {0.0f};
  float kd {0.0f};
  float q {0.0f};
  float dq {0.0f};
  float tau {0.0f};
};

MitCommand sanitizeMitCommandInputs(
  const Limit_param & limits,
  float kp,
  float kd,
  float q,
  float dq,
  float tau);

struct DmActData
{
  std::string name;
  DM_Motor_Type motorType;
  int can_id;
  int mst_id;
  double pos {0.0};
  double vel {0.0};
  double effort {0.0};
  double cmd_pos {0.0};
  double cmd_vel {0.0};
  double cmd_effort {0.0};
  double kp {0.0};
  double kd {0.0};
  double motor_sign {1.0};
  double zero_offset_rad {0.0};
  bool has_valid_feedback {false};
  std::size_t consecutive_invalid_frames {0};
  std::chrono::steady_clock::time_point last_valid_feedback_time {};
  std::chrono::steady_clock::time_point last_invalid_log_time {};
};

class Motor
{
public:
  Motor(DM_Motor_Type motor_type, MotorId slave_id, MotorId master_id);

  void receive_data(float q, float dq, float tau);
  DM_Motor_Type GetMotorType() const {return motor_type_;}
  MotorId GetMasterId() const {return master_id_;}
  MotorId GetSlaveId() const {return slave_id_;}
  float Get_Position() const {return state_q_;}
  float Get_Velocity() const {return state_dq_;}
  float Get_tau() const {return state_tau_;}
  Limit_param get_limit_param() const {return limit_param_;}

  void set_param(int key, float value);
  void set_param(int key, uint32_t value);
  float get_param_as_float(int key) const;
  uint32_t get_param_as_uint32(int key) const;
  bool is_have_param(int key) const;

private:
  union ValueUnion
  {
    float floatValue;
    uint32_t uint32Value;
  };

  struct ValueType
  {
    ValueUnion value;
    bool isFloat;
  };

  MotorId master_id_;
  MotorId slave_id_;
  float state_q_ {0.0f};
  float state_dq_ {0.0f};
  float state_tau_ {0.0f};
  Limit_param limit_param_ {};
  DM_Motor_Type motor_type_;
  std::unordered_map<uint32_t, ValueType> param_map_;
};

class Motor_Control
{
public:
  Motor_Control(
    std::string serial_port,
    int baud_rate,
    std::unordered_map<int, DmActData> * data_ptr,
    double read_error_log_interval_sec = 0.5);

  ~Motor_Control();

  void get_motor_data_thread();
  void start_receive_thread();
  void stop_receive_thread();
  void enable();
  void disable();
  void set_zero_position();
  bool switch_mode_all(Control_Mode mode);
  void write();
  void read();
  void refresh_motor_status(const Motor & motor);
  void control_mit(Motor & motor, float kp, float kd, float q, float dq, float tau);
  void control_pos_vel(Motor & motor, float pos, float vel);
  void control_vel(Motor & motor, float vel);
  void receive_param();
  void addMotor(Motor * motor);
  float read_motor_param(Motor & motor, uint8_t rid);
  bool switchControlMode(Motor & motor, Control_Mode mode);
  bool change_motor_param(Motor & motor, uint8_t rid, float data);
  void save_motor_param(Motor & motor);

  static void changeMotorLimit(Motor & motor, float p_max, float q_max, float t_max);

private:
  struct LatestFeedback
  {
    float q {0.0f};
    float dq {0.0f};
    float tau {0.0f};
    std::chrono::steady_clock::time_point received_at {};
    bool valid {false};
  };

  void WriteData(const can_send_frame & frame);
  bool ReadData(CAN_Receive_Frame & frame, size_t timeout_ms = 100);
  void control_cmd(MotorId id, uint8_t cmd);
  void write_motor_param(Motor & motor, uint8_t rid, const uint8_t data[4]);
  MotorId resolve_feedback_motor_key(const CAN_Receive_Frame & frame) const;
  bool feedback_frame_matches_motor(const CAN_Receive_Frame & frame, const Motor & motor) const;
  bool decode_feedback_frame(
    const CAN_Receive_Frame & frame,
    const Motor & motor,
    float & q,
    float & dq,
    float & tau) const;

  static bool is_in_ranges(int number)
  {
    return (7 <= number && number <= 10) || (13 <= number && number <= 16) ||
      (35 <= number && number <= 36);
  }

  static uint32_t float_to_uint32(float value)
  {
    return static_cast<uint32_t>(value);
  }

  static float uint8_to_float(const uint8_t data[4])
  {
    uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
      (static_cast<uint32_t>(data[2]) << 16) |
      (static_cast<uint32_t>(data[1]) << 8) |
      static_cast<uint32_t>(data[0]);
    float result;
    std::memcpy(&result, &combined, sizeof(result));
    return result;
  }

  std::thread rec_thread_;
  std::unordered_map<MotorId, Motor *> motors_;
  LibSerial::SerialPort serial_;
  std::mutex serial_mutex_;
  std::unordered_map<int, DmActData> * data_ptr_;
  std::mutex latest_feedback_mutex_;
  std::unordered_map<MotorId, LatestFeedback> latest_feedback_;
  std::atomic<bool> stop_thread_ {false};
  std::chrono::duration<double> read_error_log_interval_ {0.5};
  std::size_t receive_timeout_ms_ {5};
  can_send_frame send_data_;
  CAN_Receive_Frame receive_data_ {};
};

LibSerial::BaudRate intToBaudRate(int baud_rate);

}  // namespace r2_arm_control
