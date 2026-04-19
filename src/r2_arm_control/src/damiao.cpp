#include "r2_arm_control/damiao.hpp"

#include <iostream>
#include <unistd.h>

namespace r2_arm_control
{

namespace
{

bool is_valid_feedback_frame(const CAN_Receive_Frame & frame)
{
  return frame.CMD == 0x11 && frame.frameEnd == 0x55;
}

float clampFinite(float value, float lower, float upper, float fallback = 0.0f)
{
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, lower, upper);
}

}  // namespace

Limit_param limit_param[Num_Of_Motor] = {
  {12.5f, 30.0f, 10.0f},
  {12.5f, 50.0f, 10.0f},
  {12.5f, 10.0f, 28.0f},
  {12.5f, 10.0f, 28.0f},
  {12.5f, 45.0f, 12.0f},
  {12.5f, 45.0f, 20.0f},
  {12.5f, 45.0f, 54.0f},
  {12.5f, 25.0f, 200.0f},
  {12.5f, 20.0f, 200.0f},
  {12.5f, 280.0f, 1.0f},
  {12.5f, 45.0f, 10.0f},
  {12.5f, 45.0f, 10.0f}
};

MitCommand sanitizeMitCommandInputs(
  const Limit_param & limits,
  float kp,
  float kd,
  float q,
  float dq,
  float tau)
{
  MitCommand command;
  command.kp = clampFinite(kp, 0.0f, 500.0f);
  command.kd = clampFinite(kd, 0.0f, 5.0f);
  command.q = clampFinite(q, -limits.Q_MAX, limits.Q_MAX);
  command.dq = clampFinite(dq, -limits.DQ_MAX, limits.DQ_MAX);
  command.tau = clampFinite(tau, -limits.TAU_MAX, limits.TAU_MAX);
  return command;
}

Motor::Motor(DM_Motor_Type motor_type, MotorId slave_id, MotorId master_id)
: master_id_(master_id), slave_id_(slave_id), motor_type_(motor_type)
{
  limit_param_ = r2_arm_control::limit_param[motor_type];
}

void Motor::receive_data(float q, float dq, float tau)
{
  state_q_ = q;
  state_dq_ = dq;
  state_tau_ = tau;
}

void Motor::set_param(int key, float value)
{
  ValueType v {};
  v.value.floatValue = value;
  v.isFloat = true;
  param_map_[key] = v;
}

void Motor::set_param(int key, uint32_t value)
{
  ValueType v {};
  v.value.uint32Value = value;
  v.isFloat = false;
  param_map_[key] = v;
}

float Motor::get_param_as_float(int key) const
{
  const auto it = param_map_.find(key);
  if (it == param_map_.end() || !it->second.isFloat) {
    return 0.0f;
  }
  return it->second.value.floatValue;
}

uint32_t Motor::get_param_as_uint32(int key) const
{
  const auto it = param_map_.find(key);
  if (it == param_map_.end() || it->second.isFloat) {
    return 0U;
  }
  return it->second.value.uint32Value;
}

bool Motor::is_have_param(int key) const
{
  return param_map_.find(key) != param_map_.end();
}

Motor_Control::Motor_Control(
  std::string serial_port,
  int baud_rate,
  std::unordered_map<int, DmActData> * data_ptr,
  double read_error_log_interval_sec)
: data_ptr_(data_ptr),
  serial_port_(std::move(serial_port)),
  read_error_log_interval_(std::chrono::duration<double>(std::max(0.05, read_error_log_interval_sec)))
{
  for (auto it = data_ptr_->begin(); it != data_ptr_->end(); ++it) {
    auto * motor = new Motor(it->second.motorType, it->second.can_id, it->second.mst_id);
    addMotor(motor);
    latest_feedback_[motor->GetSlaveId()] = LatestFeedback {};
  }

  try {
    const LibSerial::BaudRate baud_rate_enum = intToBaudRate(baud_rate);
    std::cerr << "Configuring serial port " << serial_port_
              << " with baud rate " << baud_rate << std::endl;

    serial_.Open(serial_port_);
    serial_.SetBaudRate(baud_rate_enum);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    usleep(1000000);
  } catch (const std::exception & e) {
    std::cerr << "Failed to initialize serial port " << serial_port_
              << ". Error: " << e.what() << std::endl;
    throw std::runtime_error("Motor_Control initialization failed");
  }
}

Motor_Control::~Motor_Control()
{
  stop_receive_thread();

  for (const auto & m : *data_ptr_) {
    const int motor_id = m.first;
    if (motors_.find(motor_id) != motors_.end()) {
      control_mit(*motors_[motor_id], 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);
    }
  }

  if (serial_.IsOpen()) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_.Close();
  }

  std::unordered_map<Motor *, bool> freed;
  for (const auto & [id, motor_ptr] : motors_) {
    (void)id;
    if (!freed[motor_ptr]) {
      delete motor_ptr;
      freed[motor_ptr] = true;
    }
  }
  motors_.clear();
}

void Motor_Control::start_receive_thread()
{
  if (rec_thread_.joinable()) {
    return;
  }

  stop_thread_ = false;
  rec_thread_ = std::thread(&Motor_Control::get_motor_data_thread, this);
}

void Motor_Control::stop_receive_thread()
{
  stop_thread_ = true;
  if (rec_thread_.joinable()) {
    rec_thread_.join();
  }
}

void Motor_Control::enable()
{
  for (const auto & m : *data_ptr_) {
    const int motor_id = m.first;
    if (motors_.find(motor_id) == motors_.end()) {
      continue;
    }
    for (int j = 0; j < 20; ++j) {
      control_cmd(motors_[motor_id]->GetSlaveId(), 0xFC);
      usleep(3000);
    }
  }
}

void Motor_Control::disable()
{
  for (const auto & m : *data_ptr_) {
    const int motor_id = m.first;
    if (motors_.find(motor_id) == motors_.end()) {
      continue;
    }
    for (int j = 0; j < 20; ++j) {
      control_cmd(motors_[motor_id]->GetSlaveId(), 0xFD);
      usleep(3000);
    }
  }
}

void Motor_Control::set_zero_position()
{
  for (const auto & m : *data_ptr_) {
    const int motor_id = m.first;
    if (motors_.find(motor_id) == motors_.end()) {
      continue;
    }
    for (int j = 0; j < 20; ++j) {
      control_cmd(motors_[motor_id]->GetSlaveId(), 0xFE);
      usleep(3000);
    }
  }
}

bool Motor_Control::switch_mode_all(Control_Mode mode)
{
  bool all_ok = true;

  for (const auto & m : *data_ptr_) {
    const int motor_id = m.first;
    if (motors_.find(motor_id) == motors_.end()) {
      std::cerr << "switch_mode_all ERROR: motor id not found: " << motor_id << std::endl;
      all_ok = false;
      continue;
    }

    auto * motor = motors_[motor_id];
    bool ok = false;

    for (int outer_retry = 0; outer_retry < 5; ++outer_retry) {
      usleep(150000);
      if (switchControlMode(*motor, mode)) {
        ok = true;
        break;
      }
      std::cerr << "Retry switch motor " << motor_id
                << " to mode " << static_cast<int>(mode)
                << ", attempt " << (outer_retry + 1) << "/5" << std::endl;
    }

    if (!ok) {
      std::cerr << "Failed to switch motor " << motor_id
                << " to mode " << static_cast<int>(mode) << std::endl;
      all_ok = false;
    }

    usleep(200000);
  }

  return all_ok;
}

void Motor_Control::refresh_motor_status(const Motor & motor)
{
  const uint32_t id = 0x7FF;
  const uint8_t can_low = motor.GetSlaveId() & 0xff;
  const uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff;
  std::array<uint8_t, 8> data_buf = {can_low, can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_data_.modify(id, data_buf.data());
  WriteData(send_data_);
}

void Motor_Control::control_mit(
  Motor & motor,
  float kp,
  float kd,
  float q,
  float dq,
  float tau)
{
  static const auto float_to_uint =
    [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
      const float span = xmax - xmin;
      if (span <= 0.0f) {
        return 0;
      }
      const float data_norm = std::clamp((x - xmin) / span, 0.0f, 1.0f);
      return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
    };

  const MotorId id = motor.GetSlaveId();
  if (motors_.find(id) == motors_.end()) {
    throw std::runtime_error("Motor_Control id not found");
  }

  const Limit_param limits = motor.get_limit_param();
  const MitCommand command = sanitizeMitCommandInputs(limits, kp, kd, q, dq, tau);

  const uint16_t kp_uint = float_to_uint(command.kp, 0, 500, 12);
  const uint16_t kd_uint = float_to_uint(command.kd, 0, 5, 12);
  const uint16_t q_uint = float_to_uint(command.q, -limits.Q_MAX, limits.Q_MAX, 16);
  const uint16_t dq_uint = float_to_uint(command.dq, -limits.DQ_MAX, limits.DQ_MAX, 12);
  const uint16_t tau_uint = float_to_uint(command.tau, -limits.TAU_MAX, limits.TAU_MAX, 12);

  std::array<uint8_t, 8> data_buf {};
  data_buf[0] = (q_uint >> 8) & 0xff;
  data_buf[1] = q_uint & 0xff;
  data_buf[2] = dq_uint >> 4;
  data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
  data_buf[4] = kp_uint & 0xff;
  data_buf[5] = kd_uint >> 4;
  data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
  data_buf[7] = tau_uint & 0xff;

  send_data_.modify(id, data_buf.data());
  WriteData(send_data_);
}

void Motor_Control::control_pos_vel(Motor & motor, float pos, float vel)
{
  MotorId id = motor.GetSlaveId();
  if (motors_.find(id) == motors_.end()) {
    throw std::runtime_error("POS_VEL ERROR: Motor_Control id not found");
  }
  std::array<uint8_t, 8> data_buf {};
  std::memcpy(data_buf.data(), &pos, sizeof(float));
  std::memcpy(data_buf.data() + 4, &vel, sizeof(float));
  id += POS_MODE;
  send_data_.modify(id, data_buf.data());
  WriteData(send_data_);
}

void Motor_Control::control_vel(Motor & motor, float vel)
{
  MotorId id = motor.GetSlaveId();
  if (motors_.find(id) == motors_.end()) {
    throw std::runtime_error("VEL ERROR: id not found");
  }
  std::array<uint8_t, 8> data_buf = {0};
  std::memcpy(data_buf.data(), &vel, sizeof(float));
  id += SPEED_MODE;
  send_data_.modify(id, data_buf.data());
  WriteData(send_data_);
}

void Motor_Control::receive_param()
{
  if (receive_data_.CMD != 0x11 || receive_data_.frameEnd != 0x55) {
    return;
  }

  auto & data = receive_data_.canData;
  if (data[2] != 0x33 && data[2] != 0x55) {
    return;
  }

  const uint32_t slave_id = (uint32_t(data[1]) << 8) | data[0];
  const uint8_t rid = data[3];
  if (motors_.find(slave_id) == motors_.end()) {
    return;
  }

  if (is_in_ranges(rid)) {
    const uint32_t data_uint32 = (uint32_t(data[7]) << 24) |
      (uint32_t(data[6]) << 16) |
      (uint32_t(data[5]) << 8) | data[4];
    motors_[slave_id]->set_param(rid, data_uint32);
  } else {
    const float data_float = uint8_to_float(data + 4);
    motors_[slave_id]->set_param(rid, data_float);
  }
}

void Motor_Control::addMotor(Motor * motor)
{
  motors_.insert({motor->GetSlaveId(), motor});
  motors_.insert({motor->GetMasterId(), motor});
}

float Motor_Control::read_motor_param(Motor & motor, uint8_t rid)
{
  const uint32_t id = motor.GetSlaveId();
  const uint8_t can_low = id & 0xff;
  const uint8_t can_high = (id >> 8) & 0xff;
  std::array<uint8_t, 8> data_buf{can_low, can_high, 0x33, rid, 0x00, 0x00, 0x00, 0x00};
  send_data_.modify(0x7FF, data_buf.data());
  WriteData(send_data_);

  for (uint8_t i = 0; i < kMaxRetries; ++i) {
    usleep(kRetryIntervalUs);
    if (!ReadData(receive_data_, 20)) {
      continue;
    }
    receive_param();
    if (!motors_[motor.GetSlaveId()]->is_have_param(rid)) {
      continue;
    }
    if (is_in_ranges(rid)) {
      return static_cast<float>(motors_[motor.GetSlaveId()]->get_param_as_uint32(rid));
    }
    return motors_[motor.GetSlaveId()]->get_param_as_float(rid);
  }
  return 0.0f;
}

bool Motor_Control::switchControlMode(Motor & motor, Control_Mode mode)
{
  constexpr uint8_t kControlModeRid = CTRL_MODE;
  const uint8_t write_data[4] = {static_cast<uint8_t>(mode), 0x00, 0x00, 0x00};

  if (motors_.find(motor.GetSlaveId()) == motors_.end()) {
    return false;
  }

  for (int write_retry = 0; write_retry < 5; ++write_retry) {
    write_motor_param(motor, kControlModeRid, write_data);
    usleep(120000);

    for (int confirm_retry = 0; confirm_retry < 8; ++confirm_retry) {
      if (ReadData(receive_data_, 30)) {
        receive_param();
      }

      if (motors_[motor.GetSlaveId()]->is_have_param(kControlModeRid)) {
        const auto value = motors_[motor.GetSlaveId()]->get_param_as_uint32(kControlModeRid);
        if (value == static_cast<uint32_t>(mode)) {
          return true;
        }
      }
      usleep(30000);
    }

    usleep(150000);
  }

  return false;
}

bool Motor_Control::change_motor_param(Motor & motor, uint8_t rid, float data)
{
  if (is_in_ranges(rid)) {
    const uint32_t data_uint32 = float_to_uint32(data);
    const auto * data_uint8 = reinterpret_cast<const uint8_t *>(&data_uint32);
    write_motor_param(motor, rid, data_uint8);
  } else {
    const auto * data_uint8 = reinterpret_cast<const uint8_t *>(&data);
    write_motor_param(motor, rid, data_uint8);
  }

  if (motors_.find(motor.GetSlaveId()) == motors_.end()) {
    return false;
  }

  for (uint8_t i = 0; i < kMaxRetries; ++i) {
    usleep(kRetryIntervalUs);
    if (!ReadData(receive_data_, 20)) {
      continue;
    }
    receive_param();
    if (!motors_[motor.GetSlaveId()]->is_have_param(rid)) {
      continue;
    }
    if (is_in_ranges(rid)) {
      return motors_[motor.GetSlaveId()]->get_param_as_uint32(rid) == float_to_uint32(data);
    }
    return std::fabs(motors_[motor.GetSlaveId()]->get_param_as_float(rid) - data) < 0.1f;
  }
  return false;
}

void Motor_Control::save_motor_param(Motor & motor)
{
  disable();
  const uint32_t id = motor.GetSlaveId();
  const uint8_t id_low = id & 0xff;
  const uint8_t id_high = (id >> 8) & 0xff;
  std::array<uint8_t, 8> data_buf{id_low, id_high, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00};
  send_data_.modify(0x7FF, data_buf.data());
  WriteData(send_data_);
  usleep(100000);
}

void Motor_Control::changeMotorLimit(Motor & motor, float p_max, float q_max, float t_max)
{
  limit_param[motor.GetMotorType()] = {p_max, q_max, t_max};
}

void Motor_Control::control_cmd(MotorId id, uint8_t cmd)
{
  std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
  send_data_.modify(id, data_buf.data());
  WriteData(send_data_);
}

void Motor_Control::write_motor_param(Motor & motor, uint8_t rid, const uint8_t data[4])
{
  const uint32_t id = motor.GetSlaveId();
  const uint8_t can_low = id & 0xff;
  const uint8_t can_high = (id >> 8) & 0xff;
  std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, rid, 0x00, 0x00, 0x00, 0x00};
  data_buf[4] = data[0];
  data_buf[5] = data[1];
  data_buf[6] = data[2];
  data_buf[7] = data[3];
  send_data_.modify(0x7FF, data_buf.data());
  WriteData(send_data_);
}

MotorId Motor_Control::resolve_feedback_motor_key(const CAN_Receive_Frame & frame) const
{
  if (!is_valid_feedback_frame(frame)) {
    return 0;
  }

  const MotorId embedded_motor_id = static_cast<MotorId>(frame.canData[0] & 0x0F);
  if (embedded_motor_id != 0) {
    const auto embedded_it = motors_.find(embedded_motor_id);
    if (embedded_it != motors_.end()) {
      return embedded_it->second->GetSlaveId();
    }
  }

  if (frame.canId != 0) {
    const auto can_id_it = motors_.find(frame.canId);
    if (can_id_it != motors_.end()) {
      return can_id_it->second->GetSlaveId();
    }
  }

  return 0;
}

bool Motor_Control::feedback_frame_matches_motor(
  const CAN_Receive_Frame & frame,
  const Motor & motor) const
{
  return resolve_feedback_motor_key(frame) == motor.GetSlaveId();
}

bool Motor_Control::decode_feedback_frame(
  const CAN_Receive_Frame & frame,
  const Motor & motor,
  float & q,
  float & dq,
  float & tau) const
{
  if (!feedback_frame_matches_motor(frame, motor)) {
    return false;
  }

  static const auto uint_to_float =
    [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
      const float span = xmax - xmin;
      const float data_norm = static_cast<float>(x) / ((1 << bits) - 1);
      return data_norm * span + xmin;
    };

  const auto & data = frame.canData;
  const uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
  const uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
  const uint16_t tau_uint = (uint16_t(data[4] & 0x0f) << 8) | data[5];

  const Limit_param limits = motor.get_limit_param();
  q = uint_to_float(q_uint, -limits.Q_MAX, limits.Q_MAX, 16);
  dq = uint_to_float(dq_uint, -limits.DQ_MAX, limits.DQ_MAX, 12);
  tau = uint_to_float(tau_uint, -limits.TAU_MAX, limits.TAU_MAX, 12);
  return true;
}

void Motor_Control::write()
{
  for (const auto & m : *data_ptr_) {
    const int motor_id = m.first;
    if (motors_.find(motor_id) == motors_.end()) {
      std::cerr << "write WARNING: Motor_Control id not found: " << motor_id << std::endl;
      continue;
    }
    auto & motor = motors_[motor_id];
    control_mit(
      *motor,
      static_cast<float>(m.second.kp),
      static_cast<float>(m.second.kd),
      static_cast<float>(m.second.cmd_pos),
      static_cast<float>(m.second.cmd_vel),
      static_cast<float>(m.second.cmd_effort));
  }
}

void Motor_Control::read()
{
  const auto now_time = std::chrono::steady_clock::now();

  for (auto & m : *data_ptr_) {
    const int motor_id = m.first;
    if (motors_.find(motor_id) == motors_.end()) {
      std::cerr << "read WARNING: Motor_Control id not found: " << motor_id << std::endl;
      continue;
    }

    auto & motor = motors_[motor_id];
    LatestFeedback latest_feedback {};
    bool got_valid_feedback = false;

    {
      std::lock_guard<std::mutex> lock(latest_feedback_mutex_);
      const auto feedback_it = latest_feedback_.find(motor->GetSlaveId());
      if (feedback_it != latest_feedback_.end() && feedback_it->second.valid) {
        latest_feedback = feedback_it->second;
        got_valid_feedback = true;
      }
    }

    if (got_valid_feedback) {
      motor->receive_data(latest_feedback.q, latest_feedback.dq, latest_feedback.tau);
      if (
        !m.second.has_valid_feedback ||
        latest_feedback.received_at > m.second.last_valid_feedback_time)
      {
        m.second.last_valid_feedback_time = latest_feedback.received_at;
        m.second.consecutive_invalid_frames = 0;
      }
      m.second.has_valid_feedback = true;
    } else {
      ++m.second.consecutive_invalid_frames;
      if (
        m.second.last_invalid_log_time.time_since_epoch().count() == 0 ||
        (now_time - m.second.last_invalid_log_time) >= read_error_log_interval_)
      {
        std::cerr << "[READ] motor_id=" << motor_id << " no valid feedback frame" << std::endl;
        m.second.last_invalid_log_time = now_time;
      }
    }

    m.second.pos = motor->Get_Position();
    m.second.vel = motor->Get_Velocity();
    m.second.effort = motor->Get_tau();
  }
}

void Motor_Control::get_motor_data_thread()
{
  while (!stop_thread_) {
    try {
      const auto now_time = std::chrono::steady_clock::now();
      if (
        last_receive_diag_log_time_.time_since_epoch().count() != 0 &&
        (now_time - last_receive_diag_log_time_) >= read_error_log_interval_)
      {
        std::cerr
          << "[RX-DIAG] port=" << serial_port_
          << " ok=" << receive_ok_count_
          << " timeout=" << receive_timeout_count_
          << " invalid_frame=" << receive_invalid_frame_count_
          << " unresolved=" << receive_unresolved_frame_count_
          << " unknown_motor=" << receive_unknown_motor_count_
          << " decode_fail=" << receive_decode_failure_count_
          << std::endl;
        receive_ok_count_ = 0;
        receive_timeout_count_ = 0;
        receive_invalid_frame_count_ = 0;
        receive_unresolved_frame_count_ = 0;
        receive_unknown_motor_count_ = 0;
        receive_decode_failure_count_ = 0;
        last_receive_diag_log_time_ = now_time;
      } else if (last_receive_diag_log_time_.time_since_epoch().count() == 0) {
        last_receive_diag_log_time_ = now_time;
      }

      CAN_Receive_Frame frame {};
      if (!ReadData(frame, receive_timeout_ms_)) {
        ++receive_timeout_count_;
        continue;
      }

      if (!is_valid_feedback_frame(frame)) {
        ++receive_invalid_frame_count_;
        continue;
      }

      const MotorId motor_key = resolve_feedback_motor_key(frame);
      if (motor_key == 0) {
        ++receive_unresolved_frame_count_;
        continue;
      }

      const auto motor_it = motors_.find(motor_key);
      if (motor_it == motors_.end()) {
        ++receive_unknown_motor_count_;
        std::cerr << "[RX-WARN] port=" << serial_port_
                  << " resolved unknown motor_key=" << motor_key
                  << " can_id=0x" << std::hex << frame.canId << std::dec << std::endl;
        continue;
      }

      auto * motor = motor_it->second;
      float receive_q = 0.0f;
      float receive_dq = 0.0f;
      float receive_tau = 0.0f;
      if (!decode_feedback_frame(frame, *motor, receive_q, receive_dq, receive_tau)) {
        ++receive_decode_failure_count_;
        std::cerr << "[RX-WARN] port=" << serial_port_
                  << " decode failed for motor_id=" << motor->GetSlaveId()
                  << " can_id=0x" << std::hex << frame.canId << std::dec << std::endl;
        continue;
      }

      const auto received_at = std::chrono::steady_clock::now();
      ++receive_ok_count_;
      std::lock_guard<std::mutex> lock(latest_feedback_mutex_);
      auto & cached_feedback = latest_feedback_[motor->GetSlaveId()];
      cached_feedback.q = receive_q;
      cached_feedback.dq = receive_dq;
      cached_feedback.tau = receive_tau;
      cached_feedback.received_at = received_at;
      cached_feedback.valid = true;
    } catch (const std::exception & e) {
      std::cerr << "receive thread exception: " << e.what() << std::endl;
      usleep(20000);
    } catch (...) {
      std::cerr << "receive thread unknown exception" << std::endl;
      usleep(20000);
    }
  }
}

void Motor_Control::WriteData(const can_send_frame & frame)
{
  const auto * frame_ptr = reinterpret_cast<const uint8_t *>(&frame);
  const size_t data_size = sizeof(can_send_frame);
  LibSerial::DataBuffer tx_buffer(frame_ptr, frame_ptr + data_size);
  std::lock_guard<std::mutex> lock(serial_mutex_);
  serial_.Write(tx_buffer);
}

bool Motor_Control::ReadData(CAN_Receive_Frame & frame, size_t timeout_ms)
{
  LibSerial::DataBuffer rx_buffer;
  const size_t bytes_to_read = sizeof(CAN_Receive_Frame);

  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_.Read(rx_buffer, bytes_to_read, timeout_ms);
    if (rx_buffer.size() == bytes_to_read) {
      std::memcpy(&frame, rx_buffer.data(), bytes_to_read);
      return true;
    }
    return false;
  } catch (const LibSerial::ReadTimeout &) {
    return false;
  } catch (const std::exception & e) {
    std::cerr << "Serial read exception: " << e.what() << std::endl;
    return false;
  }
}

LibSerial::BaudRate intToBaudRate(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return LibSerial::BaudRate::BAUD_9600;
    case 19200:
      return LibSerial::BaudRate::BAUD_19200;
    case 38400:
      return LibSerial::BaudRate::BAUD_38400;
    case 57600:
      return LibSerial::BaudRate::BAUD_57600;
    case 115200:
      return LibSerial::BaudRate::BAUD_115200;
    case 230400:
      return LibSerial::BaudRate::BAUD_230400;
    case 460800:
      return LibSerial::BaudRate::BAUD_460800;
    case 500000:
      return LibSerial::BaudRate::BAUD_500000;
    case 576000:
      return LibSerial::BaudRate::BAUD_576000;
    case 921600:
      return LibSerial::BaudRate::BAUD_921600;
    case 1000000:
      return LibSerial::BaudRate::BAUD_1000000;
    case 1152000:
      return LibSerial::BaudRate::BAUD_1152000;
    case 1500000:
      return LibSerial::BaudRate::BAUD_1500000;
    default:
      throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud_rate));
  }
}

}  // namespace r2_arm_control
