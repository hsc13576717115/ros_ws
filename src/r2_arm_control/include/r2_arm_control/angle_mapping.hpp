#pragma once

#include <cmath>
#include <string>

namespace r2_arm_control
{

struct JointAngleMapping
{
  std::string joint_name;
  double motor_sign {1.0};
  double zero_offset_rad {0.0};
  double transmission_ratio {1.0};
  std::string absolute_reference_joint_name;
  double position_min {-3.14159265358979323846};
  double position_max {3.14159265358979323846};
};

inline double sanitizeMotorSign(double configured, double fallback = 1.0)
{
  return std::fabs(configured) > 1e-9 ? configured : fallback;
}

// Physical zero and URDF zero are intentionally identical for this arm model.
inline double physicalToUrdf(double physical_angle_rad)
{
  return physical_angle_rad;
}

inline double urdfToPhysical(double urdf_angle_rad)
{
  return urdf_angle_rad;
}

inline double motorToPhysical(double motor_angle_rad, const JointAngleMapping & mapping)
{
  const double ratio = std::fabs(mapping.transmission_ratio) > 1e-9 ? mapping.transmission_ratio : 1.0;
  return sanitizeMotorSign(mapping.motor_sign) * (motor_angle_rad - mapping.zero_offset_rad) * ratio;
}

inline double physicalToMotor(double physical_angle_rad, const JointAngleMapping & mapping)
{
  const double ratio = std::fabs(mapping.transmission_ratio) > 1e-9 ? mapping.transmission_ratio : 1.0;
  return physical_angle_rad / (sanitizeMotorSign(mapping.motor_sign) * ratio) + mapping.zero_offset_rad;
}

inline double motorToPhysicalScalar(double motor_value, const JointAngleMapping & mapping)
{
  const double ratio = std::fabs(mapping.transmission_ratio) > 1e-9 ? mapping.transmission_ratio : 1.0;
  return sanitizeMotorSign(mapping.motor_sign) * motor_value * ratio;
}

inline double physicalToMotorScalar(double physical_value, const JointAngleMapping & mapping)
{
  const double ratio = std::fabs(mapping.transmission_ratio) > 1e-9 ? mapping.transmission_ratio : 1.0;
  return physical_value / (sanitizeMotorSign(mapping.motor_sign) * ratio);
}

inline double normalizeAngle(double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

}  // namespace r2_arm_control
