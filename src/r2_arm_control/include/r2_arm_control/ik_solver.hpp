#pragma once

namespace r2_arm_control
{

struct ArmParams
{
  double d1 {0.30};
  double d2 {0.30};
  // Forearm absolute-angle mounting offset. Zero means q2=0 points along +x.
  double forearm_mount_offset_rad {0.0};
  double q1_min {-0.80};
  double q1_max {1.40};
  double q2_min {-1.20};
  double q2_max {1.80};
};

struct IkResult
{
  bool success {false};
  double q1 {0.0};
  double q2 {0.0};
};

struct FkResult
{
  double x {0.0};
  double z {0.0};
};

class IkSolver
{
public:
  explicit IkSolver(const ArmParams & params);

  bool isReachable(double x, double z) const;
  // q1 is the upper-arm absolute angle from +z toward +x.
  // q2 is the forearm absolute angle from +x toward -z.
  IkResult solve(double x, double z, bool elbow_up) const;
  FkResult forward(double q1, double q2) const;

private:
  bool withinLimits(double q1, double q2) const;

  ArmParams params_;
};

}  // namespace r2_arm_control
