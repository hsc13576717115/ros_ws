#include "r2_arm_control/ik_solver.hpp"
#include "r2_arm_control/angle_mapping.hpp"

#include <algorithm>
#include <cmath>

namespace r2_arm_control
{

IkSolver::IkSolver(const ArmParams & params)
: params_(params)
{
}

bool IkSolver::isReachable(double x, double z) const
{
  const double r = std::sqrt(x * x + z * z);
  const double min_r = std::fabs(params_.d1 - params_.d2);
  const double max_r = params_.d1 + params_.d2;
  return (r >= min_r && r <= max_r);
}

bool IkSolver::withinLimits(double q1, double q2) const
{
  return (q1 >= params_.q1_min && q1 <= params_.q1_max &&
    q2 >= params_.q2_min && q2 <= params_.q2_max);
}

FkResult IkSolver::forward(double q1, double q2) const
{
  FkResult fk;
  const double forearm_world = q1 + q2;
  fk.x = params_.d1 * std::sin(q1) + params_.d2 * std::cos(forearm_world);
  fk.z = params_.d1 * std::cos(q1) - params_.d2 * std::sin(forearm_world);
  return fk;
}

IkResult IkSolver::solve(double x, double z, bool elbow_up) const
{
  IkResult result;

  if (!isReachable(x, z)) {
    return result;
  }

  const double d1 = params_.d1;
  const double d2 = params_.d2;
  const double r = std::sqrt(x * x + z * z);

  if (r < 1e-9) {
    return result;
  }

  const double a = (d1 * d1 - d2 * d2 + r * r) / (2.0 * r);
  const double h2 = d1 * d1 - a * a;
  if (h2 < 0.0) {
    return result;
  }
  const double h = std::sqrt(std::max(0.0, h2));

  const double px = a * x / r;
  const double pz = a * z / r;
  const double ux = -z / r;
  const double uz = x / r;

  const double ex = elbow_up ? (px + h * ux) : (px - h * ux);
  const double ez = elbow_up ? (pz + h * uz) : (pz - h * uz);

  const double q1 = std::atan2(ex, ez);

  const double vx = x - ex;
  const double vz = z - ez;
  const double forearm_world = std::atan2(-vz, vx);
  const double q2 = normalizeAngle(forearm_world - q1);

  if (!withinLimits(q1, q2)) {
    return result;
  }

  result.success = true;
  result.q1 = q1;
  result.q2 = q2;
  return result;
}

}  // namespace r2_arm_control
