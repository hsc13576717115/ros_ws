#include "r2_arm_control/ik_solver.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

constexpr double kHalfPi = 1.5707963267948966;
constexpr double kPi = 3.1415926535897932;

TEST(IkSolverTest, ForwardMatchesZeroPoseExpectation)
{
  r2_arm_control::ArmParams params;
  params.d1 = 0.30;
  params.d2 = 0.30;

  r2_arm_control::IkSolver solver(params);
  const auto fk = solver.forward(0.0, 0.0);

  EXPECT_NEAR(fk.x, 0.30, 1e-9);
  EXPECT_NEAR(fk.z, 0.30, 1e-9);
}

TEST(IkSolverTest, ForwardMatchesQuarterTurnForearmAbsolutePose)
{
  r2_arm_control::ArmParams params;
  r2_arm_control::IkSolver solver(params);

  const auto fk = solver.forward(0.0, kHalfPi);

  EXPECT_NEAR(fk.x, 0.0, 1e-9);
  EXPECT_NEAR(fk.z, 0.0, 1e-9);
}

TEST(IkSolverTest, ForwardMatchesShoulderPlusNinetyPose)
{
  r2_arm_control::ArmParams params;
  r2_arm_control::IkSolver solver(params);

  const auto fk = solver.forward(kHalfPi, 0.0);

  EXPECT_NEAR(fk.x, 0.60, 1e-9);
  EXPECT_NEAR(fk.z, 0.0, 1e-9);
}

TEST(IkSolverTest, SolveReturnsExpectedZeroPoseForNominalTarget)
{
  r2_arm_control::ArmParams params;
  r2_arm_control::IkSolver solver(params);

  const auto ik = solver.solve(0.30, 0.30, true);

  ASSERT_TRUE(ik.success);
  EXPECT_NEAR(ik.q1, 0.0, 1e-6);
  EXPECT_NEAR(ik.q2, 0.0, 1e-6);
}

TEST(IkSolverTest, SolveAndForwardStayConsistent)
{
  r2_arm_control::ArmParams params;
  params.q1_min = -kPi;
  params.q1_max = kPi;
  params.q2_min = -kPi;
  params.q2_max = kPi;

  r2_arm_control::IkSolver solver(params);
  const auto ik = solver.solve(0.18, 0.36, true);

  ASSERT_TRUE(ik.success);

  const auto fk = solver.forward(ik.q1, ik.q2);
  EXPECT_NEAR(fk.x, 0.18, 1e-6);
  EXPECT_NEAR(fk.z, 0.36, 1e-6);
}

TEST(IkSolverTest, SolveAndForwardStayConsistentForNegativeZTarget)
{
  r2_arm_control::ArmParams params;
  params.q1_min = -kPi;
  params.q1_max = kPi;
  params.q2_min = -kPi;
  params.q2_max = kPi;

  r2_arm_control::IkSolver solver(params);
  const auto ik = solver.solve(0.15, -0.10, true);

  ASSERT_TRUE(ik.success);

  const auto fk = solver.forward(ik.q1, ik.q2);
  EXPECT_NEAR(fk.x, 0.15, 1e-6);
  EXPECT_NEAR(fk.z, -0.10, 1e-6);
}

TEST(IkSolverTest, RejectsUnreachableTarget)
{
  r2_arm_control::ArmParams params;
  r2_arm_control::IkSolver solver(params);

  EXPECT_FALSE(solver.isReachable(1.00, 0.0));
  EXPECT_FALSE(solver.solve(1.00, 0.0, true).success);
}

TEST(IkSolverTest, RejectsTargetOutsideJointLimits)
{
  r2_arm_control::ArmParams params;
  params.q1_min = 0.10;
  params.q1_max = 0.20;
  params.q2_min = -0.10;
  params.q2_max = 0.10;

  r2_arm_control::IkSolver solver(params);
  EXPECT_FALSE(solver.solve(0.30, 0.30, true).success);
}

}  // namespace
