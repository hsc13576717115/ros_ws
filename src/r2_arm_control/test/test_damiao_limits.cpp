#include "r2_arm_control/damiao.hpp"

#include <gtest/gtest.h>

#include <limits>

TEST(DamiaoMitCommandTest, ClampsToMitRanges)
{
  const r2_arm_control::Limit_param limits {12.5f, 30.0f, 10.0f};

  const auto command = r2_arm_control::sanitizeMitCommandInputs(
    limits,
    999.0f,
    9.0f,
    20.0f,
    -50.0f,
    30.0f);

  EXPECT_FLOAT_EQ(command.kp, 500.0f);
  EXPECT_FLOAT_EQ(command.kd, 5.0f);
  EXPECT_FLOAT_EQ(command.q, 12.5f);
  EXPECT_FLOAT_EQ(command.dq, -30.0f);
  EXPECT_FLOAT_EQ(command.tau, 10.0f);
}

TEST(DamiaoMitCommandTest, ReplacesInvalidNumbersWithSafeDefaults)
{
  const r2_arm_control::Limit_param limits {12.5f, 30.0f, 10.0f};

  const auto command = r2_arm_control::sanitizeMitCommandInputs(
    limits,
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity());

  EXPECT_FLOAT_EQ(command.kp, 0.0f);
  EXPECT_FLOAT_EQ(command.kd, 0.0f);
  EXPECT_FLOAT_EQ(command.q, 0.0f);
  EXPECT_FLOAT_EQ(command.dq, 0.0f);
  EXPECT_FLOAT_EQ(command.tau, 0.0f);
}
