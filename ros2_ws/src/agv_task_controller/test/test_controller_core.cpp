#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "agv_task_controller/controller_core.hpp"

using agv_task_controller::Limits;
using agv_task_controller::MotionState;
using agv_task_controller::compute_command;
using agv_task_controller::is_finite_goal;

TEST(ControllerCore, StopsInsideGoalTolerance)
{
  const auto command = compute_command(0.98, 0.0, 0.0, 1.0, 0.0, Limits{});
  EXPECT_EQ(command.state, MotionState::reached);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}

TEST(ControllerCore, RotatesBeforeDriving)
{
  const auto command = compute_command(0.0, 0.0, 0.0, 0.0, 1.0, Limits{});
  EXPECT_EQ(command.state, MotionState::rotate);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_GT(command.angular, 0.0);
}

TEST(ControllerCore, DrivesTowardAlignedGoalWithinLimits)
{
  const auto command = compute_command(0.0, 0.0, 0.0, 10.0, 0.0, Limits{});
  EXPECT_EQ(command.state, MotionState::drive);
  EXPECT_DOUBLE_EQ(command.linear, 0.25);
  EXPECT_NEAR(command.angular, 0.0, 1e-12);
}

TEST(ControllerCore, UsesShortestHeadingErrorAcrossPi)
{
  const auto command = compute_command(0.0, 0.0, -3.13, -1.0, 0.0, Limits{});
  EXPECT_EQ(command.state, MotionState::drive);
  EXPECT_LT(std::abs(command.angular), 0.1);
}

TEST(ControllerCore, RejectsNonFiniteGoalCoordinates)
{
  EXPECT_TRUE(is_finite_goal(1.0, -2.0));
  EXPECT_FALSE(is_finite_goal(std::numeric_limits<double>::infinity(), 0.0));
  EXPECT_FALSE(is_finite_goal(0.0, std::numeric_limits<double>::quiet_NaN()));
}
