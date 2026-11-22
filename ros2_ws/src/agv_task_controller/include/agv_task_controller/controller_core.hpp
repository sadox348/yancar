#pragma once

#include <algorithm>
#include <cmath>

namespace agv_task_controller
{

enum class MotionState { rotate, drive, reached };

struct Command
{
  double linear{0.0};
  double angular{0.0};
  MotionState state{MotionState::reached};
};

struct Limits
{
  double max_linear{0.25};
  double max_angular{0.8};
  double linear_gain{0.8};
  double angular_gain{1.8};
  double distance_tolerance{0.05};
  double heading_tolerance{0.15};
};

inline double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline Command compute_command(
  double x, double y, double yaw, double target_x, double target_y, const Limits & limits)
{
  const double dx = target_x - x;
  const double dy = target_y - y;
  const double distance = std::hypot(dx, dy);
  if (distance <= limits.distance_tolerance) {
    return {};
  }

  const double heading_error = normalize_angle(std::atan2(dy, dx) - yaw);
  const double angular = std::clamp(
    limits.angular_gain * heading_error, -limits.max_angular, limits.max_angular);
  if (std::abs(heading_error) > limits.heading_tolerance) {
    return {0.0, angular, MotionState::rotate};
  }

  const double linear = std::clamp(
    limits.linear_gain * distance, 0.0, limits.max_linear);
  return {linear, angular, MotionState::drive};
}

}  // namespace agv_task_controller
