"""Differential-drive odometry math with no ROS dependency."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class OdometryState:
    x: float
    y: float
    yaw: float
    linear_velocity: float
    angular_velocity: float


class DifferentialOdometry:
    def __init__(self, wheel_radius: float, wheel_separation: float,
                 encoder_counts_per_rev: float):
        if wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be positive")
        if wheel_separation <= 0.0:
            raise ValueError("wheel_separation must be positive")
        if encoder_counts_per_rev <= 0.0:
            raise ValueError("encoder_counts_per_rev must be positive")
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.encoder_counts_per_rev = encoder_counts_per_rev
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def update(self, left_delta: int, right_delta: int,
               dt: float) -> OdometryState:
        if dt <= 0.0 or not math.isfinite(dt):
            raise ValueError("dt must be finite and positive")

        metres_per_count = (
            2.0 * math.pi * self.wheel_radius / self.encoder_counts_per_rev)
        left_distance = left_delta * metres_per_count
        right_distance = right_delta * metres_per_count
        distance = 0.5 * (left_distance + right_distance)
        delta_yaw = (right_distance - left_distance) / self.wheel_separation

        heading_midpoint = self.yaw + 0.5 * delta_yaw
        self.x += distance * math.cos(heading_midpoint)
        self.y += distance * math.sin(heading_midpoint)
        self.yaw = self._normalize_angle(self.yaw + delta_yaw)

        return OdometryState(
            self.x,
            self.y,
            self.yaw,
            distance / dt,
            delta_yaw / dt,
        )

