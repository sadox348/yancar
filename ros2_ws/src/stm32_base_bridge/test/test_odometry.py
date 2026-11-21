import math
import unittest

from stm32_base_bridge.odometry import DifferentialOdometry


class OdometryTest(unittest.TestCase):
    def setUp(self):
        # One count equals one metre to make expected values obvious.
        self.odom = DifferentialOdometry(
            wheel_radius=1.0 / (2.0 * math.pi),
            wheel_separation=2.0,
            encoder_counts_per_rev=1.0,
        )

    def test_straight_line(self):
        state = self.odom.update(1, 1, 0.5)
        self.assertAlmostEqual(state.x, 1.0)
        self.assertAlmostEqual(state.y, 0.0)
        self.assertAlmostEqual(state.yaw, 0.0)
        self.assertAlmostEqual(state.linear_velocity, 2.0)

    def test_in_place_rotation(self):
        state = self.odom.update(-1, 1, 1.0)
        self.assertAlmostEqual(state.x, 0.0)
        self.assertAlmostEqual(state.y, 0.0)
        self.assertAlmostEqual(state.yaw, 1.0)
        self.assertAlmostEqual(state.angular_velocity, 1.0)

    def test_single_wheel_arc(self):
        state = self.odom.update(0, 1, 1.0)
        self.assertGreater(state.x, 0.0)
        self.assertGreater(state.y, 0.0)
        self.assertAlmostEqual(state.yaw, 0.5)

    def test_zero_delta(self):
        state = self.odom.update(0, 0, 0.1)
        self.assertEqual((state.x, state.y, state.yaw), (0.0, 0.0, 0.0))
        self.assertEqual(state.linear_velocity, 0.0)
        self.assertEqual(state.angular_velocity, 0.0)

    def test_reject_bad_dt(self):
        with self.assertRaises(ValueError):
            self.odom.update(1, 1, 0.0)


if __name__ == "__main__":
    unittest.main()

