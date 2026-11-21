import math
import unittest

from stm32_base_bridge.protocol import (
    ProtocolError,
    encode_velocity_command,
    parse_frame,
)


class ProtocolTest(unittest.TestCase):
    def test_velocity_command_scaling(self):
        self.assertEqual(
            encode_velocity_command(0.2, -0.5, 1000.0, 1000.0, 900),
            "V,200,-500\r\n",
        )

    def test_velocity_command_clamping(self):
        self.assertEqual(
            encode_velocity_command(2.0, -2.0, 1000.0, 1000.0, 900),
            "V,900,-900\r\n",
        )

    def test_stop_command(self):
        self.assertEqual(
            encode_velocity_command(0.0, 0.0, 1000.0, 1000.0, 900),
            "V,0,0\r\n",
        )

    def test_reject_non_finite_command(self):
        with self.assertRaises(ValueError):
            encode_velocity_command(math.nan, 0.0, 1000.0, 1000.0, 900)

    def test_parse_stat(self):
        frame = parse_frame("STAT,1,20,18,130,22,19,125,100,101\r\n")
        self.assertEqual(frame.kind, "STAT")
        self.assertEqual(frame.stat.left_target, 20)
        self.assertEqual(frame.stat.left_actual, 18)
        self.assertEqual(frame.stat.right_actual, 19)
        self.assertEqual(frame.stat.right_pwm, 125)

    def test_parse_control_frames(self):
        self.assertEqual(parse_frame("ACK,V,200,0").kind, "ACK")
        self.assertEqual(parse_frame("EVT,BASE_TIMEOUT_STOP").kind, "EVT")
        self.assertEqual(parse_frame("ERR,PARSE").kind, "ERR")
        self.assertEqual(parse_frame("VER,0.9,DUAL_SPEED_LOOP_STATUS").kind, "VER")

    def test_reject_bad_stat_count(self):
        with self.assertRaises(ProtocolError):
            parse_frame("STAT,1,2,3")

    def test_reject_non_integer_stat(self):
        with self.assertRaises(ProtocolError):
            parse_frame("STAT,1,20,bad,130,22,19,125,100,101")

    def test_reject_empty_or_unknown(self):
        with self.assertRaises(ProtocolError):
            parse_frame("  ")
        with self.assertRaises(ProtocolError):
            parse_frame("GARBAGE,1,2")


if __name__ == "__main__":
    unittest.main()

