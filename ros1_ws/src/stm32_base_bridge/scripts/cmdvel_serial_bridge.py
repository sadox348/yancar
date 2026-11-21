#!/usr/bin/env python3
"""ROS 1 Noetic bridge between /cmd_vel and the STM32 base controller."""

import threading
import time

import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String

from protocol import ProtocolError, encode_velocity_command, parse_frame


class CmdVelSerialBridge:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.linear_scale = float(rospy.get_param("~linear_scale", 1000.0))
        self.angular_scale = float(rospy.get_param("~angular_scale", 1000.0))
        self.max_command = int(rospy.get_param("~max_command", 900))
        self.cmd_vel_timeout = float(rospy.get_param("~cmd_vel_timeout", 0.4))

        self._serial_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._last_cmd_time = time.monotonic()
        self._command_active = False
        self._timeout_stop_sent = False

        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        rospy.loginfo("Opened serial port %s at %d", self.port, self.baud)

        self.status_pub = rospy.Publisher("/stm32_status", String, queue_size=20)
        self.encoder_pub = rospy.Publisher(
            "/wheel_encoder_delta", Int32MultiArray, queue_size=20)
        self.target_pub = rospy.Publisher(
            "/wheel_target_delta", Int32MultiArray, queue_size=20)
        self.pwm_pub = rospy.Publisher("/wheel_pwm", Int32MultiArray, queue_size=20)
        self.cmd_sub = rospy.Subscriber(
            "/cmd_vel", Twist, self.cmd_vel_callback, queue_size=10)

        self.rx_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.rx_thread.start()
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.05), self.watchdog_callback)
        rospy.on_shutdown(self.shutdown)

    def _write(self, frame):
        with self._serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.write(frame.encode("ascii"))
                self.ser.flush()

    def cmd_vel_callback(self, msg):
        try:
            frame = encode_velocity_command(
                msg.linear.x, msg.angular.z,
                self.linear_scale, self.angular_scale, self.max_command)
            self._write(frame)
        except (ValueError, serial.SerialException) as exc:
            rospy.logerr_throttle(1.0, "Failed to send cmd_vel: %s", exc)
            return

        self._last_cmd_time = time.monotonic()
        self._command_active = not (
            abs(msg.linear.x) < 1e-9 and abs(msg.angular.z) < 1e-9)
        self._timeout_stop_sent = False
        rospy.loginfo_throttle(0.5, "TX -> STM32: %s", frame.strip())

    def watchdog_callback(self, _event):
        if not self._command_active or self._timeout_stop_sent:
            return
        if time.monotonic() - self._last_cmd_time <= self.cmd_vel_timeout:
            return
        try:
            self._write("V,0,0\r\n")
            self._timeout_stop_sent = True
            self._command_active = False
            rospy.logwarn("cmd_vel timeout: sent V,0,0")
        except serial.SerialException as exc:
            rospy.logerr_throttle(1.0, "Failed to send timeout stop: %s", exc)

    def read_serial_loop(self):
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                raw = self.ser.readline().decode("ascii", errors="replace").strip()
            except serial.SerialException as exc:
                rospy.logerr_throttle(1.0, "Serial read error: %s", exc)
                self._stop_event.wait(0.2)
                continue
            if not raw:
                continue

            self.status_pub.publish(String(data=raw))
            try:
                parsed = parse_frame(raw)
            except ProtocolError as exc:
                rospy.logwarn_throttle(1.0, "Ignored malformed frame %r: %s", raw, exc)
                continue

            if parsed.stat is not None:
                stat = parsed.stat
                self.encoder_pub.publish(Int32MultiArray(
                    data=[stat.left_actual, stat.right_actual]))
                self.target_pub.publish(Int32MultiArray(
                    data=[stat.left_target, stat.right_target]))
                self.pwm_pub.publish(Int32MultiArray(
                    data=[stat.left_pwm, stat.right_pwm]))
            elif parsed.kind in {"EVT", "ERR"}:
                rospy.logwarn("STM32 %s: %s", parsed.kind, ",".join(parsed.fields))

    def shutdown(self):
        self._stop_event.set()
        try:
            self._write("V,0,0\r\n")
        except serial.SerialException:
            pass
        if self.ser and self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
    rospy.init_node("stm32_base_bridge")
    CmdVelSerialBridge()
    rospy.spin()

