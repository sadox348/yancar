"""ROS 2 Humble serial bridge for the STM32F407 differential-drive base."""

import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String
from tf2_ros import TransformBroadcaster

from .odometry import DifferentialOdometry
from .protocol import ProtocolError, encode_velocity_command, parse_frame
from .serial_transport import SerialTransport


class Stm32BaseBridge(Node):
    def __init__(self):
        super().__init__("stm32_base_bridge")

        defaults = {
            "port": "/dev/ttyUSB0",
            "baud": 115200,
            "linear_scale": 1000.0,
            "angular_scale": 1000.0,
            "max_command": 900,
            "cmd_vel_timeout": 0.4,
            "command_rate": 10.0,
            "reconnect_interval": 2.0,
            "wheel_radius": 0.05,
            "wheel_separation": 0.30,
            "encoder_counts_per_rev": 1560.0,
            "publish_tf": True,
            "odom_frame": "odom",
            "base_frame": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.max_command = int(self.get_parameter("max_command").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        command_rate = float(self.get_parameter("command_rate").value)
        reconnect_interval = float(self.get_parameter("reconnect_interval").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        if command_rate <= 0.0:
            raise ValueError("command_rate must be positive")
        if self.cmd_vel_timeout <= 0.0:
            raise ValueError("cmd_vel_timeout must be positive")

        self.odometry = DifferentialOdometry(
            float(self.get_parameter("wheel_radius").value),
            float(self.get_parameter("wheel_separation").value),
            float(self.get_parameter("encoder_counts_per_rev").value),
        )
        self.transport = SerialTransport(self.port, self.baud)
        self._stop_event = threading.Event()
        self._command_lock = threading.Lock()
        self._latest_command = (0.0, 0.0)
        self._last_command_time = time.monotonic()
        self._command_received = False
        self._timeout_stop_sent = False
        self._last_stat_time = None

        self.raw_pub = self.create_publisher(String, "/stm32/status_raw", 20)
        self.event_pub = self.create_publisher(String, "/stm32/event", 20)
        self.connected_pub = self.create_publisher(Bool, "/stm32/connected", 1)
        self.encoder_pub = self.create_publisher(
            Int32MultiArray, "/wheel_encoder_delta", 20)
        self.target_pub = self.create_publisher(
            Int32MultiArray, "/wheel_target_delta", 20)
        self.pwm_pub = self.create_publisher(Int32MultiArray, "/wheel_pwm", 20)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10)

        self.command_timer = self.create_timer(1.0 / command_rate, self._command_tick)
        self.reconnect_timer = self.create_timer(
            reconnect_interval, self._connect_if_needed)
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)

        self._connect_if_needed()
        self.reader_thread.start()

    def _publish_connected(self, connected: bool) -> None:
        self.connected_pub.publish(Bool(data=connected))

    def _connect_if_needed(self) -> None:
        if self.transport.is_open:
            return
        try:
            self.transport.open()
        except serial.SerialException as exc:
            self._publish_connected(False)
            self.get_logger().warning(f"Serial connection failed: {exc}")
            return
        self._publish_connected(True)
        self.get_logger().info(f"Opened {self.port} at {self.baud} baud")

    def _cmd_vel_callback(self, msg: Twist) -> None:
        values = (msg.linear.x, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("Ignored non-finite cmd_vel")
            return
        with self._command_lock:
            self._latest_command = values
            self._last_command_time = time.monotonic()
            self._command_received = True
            self._timeout_stop_sent = False

    def _send_command(self, linear: float, angular: float) -> bool:
        try:
            frame = encode_velocity_command(
                linear, angular, self.linear_scale,
                self.angular_scale, self.max_command)
            self.transport.write(frame)
            return True
        except (ValueError, serial.SerialException) as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
            self.transport.close()
            self._publish_connected(False)
            return False

    def _command_tick(self) -> None:
        with self._command_lock:
            if not self._command_received:
                return
            age = time.monotonic() - self._last_command_time
            if age > self.cmd_vel_timeout:
                if self._timeout_stop_sent:
                    return
                command = (0.0, 0.0)
                self._timeout_stop_sent = True
            else:
                command = self._latest_command

        if self.transport.is_open and self._send_command(*command):
            if command == (0.0, 0.0) and age > self.cmd_vel_timeout:
                self.get_logger().warning("cmd_vel timeout: sent V,0,0")

    def _read_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            if not self.transport.is_open:
                self._stop_event.wait(0.1)
                continue
            try:
                raw = self.transport.read_line()
            except serial.SerialException as exc:
                self.get_logger().error(f"Serial read failed: {exc}")
                self.transport.close()
                self._publish_connected(False)
                continue
            if raw:
                self._handle_line(raw)

    def _handle_line(self, raw: str) -> None:
        self.raw_pub.publish(String(data=raw))
        try:
            parsed = parse_frame(raw)
        except ProtocolError as exc:
            self.get_logger().warning(f"Ignored malformed frame {raw!r}: {exc}")
            return

        if parsed.stat is None:
            if parsed.kind in {"EVT", "ERR"}:
                self.event_pub.publish(String(data=raw))
            return

        stat = parsed.stat
        self.encoder_pub.publish(Int32MultiArray(
            data=[stat.left_actual, stat.right_actual]))
        self.target_pub.publish(Int32MultiArray(
            data=[stat.left_target, stat.right_target]))
        self.pwm_pub.publish(Int32MultiArray(
            data=[stat.left_pwm, stat.right_pwm]))

        now_monotonic = time.monotonic()
        dt = 0.1 if self._last_stat_time is None else now_monotonic - self._last_stat_time
        self._last_stat_time = now_monotonic
        if dt <= 0.0 or dt > 1.0:
            dt = 0.1
        state = self.odometry.update(stat.left_actual, stat.right_actual, dt)
        self._publish_odometry(state)

    def _publish_odometry(self, state) -> None:
        stamp = self.get_clock().now().to_msg()
        half_yaw = 0.5 * state.yaw
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = state.linear_velocity
        odom.twist.twist.angular.z = state.angular_velocity
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = state.x
            transform.transform.translation.y = state.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        self._stop_event.set()
        if self.transport.is_open:
            try:
                self.transport.write("V,0,0\r\n")
            except serial.SerialException:
                pass
        self.transport.close()
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32BaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

