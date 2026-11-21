#!/usr/bin/env python3
import rospy
import serial
import threading
from std_msgs.msg import String
from communicate_with_stm32.msg import MotorCmd


class MotorCmdSerialAdapter:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = rospy.get_param("~baud", 115200)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        rospy.loginfo("Opened serial port %s at %d", self.port, self.baud)

        self.status_pub = rospy.Publisher("/stm32_status", String, queue_size=10)

        self.sub = rospy.Subscriber(
            "/stm32TopicCtrl",
            MotorCmd,
            self.motor_cmd_callback,
            queue_size=10
        )

        self.rx_thread = threading.Thread(target=self.read_loop)
        self.rx_thread.daemon = True
        self.rx_thread.start()

    def motor_cmd_callback(self, msg):
        rospy.loginfo("Raw MotorCmd: cmd=%d data=%s", msg.cmd, list(msg.data))

        if msg.cmd == 6:
            # 根据你前面 echo 的结果：
            # /cmd_vel linear.x=0.2  -> data[0]=200
            # /cmd_vel angular.z=0.5 -> data[2]=5000
            # STM32 期望 V,200,500，代表 vx=0.200, wz=0.500
            vx_i = int(msg.data[0] / 10)
            wz_i = int(msg.data[2] / 10)

            frame = f"V,{vx_i},{wz_i}\r\n"

        elif msg.cmd == 2:
            frame = "STOP\r\n"

        else:
            rospy.logwarn("Unsupported cmd: %d", msg.cmd)
            return

        self.ser.write(frame.encode("utf-8"))
        self.ser.flush()
        rospy.loginfo("TX -> STM32: %s", frame.strip())

    def read_loop(self):
        rospy.loginfo("Serial read thread started")

        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    rospy.loginfo("RX <- STM32: %s", line)
                    self.status_pub.publish(line)

            except Exception as e:
                rospy.logerr("Serial read error: %s", str(e))
                rospy.sleep(0.5)


if __name__ == "__main__":
    rospy.init_node("motorcmd_serial_adapter")
    MotorCmdSerialAdapter()
    rospy.spin()

