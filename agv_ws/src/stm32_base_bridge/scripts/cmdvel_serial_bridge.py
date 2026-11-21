#!/usr/bin/env python3
import rospy
import serial
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class CmdVelSerialBridge:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = rospy.get_param("~baud", 115200)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        rospy.loginfo("Opened serial port %s at %d", self.port, self.baud)

        self.status_pub = rospy.Publisher("/stm32_status", String, queue_size=10)
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=10)

        self.rx_thread = threading.Thread(target=self.read_serial_loop)
        self.rx_thread.daemon = True
        self.rx_thread.start()

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z

        # 发给 STM32 的协议：V,vx,wz\n
        frame = f"V,{vx:.3f},{wz:.3f}\r\n"
        self.ser.write(frame.encode("utf-8"))

        rospy.loginfo_throttle(0.5, "TX -> STM32: %s", frame.strip())

    def read_serial_loop(self):
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
    rospy.init_node("cmdvel_serial_bridge")
    bridge = CmdVelSerialBridge()
    rospy.spin()
