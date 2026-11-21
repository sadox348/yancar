cmake_minimum_required(VERSION 3.0.2)
project(stm32_base_bridge)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy std_msgs)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmdvel_serial_bridge.py
  scripts/protocol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

