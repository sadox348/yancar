# Reference projects

The original ROS 1 AGV workspace is retained locally only as a reference and is
not copied into the active implementation because several packages declare an
unspecified `TODO` license.

Upstream projects referenced during the migration:

- Project overview: https://github.com/dlrdaile/agv_project
- STM32 firmware: https://github.com/dlrdaile/agv_stm32
- ROS navigation workspace: https://github.com/dlrdaile/agv_ros

The active ROS 1 and ROS 2 bridge packages in this repository are small,
independent implementations built around the UART protocol implemented by the
local STM32 firmware.

