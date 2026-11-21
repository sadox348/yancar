# STM32F407 ROS 1 / ROS 2 Differential-Drive Base

A compact migration project that connects an STM32F407 motor controller to
both ROS 1 Noetic and ROS 2 Humble. The firmware handles two PWM motor outputs,
two encoder inputs, a 100 ms wheel-speed loop and a 500 ms communication
watchdog. The ROS bridges translate `/cmd_vel`, decode controller telemetry and
publish wheel feedback. The ROS 2 version additionally integrates differential
odometry and publishes `odom -> base_link` TF.

This repository is a code reconstruction. It does **not** claim new physical
calibration, navigation accuracy, SLAM, Nav2 or road testing. Default wheel and
encoder parameters are placeholders that must be measured before navigation.

## Architecture

```text
                     ROS 1 Noetic / ROS 2 Humble
                                |
                     geometry_msgs/Twist (/cmd_vel)
                                |
                       integer UART command
                         V,<linear>,<angular>
                                |
       +--------------------------------------------------+
       | STM32F407                                       |
       | TIM3 PWM -> motor driver -> left/right motors   |
       | TIM2/TIM4 <- left/right wheel encoders          |
       | 100 ms feed-forward + PID wheel-speed loop      |
       | 500 ms command watchdog                         |
       +--------------------------------------------------+
                                |
                         STAT telemetry
                                |
           wheel delta / target / PWM / odometry / TF
```

## Repository layout

```text
firmware/stm32f407_base/       STM32CubeMX/CMake firmware
ros1_ws/src/stm32_base_bridge ROS 1 Noetic rospy bridge
ros2_ws/src/stm32_base_bridge ROS 2 Humble rclpy bridge and odometry
reference/                    upstream project references only
```

## UART protocol

The firmware is the protocol source of truth. Commands use scaled integers,
not calibrated SI units.

```text
V,<linear_int>,<angular_int>\r\n
V,0,0\r\n
```

The periodic status frame has nine integer fields:

```text
STAT,<enabled>,<left_target>,<left_actual>,<left_pwm>,
     <right_target>,<right_actual>,<right_pwm>,<left_ff>,<right_ff>
```

Other responses include `ACK`, `EVT`, `ERR` and `VER`. Both bridges convert a
`Twist` using configurable scales and clamp the result to `max_command`.

## ROS interfaces

| Direction | Name | Type |
| --- | --- | --- |
| Subscribe | `/cmd_vel` | `geometry_msgs/Twist` |
| Publish | `/wheel_encoder_delta` | `std_msgs/Int32MultiArray` |
| Publish | `/wheel_target_delta` | `std_msgs/Int32MultiArray` |
| Publish | `/wheel_pwm` | `std_msgs/Int32MultiArray` |
| Publish (ROS 2) | `/stm32/status_raw` | `std_msgs/String` |
| Publish (ROS 2) | `/stm32/event` | `std_msgs/String` |
| Publish (ROS 2) | `/stm32/connected` | `std_msgs/Bool` |
| Publish (ROS 2) | `/odom` | `nav_msgs/Odometry` |
| Broadcast (ROS 2) | `odom -> base_link` | TF |

## ROS 1 Noetic

```bash
cd ~/ros_car/ros1_ws
catkin_make
source devel/setup.bash
roslaunch stm32_base_bridge stm32_base_bridge.launch port:=/dev/ttyUSB0
```

The ROS 1 node fixes the legacy float-frame mismatch by sending the integer
format parsed by the current firmware. It also decodes `STAT` into wheel
feedback topics and sends `V,0,0` when `/cmd_vel` becomes stale.

## ROS 2 Humble

```bash
cd ~/ros_car/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch stm32_base_bridge stm32_base_bridge.launch.py port:=/dev/ttyUSB0
```

Configuration is in `config/bridge.yaml`. Important parameters:

- `linear_scale`, `angular_scale`, `max_command`: Twist-to-controller mapping.
- `cmd_vel_timeout`: ROS-side command watchdog; default 0.4 s.
- `wheel_radius`, `wheel_separation`, `encoder_counts_per_rev`: odometry geometry.
- `publish_tf`: enable or disable the `odom -> base_link` transform.

## Odometry model

For each `STAT` sample:

```text
left_distance  = left_delta  * 2*pi*wheel_radius / encoder_counts_per_rev
right_distance = right_delta * 2*pi*wheel_radius / encoder_counts_per_rev
distance       = (left_distance + right_distance) / 2
delta_yaw      = (right_distance - left_distance) / wheel_separation
```

The midpoint heading is used for planar pose integration. Because the geometry
defaults are uncalibrated, `/odom` demonstrates the software data flow rather
than measured positioning accuracy.

## Static tests

The protocol and odometry modules have no ROS dependency:

```bash
cd ros2_ws/src/stm32_base_bridge
python3 -m unittest discover -s test -v
```

Tests cover integer command scaling and clamping, malformed telemetry, straight
motion, in-place rotation, single-wheel arcs and zero encoder motion.

## Resume summary

**STM32F407 and ROS 1/ROS 2 differential-drive mobile base**

- Implemented an STM32F407 base controller with dual PWM motor drive, two
  encoder inputs, wheel-speed feedback control and communication timeout stop.
- Developed ROS 1 and ROS 2 serial bridge nodes that subscribe to `/cmd_vel`,
  encode UART commands and parse STM32 `ACK/STAT/EVT` telemetry into wheel
  feedback topics.
- Added differential-drive odometry and `odom -> base_link` TF publication to
  the ROS 2 migration, with configurable wheel geometry and encoder resolution.

## Attribution

The original AGV repositories are listed under `reference/`. Their navigation
workspace is not bundled into the active implementation because several source
packages do not declare a usable license. The bridge and odometry modules here
are independent minimal implementations around the local firmware protocol.

