# ROS 2 DDS and cross-host verification

## Scope

The Linux-side ROS 2 graph uses DDS through the ROS middleware (RMW) layer:

```text
agv_task_controller (rclcpp)
  /cmd_vel: Reliable, Keep Last(1), Volatile
             |
             | DDS / RMW
             v
stm32_base_bridge (rclpy)
  /odom: Reliable, Keep Last(10), Volatile
             |
             | UART (not DDS)
             v
STM32F407
```

The QoS policies are deliberately identical at both ends of each DDS topic.
Control commands retain only the newest sample; odometry retains a short queue
to absorb scheduling jitter.

## Single-host verification

Build and select Fast DDS explicitly on ROS 2 Humble:

```bash
cd ~/ros_car/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
ros2 launch agv_task_controller agv_system.launch.py port:=/dev/ttyUSB0
```

In another sourced terminal, inspect endpoints and QoS:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
ros2 node list
ros2 topic info /cmd_vel --verbose
ros2 topic info /odom --verbose
ros2 topic hz /odom
ros2 topic echo /agv_task/state
```

The controller must remain stopped after launch. Send a valid goal explicitly:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}"
```

Expected results:

- `/cmd_vel` has the C++ publisher and Python bridge subscriber.
- `/odom` has the bridge publisher and C++ controller subscriber.
- Both endpoints report `RELIABLE` and `VOLATILE`; command depth is 1 and
  odometry depth is 10.
- Before a goal is sent, the controller reports `WAITING_FOR_GOAL` and publishes
  only zero velocity commands.
- After a valid goal, states progress through `WAITING_FOR_ODOM`, `ROTATING` or
  `DRIVING`, and finally `GOAL_REACHED`.
- Goals with a missing/non-`odom` frame or non-finite coordinates are rejected.
- Stopping the bridge causes `ODOM_TIMEOUT` and zero `/cmd_vel` output.

## Two-host DDS v3 test

Use two Ubuntu machines on the same LAN. Install the workspace on both, source
the same ROS 2 distribution, and use the same RMW implementation and domain ID.
Do not set `ROS_LOCALHOST_ONLY=1`.

Device A (task controller):

```bash
source /opt/ros/humble/setup.bash
source ~/ros_car/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 launch agv_task_controller task_controller.launch.py
```

Device B (STM32 serial bridge):

```bash
source /opt/ros/humble/setup.bash
source ~/ros_car/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 launch stm32_base_bridge stm32_base_bridge.launch.py port:=/dev/ttyUSB0
```

Verify from either machine:

```bash
ros2 node list
ros2 topic info /cmd_vel --verbose
ros2 topic info /odom --verbose
ros2 topic echo /agv_task/state
```

If discovery fails, first confirm the machines can ping each other, both use
the same `ROS_DOMAIN_ID`, and multicast/UDP traffic is allowed by the network
and host firewalls. Guest Wi-Fi and AP/client isolation commonly block DDS
discovery. A direct Ethernet connection or normal private LAN is preferable for
the demo.

## QoS incompatibility demonstration

For an interview/demo, temporarily change one endpoint to an incompatible
policy and observe ROS 2's warning and missing data, then restore the committed
configuration. The production source should remain Reliable/Volatile on both
sides. This demonstrates that discovery alone does not guarantee compatible
data delivery.
