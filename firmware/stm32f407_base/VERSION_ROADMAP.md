# STM32F407 AGV Base Firmware Roadmap

## Current version: v0.9

The firmware controls a two-wheel differential-drive base over an ASCII UART
protocol. It is independent from ROS so the same controller can be used by both
ROS 1 and ROS 2 bridge nodes.

### Implemented

- STM32F407 project generated with STM32CubeMX.
- USART1 at 115200 baud with line-oriented command parsing.
- TIM3 CH1/CH2 PWM outputs for two brushed DC motors.
- TIM2/TIM4 encoder inputs for left and right wheel feedback.
- Signed forward, reverse and stop control for both motors.
- Differential command mixing from `V,<linear>,<angular>`.
- 100 ms dual-wheel speed loop with feed-forward and PID correction.
- Minimum start PWM, output saturation and direction compensation.
- 500 ms base-command timeout with immediate motor stop.
- `ACK`, `STAT`, `EVT`, `ERR` and `VER` response frames.

### UART protocol

Command:

```text
V,<linear_int>,<angular_int>\r\n
```

Periodic status:

```text
STAT,<enabled>,<left_target>,<left_actual>,<left_pwm>,
     <right_target>,<right_actual>,<right_pwm>,<left_ff>,<right_ff>
```

The command values are scaled controller inputs rather than calibrated SI
units. Wheel radius, separation, encoder counts per revolution and command
scales must be calibrated before using odometry for navigation.

### Deferred

- Physical wheel and encoder calibration.
- Hardware-in-the-loop regression tests.
- Accurate SI-unit velocity control.
- Navigation-grade odometry validation.

