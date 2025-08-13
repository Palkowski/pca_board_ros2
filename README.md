# ROS2 PCA9685 node

A set of tools to work with PCA9685 board via I2C to control servo motors,
LEDs or DC motors, designed for use with **Raspberry Pi**, **Jetson**, or
any Linux-based platform.

The ROS2 node is able work with 16 channels in servo mode, or to set duty
cycle directly. PWM frequency is one for the entire board.

This repository includes:
- A **ROS 2 node** for high-level control of servos, LEDs, and DC motors
- A **lightweight C library** for direct I2C access (no ROS dependency)

## Installation

The only dependency (besides gcc, make and ROS2 if you want to use it) is
I2C library for linux, which can be installed on debian based systems with:

```shell
sudo apt install libi2c-dev
```

Build and run `pca_board`:

```shell
cd src/
make
```

## Running

```shell
cd src/
source install/setup.bash
ros2 run pca_board_bringup pca_board
```

There are two launch files: `src/launch/single_board.launch.py`
`src/launch/multi_board.launch.py` which can be edited to configure the node.

For single board use:

```shell
ros2 launch pca_board_bringup single_board.launch.py
```

If there are multiple PCA boards, you can run multiple nodes using ROS
workspaces with this launch file:

```shell
ros2 launch pca_board_bringup multi_board.launch.py
```

## Topics

Node `pca_board` is subscribed to:

- `/servo_angle_set`
- `/servo_angle_add`
- `/multi_servo_set`
- `/pwm_freq_set`
- `/duty_cycle_set`

And provides service:

- `servo_state`

## Interfaces

Interfaces are defined in package `src/pca_board_interfaces`.

### Message interfaces

#### DutyCyclePercent.msg

```
{
    channel: <int32 PCA channel number>,
    duty_cycle: <float64 duty cycle percent>
}
```

#### MultiServoAngleDeg.msg

```
{
    channel_bits: <uint16 bit mask of PCA channels>,
    angle: <float64[16] angle array degrees>
}
```

#### PWMFreqHz.msg

```
{
    freq: <float64 PWM frequency Hz>
}
```

#### ServoAngleDeg.msg

```
{
    channel: <int32 PCA channel index>,
    angle: <float64 angle degrees>
}
```

### Service interfaces

#### ServoState.srv

Request:

```
{
    channel: <int32 PCA channel index>
}
```

Response:

```
{
    channel: <int32 PCA channel index>,
    angle: <float64 angle degrees>
}
```

## ROS Parameters

### Board parameters

These are common PCA board parameters.

- `slave_addr` PCA9685 I2C slave address. Default is `0x40`.
- `i2c` I2C bus number. Default is `1`, which is `/dev/i2c-1`.
- `freq` PWM frequency [Hz]. Default is `50`.

### Default servo parameters

Servo parameters that applied to all PCA channels by default.

- `angle_range` default servo range [deg]
- `pl_min` default min pulse len [ms]
- `pl_max` default max pulse len [ms]
- `zero_pos` default servo zero position [deg]
- `angle_min` default servo min limit [deg]
- `angle_max` default servo max limit [deg]

### Unique servo parameters

When defined, these parameters override the default ones.

- `angle_range_0`
- `angle_range_1`
- `angle_range_2`
- `angle_range_3`
- `angle_range_4`
- `angle_range_5`
- `angle_range_6`
- `angle_range_7`
- `angle_range_8`
- `angle_range_9`
- `angle_range_10`
- `angle_range_11`
- `angle_range_12`
- `angle_range_13`
- `angle_range_14`
- `angle_range_15`

- `pl_max_0`
- `pl_max_1`
- `pl_max_2`
- `pl_max_3`
- `pl_max_4`
- `pl_max_5`
- `pl_max_6`
- `pl_max_7`
- `pl_max_8`
- `pl_max_9`
- `pl_max_10`
- `pl_max_11`
- `pl_max_12`
- `pl_max_13`
- `pl_max_14`
- `pl_max_15`

- `pl_min_0`
- `pl_min_1`
- `pl_min_2`
- `pl_min_3`
- `pl_min_4`
- `pl_min_5`
- `pl_min_6`
- `pl_min_7`
- `pl_min_8`
- `pl_min_9`
- `pl_min_10`
- `pl_min_11`
- `pl_min_12`
- `pl_min_13`
- `pl_min_14`
- `pl_min_15`

- `zero_pos_0`
- `zero_pos_1`
- `zero_pos_2`
- `zero_pos_3`
- `zero_pos_4`
- `zero_pos_5`
- `zero_pos_6`
- `zero_pos_7`
- `zero_pos_8`
- `zero_pos_9`
- `zero_pos_10`
- `zero_pos_11`
- `zero_pos_12`
- `zero_pos_13`
- `zero_pos_14`
- `zero_pos_15`

- `angle_min_0`
- `angle_min_1`
- `angle_min_2`
- `angle_min_3`
- `angle_min_4`
- `angle_min_5`
- `angle_min_6`
- `angle_min_7`
- `angle_min_8`
- `angle_min_9`
- `angle_min_10`
- `angle_min_11`
- `angle_min_12`
- `angle_min_13`
- `angle_min_14`
- `angle_min_15`

- `angle_max_0`
- `angle_max_1`
- `angle_max_2`
- `angle_max_3`
- `angle_max_4`
- `angle_max_5`
- `angle_max_6`
- `angle_max_7`
- `angle_max_8`
- `angle_max_9`
- `angle_max_10`
- `angle_max_11`
- `angle_max_12`
- `angle_max_13`
- `angle_max_14`
- `angle_max_15`
