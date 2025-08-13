# ROS 2 PCA9685 Node

A set of tools for controlling the **PCA9685 PWM/Servo Driver** via I2C,
designed for use with **Raspberry Pi**, **Jetson**, or any Linux-based
platform.

This repository includes:
- A **ROS 2 node** for high-level control of servos, LEDs, and DC motors
- A **lightweight C library** for direct I2C access (no ROS dependency)
- Support for **single or multiple PCA9685 boards**
- Configurable per-channel servo parameters
- Tested on **ROS 2 Humble** and **ROS 2 Jazzy**

## Directory Structure

```
├── README.md
├── i2c_pca_utils/            # Standalone C library + examples
│   ├── i2c_pca_utils.h
│   ├── i2c_pca_utils.c
│   ├── set_servo_angle.c
│   └── Makefile
└── src/                      # ROS 2 packages
    ├── pca_board_bringup/    # Node and launch files
    ├── pca_board_interfaces/ # Custom message definitions
    └── Makefile
```

## Features

- Control up to **16 servo channels** individually
- Set **duty cycle** for LEDs or DC motors
- Supports **multiple PCA9685 boards**

## Installation

### Prerequisites

Install the Linux I2C library:

```bash
sudo apt install libi2c-dev
```

Ensure your user has access to I2C:
```bash
sudo usermod -aG i2c $USER
```

Reboot or log out/in to apply.

### Build the ROS 2 Node

This builds the `pca_board` executable and message interfaces:

```bash
cd src
make
```

## Running the Node

### Source Workspace
```bash
source install/setup.bash
```

### Run Directly
```bash
ros2 run pca_board_bringup pca_board
```

### Using Launch Files

Edit launch files to customize I2C address, bus, or servo parameters.

#### Single Board
```bash
ros2 launch pca_board_bringup single_board.launch.py
```

#### Multiple Boards
Use `multi_board.launch.py` to run multiple instances with different parameters:
```bash
ros2 launch pca_board_bringup multi_board.launch.py
```

## Topics

The `pca_board` node subscribes to:

| Topic | Message Type | Description |
|------|--------------|-------------|
| `/servo_angle_set` | `ServoAngleDeg` | Set angle for a single servo |
| `/servo_angle_add` | `ServoAngleDeg` | Adjust servo angle by delta |
| `/multi_servo_set` | `MultiServoAngleDeg` | Set angles for multiple servos |
| `/duty_cycle_set` | `DutyCyclePercent` | Set PWM duty cycle (e.g., for LEDs) |
| `/pwm_freq_set` | `PWMFreqHz` | Set board PWM frequency |

### Services

| Service | Type | Description |
|-------|------|-------------|
| `/servo_state` | `ServoState` | Get current angle of a servo channel |


## Message Interfaces

Defined in `pca_board_interfaces`.

### DutyCyclePercent.msg
```
int32 channel
float64 duty_cycle  # 0.0 to 100.0 (%)
```
Set PWM duty cycle for a channel.

### MultiServoAngleDeg.msg
```
uint16 channel_bits     # Bitmask of channels to update
float64[16] angle       # Angle in degrees (ignored if bit not set)
```
Set angles for multiple servos in one message.

### PWMFreqHz.msg
```
float64 freq  # PWM frequency in Hz
```

### ServoAngleDeg.msg
```
int32 channel
float64 angle  # Angle in degrees
```

### ServoState.srv
**Request:**
```
int32 channel
```

**Response:**
```
int32 channel
float64 angle  # Current angle in degrees
```

## ROS 2 Parameters

### Board-Level Parameters

| Parameter | Type | Default | Description |
|---------|------|--------|-------------|
| `slave_addr` | `int` | `0x40` | I2C address of PCA9685 |
| `i2c` | `int` | `1` | I2C bus number (`/dev/i2c-1`) |
| `freq` | `double` | `50.0` | PWM frequency in Hz |

### Default Servo Parameters

Apply to all channels unless overridden:

| Parameter | Type | Description |
|---------|------|-------------|
| `angle_range` | `double` | Servo angular range (e.g., 180°) |
| `pl_min` | `double` | Minimum pulse length [ms] |
| `pl_max` | `double` | Maximum pulse length [ms] |
| `zero_pos` | `double` | Angle at 1.5ms pulse [deg] |
| `angle_min` | `double` | Mechanical min angle limit |
| `angle_max` | `double` | Mechanical max angle limit |

### Per-Channel Overrides

Each parameter can be overridden per channel (0–15):

- `angle_range_0` to `angle_range_15`
- `pl_min_0` to `pl_min_15`
- `pl_max_0` to `pl_max_15`
- `zero_pos_0` to `zero_pos_15`
- `angle_min_0` to `angle_min_15`
- `angle_max_0` to `angle_max_15`

## Standalone C Library (No ROS)

For non-ROS applications, use the `i2c_pca_utils` library:

### Build
```bash
cd i2c_pca_utils
make
```

### Example
```bash
./set_servo_angle 0 90  # Set channel 0 to 90°
```

## Example Usage

Set servo on channel 3 to 45°:
```bash
ros2 topic pub /servo_angle_set pca_board_interfaces/msg/ServoAngleDeg \
  "{channel: 3, angle: 45.0}" --once
```

Query current position:
```bash
ros2 service call /servo_state pca_board_interfaces/srv/ServoState \
  "{channel: 3}"
```

## Tips

- Use `i2cdetect -y 1` to verify PCA9685 is visible on the bus
- For multiple boards, set unique I2C slave addresses

## License

MIT License – see `LICENSE` for details.
