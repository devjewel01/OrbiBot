# CLAUDE.md

This file provides guidance to Claude Code when working with OrbiBot, a mecanum-wheeled mobile robot.

## Current Development Status

**✅ COMPLETED**: orbibot_description
**🔄 NEXT**: orbibot_msgs → orbibot_hardware → orbibot_control

## Repository Overview

ROS 2 Jazzy workspace for OrbiBot running on Raspberry Pi 5 with Ubuntu 24.04.

### Hardware Specifications
- **Platform**: Raspberry Pi 5 (8GB RAM), Ubuntu 24.04, ROS 2 Jazzy
- **Motors**: 4x Yahboom JGB37-520 (333 RPM, 1:30 gear ratio)
- **Drive**: 100mm mecanum wheels, holonomic drive
- **Controller**: Yahboom ROS Expansion Board V3.0 (USB-Serial `/dev/motordriver`)
- **Sensors**: RPLIDAR A1 (`/dev/lidar`), Intel RealSense D435, MPU9250 IMU
- **Robot**: 350×250×100mm, 8.5kg, wheel separation 180mm×300mm

## Build Commands

```bash
# Setup environment
source /opt/ros/jazzy/setup.bash
cd ~/orbibot_ws

# Build and source
colcon build
source install/setup.bash

# Build specific packages
colcon build --packages-select orbibot_description
colcon build --packages-select orbibot_msgs        # When created
colcon build --packages-select orbibot_hardware    # When created
```

## Current Package Status

### ✅ orbibot_description (COMPLETE)
- **Location**: `src/orbibot_description/`
- **Type**: ament_cmake package
- **Contents**: Complete robot URDF with mecanum wheels and sensor mounts
- **Key Files**:
  - `urdf/orbibot.urdf.xacro` - Main robot definition
  - `launch/display.launch.py` - RViz visualization
  - `meshes/` - 3D models for wheels and sensors

**Test Commands:**
```bash
ros2 launch orbibot_description display.launch.py
```

### 🔄 orbibot_msgs (NEXT PRIORITY)
**Needed Messages:**
```
WheelSpeeds.msg:
  float64 front_left
  float64 front_right  
  float64 back_left
  float64 back_right

MotorFeedback.msg:
  std_msgs/Header header
  int32[] encoder_counts
  float64[] positions  
  float64[] velocities
  bool[] motor_enabled

SystemStatus.msg:
  std_msgs/Header header
  float32 battery_voltage
  bool motors_enabled
  bool hardware_ok
```

**Needed Services:**
```
SetMotorEnable.srv:
  bool enable
  ---
  bool success
  string message
```

### 📋 orbibot_hardware (PLANNED)
**Requirements:**
- ROSMaster_Lib integration for Yahboom board
- Motor control (4 motors, RPM commands)
- Encoder feedback (1320 counts/revolution)
- IMU publishing (accelerometer, gyroscope, magnetometer)
- Safety monitoring (battery, emergency stop, timeouts)

**Key APIs to implement:**
```python
# ROSMaster_Lib usage
import Rosmaster_Lib
board = Rosmaster_Lib.Rosmaster()

# Motor control
board.set_motor(motor_id, rpm)        # motor_id: 1-4, rpm: -333 to +333
board.set_motor_enable(True/False)    # Enable/disable all motors

# Sensor reading  
board.get_motor_encoder(motor_id)     # Get encoder count
board.get_accelerometer_data()        # [ax, ay, az] m/s²
board.get_gyroscope_data()           # [wx, wy, wz] rad/s
board.get_battery_voltage()          # Battery voltage
```

### 📋 orbibot_control (PLANNED)
**Mecanum Drive Kinematics:**
```python
# Physical parameters
WHEEL_RADIUS = 0.05      # 50mm radius
WHEEL_SEP_X = 0.18       # 180mm front-back
WHEEL_SEP_Y = 0.30       # 300mm left-right

# Inverse kinematics (cmd_vel → wheel speeds)
def inverse_kinematics(vx, vy, wz):
    wheel_base = (WHEEL_SEP_X + WHEEL_SEP_Y) / 2.0
    front_left  = (vx - vy - wz * wheel_base) / WHEEL_RADIUS
    front_right = (vx + vy + wz * wheel_base) / WHEEL_RADIUS
    back_left   = (vx + vy - wz * wheel_base) / WHEEL_RADIUS
    back_right  = (vx - vy + wz * wheel_base) / WHEEL_RADIUS
    return [front_left, front_right, back_left, back_right]
```

## Development Workflow

### Package Creation Order
1. **orbibot_msgs** - Message definitions
2. **orbibot_hardware** - Hardware interface  
3. **orbibot_control** - Motion control
4. **orbibot_localization** - Odometry and pose
5. **orbibot_navigation** - SLAM and autonomous navigation

### Safety Requirements
- Always enable/disable motors through service calls
- Implement command timeout (2.0 seconds)
- Monitor battery voltage (warn <11.0V)
- Emergency stop capability
- Test with robot on blocks initially

### Hardware Setup Prerequisites
```bash
# Install ROSMaster_Lib
# Download from: http://www.yahboom.net/study/ros-driver-board
sudo python3 setup.py install

# Setup device permissions (create udev rules)
sudo tee /etc/udev/rules.d/99-orbibot.rules > /dev/null <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="motordriver", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
EOF
sudo udevadm control --reload-rules

# Verify connections
ls -la /dev/motordriver /dev/lidar
```

## Key File Locations

```bash
# Workspace
~/orbibot_ws/

# Current packages
~/orbibot_ws/src/orbibot_description/urdf/orbibot.urdf.xacro  # Robot definition

# Future packages (to be created)
~/orbibot_ws/src/orbibot_msgs/         # Custom messages
~/orbibot_ws/src/orbibot_hardware/     # Hardware interface
~/orbibot_ws/src/orbibot_control/      # Motion control
```

## Common Commands

```bash
# Environment setup
source /opt/ros/jazzy/setup.bash
source ~/orbibot_ws/install/setup.bash

# Visualization
ros2 launch orbibot_description display.launch.py

# Development monitoring
ros2 topic list
ros2 service list
ros2 node list

# Future hardware testing (when implemented)
ros2 run orbibot_hardware hardware_test
ros2 service call /orbibot/set_motor_enable orbibot_msgs/srv/SetMotorEnable "{enable: true}"
ros2 topic echo /orbibot/system_status
```

## Next Development Steps

1. **Create orbibot_msgs package** with message definitions
2. **Implement orbibot_hardware** with ROSMaster_Lib integration
3. **Test hardware interface** with motor control and sensor reading
4. **Develop orbibot_control** for mecanum drive kinematics
5. **Add safety features** and emergency handling

---

**Current Status**: Robot description complete, ready for hardware interface development  
**Hardware Dependencies**: ROSMaster_Lib, proper USB permissions  
**Safety Note**: Always test motor commands with robot elevated or in safe area