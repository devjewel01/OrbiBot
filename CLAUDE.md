# CLAUDE.md

This file provides guidance to Claude Code when working with OrbiBot, a mecanum-wheeled mobile robot.

## Current Development Status

**✅ COMPLETED**: orbibot_description, orbibot_msgs, orbibot_hardware, orbibot_control, orbibot_webui, orbibot_teleop
**🔄 NEXT**: orbibot_localization → orbibot_navigation (SLAM & Autonomous Navigation)

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
colcon build --packages-select orbibot_msgs
colcon build --packages-select orbibot_hardware
colcon build --packages-select orbibot_control
colcon build --packages-select orbibot_webui
colcon build --packages-select orbibot_teleop
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

### ✅ orbibot_msgs (COMPLETE)
- **Location**: `src/orbibot_msgs/`
- **Type**: ament_cmake package
- **Contents**: Custom message and service definitions
- **Key Messages**:
  - `WheelSpeeds.msg` - Individual wheel speed commands
  - `MotorFeedback.msg` - Encoder positions, velocities, status
  - `SystemStatus.msg` - Battery voltage, hardware status
  - `SafetyStatus.msg` - Safety monitoring data
- **Key Services**:
  - `SetMotorEnable.srv` - Enable/disable motors
  - `CalibrateIMU.srv` - IMU calibration service

### ✅ orbibot_hardware (COMPLETE)
- **Location**: `src/orbibot_hardware/`
- **Type**: ament_python package
- **Architecture**: Custom hardware interface (not ros2_control)
- **Contents**: 
  - `hardware_node.py` - Main hardware interface node
  - `rosmaster_interface.py` - ROSMaster_Lib wrapper
- **Features**:
  - Direct `/cmd_vel` processing with ROSMaster mecanum kinematics
  - Real-time encoder feedback and joint state publishing
  - IMU attitude data (roll, pitch, yaw)
  - Battery monitoring and safety features
  - Command timeout protection and emergency stop

**Key Topics:**
```bash
# Published
/joint_states              # Joint positions/velocities for RViz
/orbibot/motor_feedback    # Encoder counts and motor status
/imu/data                  # IMU orientation data
/orbibot/system_status     # Battery voltage, motor enable status
/orbibot/safety_status     # Safety monitoring data

# Subscribed  
/cmd_vel                   # Velocity commands (primary control)
/orbibot/wheel_speeds      # Direct wheel speed commands
```

### ✅ orbibot_control (COMPLETE)
- **Location**: `src/orbibot_control/`
- **Type**: ament_python package
- **Architecture**: Separate control layer for high-level coordination
- **Contents**:
  - `control_manager.py` - Odometry calculation and system monitoring
- **Features**:
  - Forward kinematics for odometry calculation
  - TF broadcasting (odom → base_link)
  - System health monitoring and coordination

**Note**: Inverse kinematics handled by ROSMaster hardware (`set_car_motion()`)

### ✅ orbibot_webui (COMPLETE)
- **Location**: `src/orbibot_webui/`
- **Type**: ament_python package
- **Contents**: Web-based monitoring interface
- **Features**:
  - Real-time robot status dashboard
  - Battery and system monitoring
  - Web-based remote monitoring

### ✅ orbibot_teleop (COMPLETE)
- **Location**: `src/orbibot_teleop/`
- **Type**: ament_python package
- **Contents**: Advanced keyboard teleoperation
- **Features**:
  - Hardware-integrated teleop with motor enable/disable
  - Press-and-hold keyboard control
  - SSH/remote operation compatibility
  - Emergency stop and safety features
  - Real-time status display and speed adjustment

## Development Workflow

### Next Development Priority
1. **orbibot_localization** - Enhanced odometry, pose estimation, sensor fusion
2. **orbibot_navigation** - SLAM mapping and autonomous navigation
3. **orbibot_perception** - Object detection and environment understanding (optional)
4. **orbibot_missions** - Task execution and mission planning (optional)

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

# Completed packages
~/orbibot_ws/src/orbibot_description/  # Robot URDF and visualization
~/orbibot_ws/src/orbibot_msgs/         # Custom messages and services
~/orbibot_ws/src/orbibot_hardware/     # Hardware interface and control
~/orbibot_ws/src/orbibot_control/      # Odometry and system coordination
~/orbibot_ws/src/orbibot_webui/        # Web monitoring interface
~/orbibot_ws/src/orbibot_teleop/       # Advanced keyboard teleoperation

# Future packages (to be created)
~/orbibot_ws/src/orbibot_localization/ # Enhanced localization
~/orbibot_ws/src/orbibot_navigation/   # SLAM and autonomous navigation
```

## Common Commands

```bash
# Environment setup
source /opt/ros/jazzy/setup.bash
source ~/orbibot_ws/install/setup.bash

# System Launch Commands
ros2 launch orbibot_description display.launch.py     # RViz visualization
ros2 launch orbibot_description robot_state.launch.py # Robot state publisher only
ros2 launch orbibot_hardware hardware.launch.py      # Hardware interface
ros2 launch orbibot_control control.launch.py        # Control manager
ros2 launch orbibot_teleop teleop.launch.py          # Advanced teleop
ros2 launch orbibot_webui webui.launch.py           # Web monitoring

# System Integration
ros2 launch orbibot_control orbibot_system.launch.py # Complete system

# Development monitoring
ros2 topic list
ros2 service list
ros2 node list

# Hardware testing and control
ros2 service call /orbibot/set_motor_enable orbibot_msgs/srv/SetMotorEnable "{enable: true}"
ros2 topic echo /orbibot/system_status
ros2 topic echo /joint_states
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

# Teleoperation options
ros2 run teleop_twist_keyboard teleop_twist_keyboard  # Basic teleop
ros2 run orbibot_teleop keyboard_teleop              # Advanced OrbiBot teleop
```

## Next Development Steps

### Priority 1: Enhanced Localization (orbibot_localization)
1. **Sensor fusion** - Combine odometry, IMU, and visual odometry
2. **Extended Kalman Filter** - Improve pose estimation accuracy
3. **Landmark detection** - Use camera for visual localization
4. **Loop closure** - Handle drift in long-term operation

### Priority 2: Autonomous Navigation (orbibot_navigation)
1. **SLAM implementation** - Simultaneous Localization and Mapping
   - Consider: `slam_toolbox`, `cartographer`, or `rtabmap`
2. **Path planning** - Global and local path planning algorithms
3. **Obstacle avoidance** - Dynamic obstacle detection and avoidance
4. **Navigation stack integration** - ROS 2 Nav2 stack integration

### Priority 3: Enhanced Capabilities (Optional)
1. **orbibot_perception** - Object detection, person following
2. **orbibot_missions** - Task planning and execution
3. **Multi-robot coordination** - Fleet management capabilities

---

**Current Status**: All core robot packages complete and tested  
**Hardware Dependencies**: ROSMaster_Lib installed, proper USB permissions configured  
**Next Focus**: Autonomous navigation and SLAM capabilities  
**Safety Note**: Always test new autonomous features in safe, controlled environment