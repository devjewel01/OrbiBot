# CLAUDE.md

This file provides guidance to Claude Code when working with OrbiBot, a mecanum-wheeled mobile robot.

## Current Development Status

**✅ COMPLETED**: orbibot_description, orbibot_msgs, orbibot_hardware, orbibot_control, orbibot_webui, orbibot_teleop, orbibot_localization, orbibot_sensors, orbibot_slam, orbibot_navigation, orbibot_bringup
**🔄 ACTIVE**: Full autonomous navigation system with SLAM capabilities

## Repository Overview

ROS 2 Jazzy workspace for OrbiBot running on Raspberry Pi 5 with Ubuntu 24.04.

### System Architecture
- **Robot Platform**: Raspberry Pi 5 with Ubuntu 24.04 Server + ROS 2 Jazzy Base
- **Development Machine**: Ubuntu 24.04 Desktop + ROS 2 Jazzy Desktop
- **Network**: ROS 2 Domain ID = 42
- **Visualization Strategy**: All RViz and visualization run on development machine to reduce robot load
- **Launch Philosophy**: Individual launch files for each package to enable easy testing and debugging
- **Design Goals**: Simple, clean launch files without simulation time for real hardware operation

### Hardware Specifications
- **Platform**: Raspberry Pi 5 (8GB RAM), Ubuntu 24.04 Server, ROS 2 Jazzy Base
- **Motors**: 4x Yahboom JGB37-520 (333 RPM, 1:30 gear ratio) with encoders
- **Drive**: 100mm mecanum wheels, holonomic drive
- **Controller**: Yahboom ROS Expansion Board V3.0 with integrated IMU sensor and motor driver
- **Sensors**: RPLIDAR A1 (`/dev/lidar`), Intel RealSense D435, MPU9250 IMU (onboard)
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
colcon build --packages-select orbibot_localization
colcon build --packages-select orbibot_sensors
colcon build --packages-select orbibot_slam
colcon build --packages-select orbibot_navigation
colcon build --packages-select orbibot_bringup
```

## Current Package Status

### ✅ orbibot_description (COMPLETE)
- **Location**: `src/orbibot_description/`
- **Type**: ament_cmake package
- **Contents**: Complete robot URDF with mecanum wheels and sensor mounts
- **Key Files**:
  - `urdf/orbibot.urdf.xacro` - Main robot definition
  - `launch/description.launch.py` - Robot state publisher only
  - `launch/orbibot_remote_gui.launch.py` - Remote RViz visualization
  - `meshes/` - 3D models for wheels and sensors

**Test Commands:**
```bash
ros2 launch orbibot_description description.launch.py          # Robot state publisher only
ros2 launch orbibot_description orbibot_remote_gui.launch.py   # Remote RViz visualization
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
  - `ResetOdometry.srv` - Reset odometry values

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
- **Contents**: Multiple teleoperation modes
- **Key Files**:
  - `keyboard_teleop.py` - Advanced keyboard teleoperation
  - `ps_controller_teleop.py` - PlayStation controller support
  - `launch/keyboard_teleop.launch.py` - Keyboard teleop launcher
  - `launch/ps_controller_teleop.launch.py` - Controller teleop launcher
- **Features**:
  - Hardware-integrated teleop with motor enable/disable
  - Press-and-hold keyboard control
  - PlayStation controller support
  - SSH/remote operation compatibility
  - Emergency stop and safety features
  - Real-time status display and speed adjustment

### ✅ orbibot_sensors (COMPLETE)
- **Location**: `src/orbibot_sensors/`
- **Type**: ament_python package
- **Contents**: Sensor integration and drivers
- **Key Files**:
  - `launch/sensors.launch.py` - All sensors
  - `launch/lidar.launch.py` - RPLIDAR A1 only
  - `launch/camera.launch.py` - RealSense D435 only
  - `config/rplidar_params.yaml` - LIDAR configuration
  - `config/realsense_camera_params.yaml` - Camera configuration
- **Features**:
  - RPLIDAR A1 integration
  - Intel RealSense D435 camera support
  - Sensor parameter configuration
  - Individual sensor launch files

### ✅ orbibot_localization (COMPLETE)
- **Location**: `src/orbibot_localization/`
- **Type**: ament_python package
- **Contents**: Enhanced localization with sensor fusion
- **Key Files**:
  - `launch/localization.launch.py` - Localization stack
  - `config/localization_params.yaml` - EKF parameters
- **Features**:
  - Extended Kalman Filter (EKF) for sensor fusion
  - Combines odometry, IMU, and visual odometry
  - Improved pose estimation accuracy
  - Integration with robot_localization package

### ✅ orbibot_slam (COMPLETE)
- **Location**: `src/orbibot_slam/`
- **Type**: ament_python package
- **Contents**: SLAM implementation using slam_toolbox
- **Key Files**:
  - `launch/slam.launch.py` - SLAM launcher
  - `config/slam_params.yaml` - SLAM configuration
  - `rviz/slam.rviz` - SLAM RViz configuration
- **Features**:
  - Real-time SLAM mapping
  - Loop closure detection
  - Map saving and loading
  - Integration with slam_toolbox

### ✅ orbibot_navigation (COMPLETE)
- **Location**: `src/orbibot_navigation/`
- **Type**: ament_cmake package
- **Contents**: Nav2 stack integration only
- **Key Files**:
  - `launch/navigation.launch.py` - Nav2 stack launcher
  - `config/nav2_params.yaml` - Navigation parameters
  - `maps/` - Saved maps directory
- **Features**:
  - Nav2 stack integration
  - SLAM and localization modes
  - Path planning and obstacle avoidance
  - Supports multiple navigation modes

### ✅ orbibot_bringup (COMPLETE)
- **Location**: `src/orbibot_bringup/`
- **Type**: ament_python package
- **Contents**: System launch files and complete robot bringup
- **Key Files**:
  - `launch/orbibot_system.launch.py` - Complete system with navigation
  - `launch/orbibot_basic.launch.py` - Basic system (hardware + control + teleop)
  - `launch/autonomous_navigation.launch.py` - Autonomous navigation
- **Features**:
  - Complete system integration
  - Modular component launching
  - Autonomous navigation capabilities
  - Configurable system startup

## Development Workflow

### Current Status
All core packages are complete and functional:
- ✅ **Robot Description** - Complete URDF with sensor mounts
- ✅ **Hardware Interface** - ROSMaster integration working
- ✅ **Control System** - Odometry and system coordination
- ✅ **Sensor Integration** - LIDAR and camera support
- ✅ **Localization** - EKF sensor fusion
- ✅ **SLAM** - Real-time mapping capabilities
- ✅ **Navigation** - Autonomous navigation with Nav2
- ✅ **Teleoperation** - Multiple control modes
- ✅ **Web Interface** - Remote monitoring

### Testing Strategy
- **Individual Package Testing**: Each package has separate launch files for isolated testing
- **Component Verification**: Test each component individually before system integration
- **Remote Development**: Development machine handles all visualization to reduce robot load
- **Hardware Motor Control**: Always enable motors through service calls for safety

### Future Development Priorities
1. **orbibot_perception** - Object detection and environment understanding
2. **orbibot_missions** - Task execution and mission planning
3. **Multi-robot coordination** - Fleet management capabilities
4. **Advanced behaviors** - Person following, patrol modes

### Safety Requirements
- Always enable/disable motors through service calls using hardware.launch.py
- Implement command timeout (2.0 seconds)
- Monitor battery voltage (warn <11.0V)
- Emergency stop capability
- Test with robot on blocks initially
- Individual component testing before full system integration

### Hardware Setup Prerequisites
```bash
# Install ROSMaster_Lib on Raspberry Pi 5
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

# Set ROS 2 Domain ID on both machines
export ROS_DOMAIN_ID=42
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
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
~/orbibot_ws/src/orbibot_sensors/      # Sensor integration (LIDAR, camera)
~/orbibot_ws/src/orbibot_localization/ # Enhanced localization with EKF
~/orbibot_ws/src/orbibot_slam/         # SLAM implementation
~/orbibot_ws/src/orbibot_navigation/   # Nav2 stack integration
~/orbibot_ws/src/orbibot_bringup/      # System launch files and complete robot bringup
~/orbibot_ws/src/orbibot_teleop/       # Multiple teleoperation modes
~/orbibot_ws/src/orbibot_webui/        # Web monitoring interface

# Future packages (to be created)
~/orbibot_ws/src/orbibot_perception/   # Object detection and recognition
~/orbibot_ws/src/orbibot_missions/     # Task execution and mission planning
```

## Common Commands

```bash
# Environment setup (on both machines)
source /opt/ros/jazzy/setup.bash
source ~/orbibot_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# Individual Component Launch Commands (for testing and debugging)
ros2 launch orbibot_description description.launch.py          # Robot state publisher only
ros2 launch orbibot_description orbibot_remote_gui.launch.py   # Remote RViz visualization (run on dev machine)
ros2 launch orbibot_hardware hardware.launch.py               # Hardware interface with motor enable
ros2 launch orbibot_control control.launch.py                 # Control manager with odometry
ros2 launch orbibot_sensors sensors.launch.py                 # All sensors
ros2 launch orbibot_sensors lidar.launch.py                   # RPLIDAR A1 only
ros2 launch orbibot_sensors camera.launch.py                  # RealSense D435 only
ros2 launch orbibot_localization localization.launch.py       # Enhanced localization (EKF)
ros2 launch orbibot_slam slam.launch.py                       # SLAM mapping
ros2 launch orbibot_teleop keyboard_teleop.launch.py          # Keyboard teleoperation
ros2 launch orbibot_teleop ps_controller_teleop.launch.py     # PlayStation controller
ros2 launch orbibot_webui webui.launch.py                     # Web monitoring interface

# Complete System Integration
ros2 launch orbibot_bringup orbibot_system.launch.py          # Complete system with navigation
ros2 launch orbibot_bringup orbibot_basic.launch.py           # Basic system (hardware + control + teleop)
ros2 launch orbibot_bringup autonomous_navigation.launch.py   # Autonomous navigation
ros2 launch orbibot_navigation navigation.launch.py           # Nav2 stack only

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

## System Operation Modes

### 1. Manual Teleoperation
```bash
# Basic system (hardware + control + teleop)
ros2 launch orbibot_bringup orbibot_basic.launch.py

# Individual components for debugging
ros2 launch orbibot_hardware hardware.launch.py
ros2 launch orbibot_control control.launch.py
ros2 launch orbibot_teleop keyboard_teleop.launch.py
```

### 2. SLAM Mapping
```bash
# Complete SLAM system
ros2 launch orbibot_bringup orbibot_system.launch.py nav_mode:=slam

# Individual components for debugging
ros2 launch orbibot_hardware hardware.launch.py
ros2 launch orbibot_control control.launch.py
ros2 launch orbibot_sensors lidar.launch.py
ros2 launch orbibot_slam slam.launch.py
ros2 launch orbibot_navigation navigation.launch.py mode:=slam
```

### 3. Autonomous Navigation
```bash
# Complete navigation system
ros2 launch orbibot_bringup orbibot_system.launch.py nav_mode:=localization

# Autonomous navigation with existing map
ros2 launch orbibot_bringup autonomous_navigation.launch.py
```

### 4. Development and Testing
```bash
# Remote visualization (run on development machine)
ros2 launch orbibot_description orbibot_remote_gui.launch.py

# Monitor system status
ros2 launch orbibot_webui webui.launch.py
```

---

**Current Status**: Complete autonomous mobile robot system with individual component testing capability  
**Hardware Dependencies**: ROSMaster_Lib installed, proper USB permissions configured, ROS_DOMAIN_ID=42  
**Development Setup**: Raspberry Pi 5 (robot) + Ubuntu Desktop (visualization)  
**Capabilities**: Teleoperation, SLAM mapping, autonomous navigation, web monitoring  
**Testing Philosophy**: Each package tested individually before system integration  
**Safety Note**: Always test new autonomous features in safe, controlled environment