# OrbiBot Autonomous Operation Guide

## 🤖 Overview

This guide provides step-by-step instructions for operating your OrbiBot in autonomous mode. Your robot is equipped with complete autonomous navigation capabilities including SLAM mapping, path planning, obstacle avoidance, and safety monitoring.

## 🏗️ System Architecture

### Hardware Components
- **Platform**: Raspberry Pi 5 (8GB RAM), Ubuntu 24.04, ROS 2 Jazzy
- **Drive System**: 4x Mecanum wheels with Yahboom JGB37-520 motors
- **Sensors**: 
  - RPLIDAR A1 (2D mapping)
  - Intel RealSense D435 (depth/visual odometry)
  - MPU9250 IMU (orientation, acceleration, gyroscope)
  - Encoder feedback (1320 counts/rev)
- **Controller**: Yahboom ROS Expansion Board V3.0

### Software Stack
- **orbibot_hardware**: Complete sensor integration and motor control
- **orbibot_localization**: Enhanced sensor fusion (wheel odometry + IMU + visual)
- **orbibot_navigation**: SLAM and autonomous navigation
- **orbibot_control**: High-level coordination and monitoring
- **orbibot_teleop**: Manual control and emergency override

## 🚀 Quick Start - Autonomous Navigation

### Prerequisites
```bash
# Ensure all packages are built
cd ~/orbibot_ws
colcon build
source install/setup.bash

# Verify hardware connections
ls -la /dev/motordriver /dev/lidar  # Should exist
```

### Method 1: One-Command Autonomous Startup
```bash
# Launch complete autonomous system with existing map
ros2 launch orbibot_navigation orbibot_navigation.launch.py map:=~/orbibot_ws/maps/your_map.yaml
```

## 📍 Step-by-Step Setup

### Step 1: Hardware Initialization
```bash
# Terminal 1: Start hardware interface
ros2 launch orbibot_hardware hardware.launch.py

# Wait for "Hardware initialized successfully" message
# Check status
ros2 topic echo /orbibot/system_status --once
```

### Step 2: Create Map (First Time Only)

#### Option A: 2D LIDAR SLAM
```bash
# Terminal 2: Start SLAM mapping
ros2 launch orbibot_navigation slam.launch.py

# Terminal 3: Manual control for mapping
ros2 launch orbibot_teleop teleop.launch.py

# Terminal 4: Monitor mapping progress
rviz2 -d ~/orbibot_ws/src/orbibot_navigation/config/slam.rviz
```

#### Option B: Enhanced SLAM with RealSense
```bash
# For better 3D obstacle detection
ros2 launch orbibot_navigation realsense_reliable.launch.py
```

**Mapping Instructions:**
1. Drive robot manually using keyboard teleop
2. Cover entire area systematically
3. Ensure good loop closure (return to starting position)
4. Check map quality in RViz

**Save the Map:**
```bash
# Save completed map
ros2 run nav2_map_server map_saver_cli -f ~/orbibot_ws/maps/my_house_map

# Creates: my_house_map.yaml and my_house_map.pgm
```

### Step 3: Autonomous Navigation

#### Basic Navigation
```bash
# Terminal 1: Hardware
ros2 launch orbibot_hardware hardware.launch.py

# Terminal 2: Navigation stack
ros2 launch orbibot_navigation navigation.launch.py map:=~/orbibot_ws/maps/my_house_map.yaml

# Terminal 3: Enhanced localization (optional)
ros2 launch orbibot_localization localization.launch.py

# Terminal 4: Monitoring
rviz2 -d ~/orbibot_ws/src/orbibot_navigation/config/nav2.rviz
```

#### Enhanced Navigation with RealSense
```bash
# For advanced obstacle avoidance
ros2 launch orbibot_navigation realsense_navigation.launch.py map:=~/orbibot_ws/maps/my_house_map.yaml
```

## 🎮 Operating Modes

### Mode 1: RViz Point-and-Click Navigation

1. **Set Initial Pose:**
   - Click "2D Pose Estimate" button in RViz
   - Click and drag on map to set robot's current position and orientation

2. **Navigate to Goal:**
   - Click "Nav2 Goal" button in RViz
   - Click on map destination
   - Robot will plan path and navigate autonomously

### Mode 2: Command Line Navigation
```bash
# Send navigation goal via CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 1.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

### Mode 3: Waypoint Following
```bash
# Navigate through multiple points
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses "
{
  poses: [
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 2.0, y: 1.0, z: 0.0},
        orientation: {w: 1.0}
      }
    },
    {
      header: {frame_id: 'map'}, 
      pose: {
        position: {x: 0.0, y: 3.0, z: 0.0},
        orientation: {z: 0.707, w: 0.707}
      }
    }
  ]
}"
```

## 🛡️ Safety Systems

### Emergency Stop
```bash
# Immediate stop via command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once

# Or use teleop emergency stop (X key)
ros2 run orbibot_teleop keyboard_teleop
```

### Motor Control
```bash
# Enable motors
ros2 service call /orbibot/set_motor_enable orbibot_msgs/srv/SetMotorEnable "{enable: true}"

# Disable motors
ros2 service call /orbibot/set_motor_enable orbibot_msgs/srv/SetMotorEnable "{enable: false}"
```

### System Monitoring
```bash
# Monitor system health
ros2 topic echo /orbibot/system_status
ros2 topic echo /orbibot/safety_status

# Monitor navigation status
ros2 topic echo /diagnostics
```

## 🔧 Troubleshooting

### Common Issues

#### Robot Not Moving
```bash
# Check motor enable status
ros2 topic echo /orbibot/system_status --once

# Enable motors if disabled
ros2 service call /orbibot/set_motor_enable orbibot_msgs/srv/SetMotorEnable "{enable: true}"

# Check for emergency stop
ros2 topic echo /orbibot/safety_status --once
```

#### Poor Localization
```bash
# Reset odometry
ros2 service call /orbibot/reset_odometry orbibot_msgs/srv/ResetOdometry

# Check sensor health
ros2 topic echo /imu/data --once
ros2 topic echo /scan --once

# Restart localization
ros2 launch orbibot_localization localization.launch.py
```

#### Navigation Failures
```bash
# Clear costmaps
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntirelyGlobalCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntirelyLocalCostmap

# Check path planning
ros2 topic echo /plan
ros2 topic echo /local_plan
```

#### Hardware Connection Issues
```bash
# Check device connections
ls -la /dev/motordriver /dev/lidar

# Restart hardware node
ros2 launch orbibot_hardware hardware.launch.py

# Check battery voltage
ros2 topic echo /orbibot/system_status
```

### Performance Optimization

#### Map Quality
- Ensure good lighting for visual odometry
- Drive slowly during mapping (< 0.2 m/s)
- Cover all areas systematically
- Verify loop closure by returning to start

#### Navigation Tuning
```bash
# Edit navigation parameters
nano ~/orbibot_ws/src/orbibot_navigation/config/nav2_params.yaml

# Key parameters to adjust:
# - xy_goal_tolerance: Goal reaching precision
# - max_vel_x/y: Maximum velocities
# - obstacle_range: Obstacle detection distance
```

## 📊 System Monitoring

### Real-time Status Dashboard
```bash
# Web-based monitoring
ros2 launch orbibot_webui webui.launch.py
# Open browser: http://localhost:8080
```

### Command Line Monitoring
```bash
# System overview
ros2 node list
ros2 topic list | grep orbibot

# Performance metrics
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /imu/data

# Resource usage
htop
```

## 🎯 Advanced Features

### Patrol Mode Script
Create `~/orbibot_ws/patrol_mission.py`:
```python
#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # Define patrol waypoints
    waypoints = [
        (2.0, 1.0, 0.0),    # Living room
        (0.0, 3.0, 1.57),   # Kitchen
        (-1.0, 0.0, 3.14),  # Bedroom
        (0.0, 0.0, 0.0)     # Home base
    ]
    
    # Execute patrol
    for x, y, yaw in waypoints:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        
        navigator.goToPose(goal_pose)
        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.1)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Autonomous Startup Script
Create `~/orbibot_ws/autonomous_startup.sh`:
```bash
#!/bin/bash

echo "🤖 Starting OrbiBot Autonomous Navigation..."

# Start hardware
ros2 launch orbibot_hardware hardware.launch.py &
HARDWARE_PID=$!
echo "⚡ Hardware started (PID: $HARDWARE_PID)"
sleep 5

# Start navigation
ros2 launch orbibot_navigation navigation.launch.py map:=~/orbibot_ws/maps/my_house_map.yaml &
NAVIGATION_PID=$!
echo "🗺️  Navigation started (PID: $NAVIGATION_PID)"
sleep 3

# Start enhanced localization
ros2 launch orbibot_localization localization.launch.py &
LOCALIZATION_PID=$!
echo "📍 Localization started (PID: $LOCALIZATION_PID)"
sleep 2

# Start monitoring
rviz2 -d ~/orbibot_ws/src/orbibot_navigation/config/nav2.rviz &
RVIZ_PID=$!
echo "📺 RViz started (PID: $RVIZ_PID)"

echo ""
echo "🎯 OrbiBot autonomous navigation ready!"
echo "💡 Use RViz to set 2D Pose Estimate and Nav2 Goals"
echo "🛑 Press Ctrl+C to shutdown all processes"

# Wait for Ctrl+C
trap 'echo "Shutting down..."; kill $HARDWARE_PID $NAVIGATION_PID $LOCALIZATION_PID $RVIZ_PID; exit' INT
wait
```

Make executable:
```bash
chmod +x ~/orbibot_ws/autonomous_startup.sh
```

## 📋 System Specifications

### Performance Metrics
- **Localization Accuracy**: ±5cm position, ±2° orientation
- **Navigation Speed**: 0.1-0.4 m/s (configurable)
- **Obstacle Detection**: 8m range (LIDAR), 3m range (RealSense)
- **Battery Life**: 2-3 hours continuous operation
- **Map Size**: Up to 100m x 100m indoor environments

### Operating Limits
- **Maximum Speed**: 0.5 m/s linear, 1.2 rad/s angular
- **Minimum Space**: 60cm wide corridors
- **Step Height**: Max 1cm (mecanum wheels)
- **Operating Temperature**: 0°C to 40°C
- **Surface**: Hard floors (tile, wood, concrete)

## 🆘 Emergency Procedures

### Immediate Stop
1. **Physical**: Power button on Raspberry Pi
2. **Software**: `Ctrl+C` in terminal or `X` key in teleop
3. **Remote**: Send empty `/cmd_vel` message

### Recovery Procedures
1. **Robot Stuck**: Clear path, restart navigation
2. **Lost Localization**: Set 2D pose estimate in RViz
3. **Hardware Fault**: Check connections, restart hardware node
4. **Low Battery**: Manual return to charging station

## 📞 Support

### Log Files
```bash
# ROS logs
ls ~/.ros/log/

# System logs
journalctl -u your_service_name
```

### Diagnostic Commands
```bash
# Full system check
ros2 launch orbibot_hardware hardware.launch.py
ros2 run orbibot_hardware diagnostics_check

# Network test
ros2 topic list
ros2 node list
```

---

## 📝 Quick Reference

### Essential Commands
```bash
# Start autonomous navigation
ros2 launch orbibot_navigation orbibot_navigation.launch.py map:=path/to/map.yaml

# Emergency stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once

# Enable motors
ros2 service call /orbibot/set_motor_enable orbibot_msgs/srv/SetMotorEnable "{enable: true}"

# Check status
ros2 topic echo /orbibot/system_status --once
```

### Key Topics
- `/cmd_vel` - Velocity commands
- `/scan` - LIDAR data
- `/odom` - Odometry
- `/imu/data` - IMU sensor data
- `/orbibot/system_status` - System health

### Important Services
- `/orbibot/set_motor_enable` - Motor control
- `/orbibot/reset_odometry` - Reset pose
- `/orbibot/calibrate_imu` - IMU calibration

---

**🎯 Your OrbiBot is now ready for autonomous operation! Follow this guide to safely operate your robot in autonomous mode.**

**⚠️ Safety First: Always supervise autonomous operation and ensure clear paths before sending navigation commands.**