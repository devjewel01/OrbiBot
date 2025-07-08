# OrbiBot Navigation Package

Complete autonomous navigation system for OrbiBot mecanum-wheeled mobile robot.

## Architecture Overview

This package provides a clean, hierarchical launch system for OrbiBot navigation with three main modes:

### Navigation Modes

1. **SLAM Mode** (`mode:=slam`) - Simultaneous Localization and Mapping
   - Creates maps while navigating
   - Combines SLAM toolbox with Nav2 navigation stack
   - Default mode for exploration and mapping

2. **Localization Mode** (`mode:=localization`) - Map-based Navigation
   - Uses pre-built maps for localization with AMCL
   - Full Nav2 stack with autonomous navigation
   - Requires existing map file

3. **Mapping Only Mode** (`mode:=mapping_only`) - Pure Mapping
   - SLAM mapping without navigation planning
   - Useful for teleoperated mapping sessions

## Launch File Structure

### Main Launch File
- `orbibot_navigation.launch.py` - **Primary entry point** for all navigation modes

### Modular Launch Files
- `hardware.launch.py` - Robot hardware system (description, hardware, control)
- `sensors.launch.py` - Sensor configuration (LIDAR, RealSense, or both)
- `navigation.launch.py` - Mode-specific navigation launching
- `slam.launch.py` - SLAM toolbox configuration
- `rviz.launch.py` - Mode-specific RViz visualization

## Configuration Files

### Optimized Parameters
- `orbibot_nav2_params.yaml` - Nav2 stack parameters optimized for OrbiBot
- `orbibot_slam_params.yaml` - SLAM toolbox parameters for OrbiBot

### Legacy Files (Deprecated)
- `nav2_params.yaml` - Original Nav2 parameters (kept for compatibility)
- `slam_params.yaml` - Original SLAM parameters (kept for compatibility)

## Prerequisites

Before using this package, install the required Nav2 and SLAM dependencies:

```bash
# Install Nav2 stack
sudo apt update
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Install SLAM Toolbox
sudo apt install -y ros-jazzy-slam-toolbox

# Install additional navigation dependencies
sudo apt install -y ros-jazzy-robot-localization ros-jazzy-dwb-*
```

## Usage Examples

### Basic SLAM Navigation
```bash
# Complete system with SLAM
ros2 launch orbibot_navigation orbibot_navigation.launch.py

# SLAM with RViz
ros2 launch orbibot_navigation orbibot_navigation.launch.py rviz:=true

# SLAM without hardware (simulation)
ros2 launch orbibot_navigation orbibot_navigation.launch.py hardware:=false
```

### Localization-based Navigation
```bash
# Navigate with existing map
ros2 launch orbibot_navigation orbibot_navigation.launch.py mode:=localization map_file:=/path/to/map.yaml

# Localization with RViz
ros2 launch orbibot_navigation orbibot_navigation.launch.py mode:=localization rviz:=true
```

### Mapping Only (Teleoperated)
```bash
# Pure mapping without navigation
ros2 launch orbibot_navigation orbibot_navigation.launch.py mode:=mapping_only

# Combined with teleop
ros2 launch orbibot_navigation orbibot_navigation.launch.py mode:=mapping_only &
ros2 run orbibot_teleop keyboard_teleop
```

### Sensor Configurations
```bash
# LIDAR only (default)
ros2 launch orbibot_navigation orbibot_navigation.launch.py sensors:=lidar

# RealSense camera only
ros2 launch orbibot_navigation orbibot_navigation.launch.py sensors:=realsense

# Both LIDAR and RealSense
ros2 launch orbibot_navigation orbibot_navigation.launch.py sensors:=both
```

## Launch Arguments

### Main Launch Arguments
- `mode` - Navigation mode: `slam`, `localization`, `mapping_only` (default: `slam`)
- `use_sim_time` - Use simulation time (default: `false`)
- `hardware` - Launch hardware nodes (default: `true`)
- `sensors` - Sensor configuration: `lidar`, `realsense`, `both` (default: `lidar`)
- `rviz` - Launch RViz visualization (default: `true`)
- `map_file` - Map file path for localization mode (default: `''`)
- `log_level` - Log level for all nodes (default: `info`)

### Common Combinations
```bash
# Development with simulation
ros2 launch orbibot_navigation orbibot_navigation.launch.py hardware:=false use_sim_time:=true

# Production mapping
ros2 launch orbibot_navigation orbibot_navigation.launch.py mode:=slam sensors:=both

# Autonomous navigation
ros2 launch orbibot_navigation orbibot_navigation.launch.py mode:=localization map_file:=maps/office.yaml
```

## Key Features

### Optimized for OrbiBot
- Mecanum wheel kinematics support
- Appropriate velocity and acceleration limits
- Optimized costmap parameters for robot size
- LIDAR and RealSense sensor integration

### Modular Design
- Clear separation of concerns
- Easy to extend and modify
- Consistent parameter passing
- Conditional launching based on mode

### Production Ready
- Comprehensive error handling
- Configurable logging levels
- Respawn and timeout handling
- Performance optimizations

## Dependencies

### Required ROS 2 Packages
- `nav2_bringup` - Nav2 navigation stack
- `slam_toolbox` - SLAM implementation
- `rplidar_ros` - LIDAR driver
- `realsense2_camera` - RealSense camera driver (optional)
- `depthimage_to_laserscan` - Depth to laser scan converter

### OrbiBot Packages
- `orbibot_description` - Robot URDF and transforms
- `orbibot_hardware` - Hardware interface
- `orbibot_control` - Control system and odometry
- `orbibot_teleop` - Teleoperation (for mapping mode)

## Migration from Old Launch Files

### Old vs New Launch Files
| Old File | New Equivalent | Notes |
|----------|---------------|-------|
| `basic_navigation.launch.py` | `orbibot_navigation.launch.py` | Use default parameters |
| `navigation_basic.launch.py` | `orbibot_navigation.launch.py` | Removed redundancy |
| `navigation_simple.launch.py` | `orbibot_navigation.launch.py` | Simplified interface |
| `realsense_navigation.launch.py` | `orbibot_navigation.launch.py sensors:=realsense` | Sensor selection |
| `realsense_reliable.launch.py` | `orbibot_navigation.launch.py sensors:=realsense` | Consolidated |
| `lidar.launch.py` | `sensors.launch.py` | Modular sensors |

### Migration Commands
```bash
# Old command
ros2 launch orbibot_navigation basic_navigation.launch.py

# New equivalent
ros2 launch orbibot_navigation orbibot_navigation.launch.py

# Old command
ros2 launch orbibot_navigation realsense_navigation.launch.py

# New equivalent
ros2 launch orbibot_navigation orbibot_navigation.launch.py sensors:=realsense
```

## Topics

### Subscribed Topics
- `/scan` - LIDAR data for SLAM and obstacle detection
- `/odom` - Odometry data from orbibot_control
- `/goal_pose` - Navigation goals
- `/cmd_vel` - Velocity commands (output to robot)

### Published Topics
- `/map` - Occupancy grid map
- `/plan` - Global path plan
- `/local_plan` - Local trajectory
- `/costmap` - Local and global costmaps

## Troubleshooting

### Common Issues
1. **No map being built** - Check LIDAR topics and SLAM parameters
2. **Poor localization** - Verify map quality and AMCL parameters
3. **Navigation fails** - Check costmap parameters and robot footprint
4. **Sensor not detected** - Verify device permissions and connections

### Debug Commands
```bash
# Check topics
ros2 topic list | grep -E "(scan|map|odom|cmd_vel)"

# Monitor navigation status
ros2 topic echo /navigate_to_pose/_action/status

# Check transforms
ros2 run tf2_tools view_frames
```

## Performance Tuning

### For Better Mapping
- Reduce `minimum_travel_distance` in SLAM params
- Increase `map_update_interval` for real-time mapping
- Adjust `correlation_search_space_dimension` for accuracy

### For Better Navigation
- Tune `controller_frequency` for responsiveness
- Adjust `xy_goal_tolerance` for precision
- Modify `max_vel_x/y` for speed requirements

## Future Enhancements

- Multi-robot coordination support
- Enhanced sensor fusion
- Advanced behavior trees
- Mission planning integration



For SLAM + Autonomous Navigation:

  # Robot: Start SLAM with autonomous navigation
  ros2 launch orbibot_navigation
  autonomous_navigation.launch.py

  # Computer: Monitor with RViz
  ros2 launch nav2_bringup rviz_launch.py
  rviz_config:=/home/orbitax/orbibot_ws/src/orbibot_navigat
  ion/config/orbibot_slam.rviz

  For Map-based Autonomous Navigation:

  # Robot: Start with existing map
  ros2 launch orbibot_navigation
  autonomous_navigation.launch.py
  map:=~/orbibot_ws/maps/your_map.yaml

  # Computer: Monitor with RViz
  ros2 launch nav2_bringup rviz_launch.py

  Standalone Autonomous Navigation:

  # After navigation stack is running
  ros2 run orbibot_navigation autonomous_navigation.py