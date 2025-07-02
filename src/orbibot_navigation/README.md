# OrbiBot Navigation Package

This package provides autonomous navigation capabilities for OrbiBot.

## Prerequisites

Before using this package, you need to install the Nav2 navigation stack and SLAM Toolbox:

```bash
# Install Nav2 stack
sudo apt update
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Install SLAM Toolbox
sudo apt install -y ros-jazzy-slam-toolbox

# Install additional navigation dependencies
sudo apt install -y ros-jazzy-robot-localization ros-jazzy-dwb-*
```

## Package Structure

```
orbibot_navigation/
├── config/
│   ├── slam_params.yaml      # SLAM Toolbox configuration
│   └── nav2_params.yaml      # Nav2 stack parameters
├── launch/
│   ├── slam.launch.py        # SLAM mapping
│   ├── navigation.launch.py  # Nav2 navigation
│   ├── orbibot_navigation.launch.py  # Complete system
│   └── basic_navigation.launch.py    # Basic setup
├── maps/                     # Saved maps
└── params/                   # Additional parameters
```

## Usage

### 1. SLAM Mapping

Create a map of your environment:

```bash
# Launch the complete system with SLAM
ros2 launch orbibot_navigation orbibot_navigation.launch.py slam:=true

# Or just SLAM
ros2 launch orbibot_navigation slam.launch.py

# Drive the robot around to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save the map when done
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 2. Autonomous Navigation

Navigate autonomously using a pre-built map:

```bash
# Launch with saved map
ros2 launch orbibot_navigation orbibot_navigation.launch.py slam:=false navigation:=true

# Set navigation goal via RViz 2D Nav Goal tool
# Or programmatically:
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 3. Complete System Launch

Launch everything at once:

```bash
# SLAM mode (mapping)
ros2 launch orbibot_navigation orbibot_navigation.launch.py slam:=true navigation:=true hardware:=true

# Navigation mode (with pre-built map)
ros2 launch orbibot_navigation orbibot_navigation.launch.py slam:=false navigation:=true hardware:=true
```

## Key Features

- **SLAM Toolbox Integration**: Real-time mapping with loop closure
- **Nav2 Stack**: Complete autonomous navigation
- **Mecanum Drive Support**: Optimized for holonomic movement
- **Sensor Fusion**: Uses enhanced localization from orbibot_localization
- **Safety Features**: Obstacle avoidance and recovery behaviors

## Topics

### Subscribed Topics
- `/scan` - LIDAR data for SLAM and obstacle detection
- `/orbibot/odometry/filtered` - Enhanced odometry from EKF
- `/goal_pose` - Navigation goals

### Published Topics
- `/map` - Occupancy grid map
- `/cmd_vel` - Velocity commands to robot
- `/plan` - Global path plan
- `/local_plan` - Local trajectory

## Configuration

### SLAM Parameters (`config/slam_params.yaml`)
- Map resolution, update rates
- Loop closure settings
- Scan matching parameters

### Navigation Parameters (`config/nav2_params.yaml`)
- Costmap configuration
- Path planning algorithms
- Controller parameters optimized for mecanum drive

## Troubleshooting

1. **No map being built**: Check that `/scan` topic is published and LIDAR is working
2. **Poor localization**: Verify that `/orbibot/odometry/filtered` is published
3. **Robot not moving**: Check that `/cmd_vel` commands reach orbibot_hardware
4. **Path planning fails**: Verify map quality and goal accessibility

## Next Steps

- Install required packages with apt
- Test SLAM mapping
- Build maps of your environment
- Configure navigation parameters for your specific needs
- Integrate with mission planning for complex tasks