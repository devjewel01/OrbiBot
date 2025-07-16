# OrbiBot Simulation

Complete Gazebo simulation environment for OrbiBot mecanum-wheeled mobile robot.

## Overview

This simulation environment provides an exact digital copy of the OrbiBot robot with all sensors and capabilities, allowing you to:
- Test and develop at home without physical hardware
- Prototype navigation algorithms safely
- Test SLAM mapping in virtual environments
- Validate control systems before deployment

## Package Structure

### orbibot_gazebo
- **Purpose**: Gazebo simulation environment and world files
- **Contents**: 
  - URDF with Gazebo plugins
  - World files (empty, office environments)
  - Launch files for different scenarios

### orbibot_simulation  
- **Purpose**: Simulated hardware interface that mirrors the real robot
- **Contents**:
  - `simulation_hardware_node.py` - Virtual hardware interface
  - `simulation_control_node.py` - Odometry and control coordination
  - Launch files for complete simulation stack

## Installation

### Prerequisites

1. **Build the simulation packages:**
   ```bash
   cd ~/orbibot_ws
   colcon build --packages-select orbibot_gazebo orbibot_simulation
   source install/setup.bash
   ```

2. **Optional - Install Gazebo for full 3D simulation:**
   ```bash
   sudo apt install ros-jazzy-gz-sim-vendor ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
   ```
   
   **Note**: Gazebo is not required for basic simulation. The simulation works without it, but Gazebo provides full 3D visualization and physics.

## Usage

### 1. Basic Simulation (Hardware Interface Only)

Start the simulated hardware interface without Gazebo:
```bash
ros2 launch orbibot_simulation simulation.launch.py
```

### 2. Full Gazebo Simulation (requires Gazebo packages)

Start complete simulation with Gazebo GUI:
```bash
ros2 launch orbibot_simulation simulation.launch.py enable_gazebo:=true world:=empty gui:=true
```

Or use the dedicated Gazebo launcher:
```bash
ros2 launch orbibot_gazebo gazebo_sim.launch.py world:=empty gui:=true
```

### 3. Simulation with Teleoperation

Test keyboard control in simulation:
```bash
ros2 launch orbibot_simulation simulation_teleop.launch.py world:=empty teleop_type:=keyboard
```

### 4. Enable Motors

The simulation requires motors to be enabled before movement:
```bash
ros2 service call /orbibot/set_motor_enable orbibot_msgs/srv/SetMotorEnable "{enable: true}"
```

### 5. Manual Control

Send velocity commands directly:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

## Simulation Features

### Hardware Interface
- **Identical Topics**: Same topics as real robot (`/cmd_vel`, `/joint_states`, `/imu/data`, etc.)
- **Motor Control**: Enable/disable motors via service calls
- **Battery Simulation**: Realistic battery drain during operation
- **Safety Systems**: Command timeout, emergency stop, temperature monitoring

### Sensor Simulation
- **RPLIDAR A1**: 360° laser scanner with realistic noise
- **Intel RealSense D435**: RGB and depth camera simulation
- **MPU9250 IMU**: Inertial measurement unit with noise modeling
- **Wheel Encoders**: Accurate position and velocity feedback

### Physics Simulation
- **Mecanum Drive**: Accurate holonomic motion modeling
- **Wheel Dynamics**: Realistic wheel friction and inertia
- **Collision Detection**: Accurate robot-environment interaction

## Testing Commands

### Check Status
```bash
# List active topics
ros2 topic list | grep orbibot

# Check motor feedback
ros2 topic echo /orbibot/motor_feedback --once

# Check system status
ros2 topic echo /orbibot/system_status --once

# Check joint states
ros2 topic echo /joint_states --once
```

### Movement Testing
```bash
# Forward movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once

# Strafe left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.5}}" --once

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

## Integration with Existing Packages

### Using with SLAM
```bash
# Start simulation
ros2 launch orbibot_simulation simulation.launch.py

# Enable motors
ros2 service call /orbibot/set_motor_enable orbibot_msgs/srv/SetMotorEnable "{enable: true}"

# Start SLAM (in separate terminal)
ros2 launch orbibot_slam slam.launch.py use_sim_time:=true
```

### Using with Navigation
```bash
# Start simulation
ros2 launch orbibot_simulation simulation.launch.py

# Enable motors
ros2 service call /orbibot/set_motor_enable orbibot_msgs/srv/SetMotorEnable "{enable: true}"

# Start navigation (in separate terminal)
ros2 launch orbibot_navigation navigation.launch.py use_sim_time:=true
```

### Using with Control Package
```bash
# Start simulation
ros2 launch orbibot_simulation simulation.launch.py

# Start control manager (in separate terminal)
ros2 launch orbibot_control control.launch.py use_sim_time:=true
```

## Configuration

### Simulation Parameters
Edit `config/simulation_params.yaml` to adjust:
- Physics settings (timestep, solver)
- Sensor parameters (noise, update rates)
- Robot dynamics (mass, friction)
- Battery simulation parameters

### World Files
Available worlds in `orbibot_gazebo/worlds/`:
- `empty.world` - Basic environment for testing
- `office.world` - Office environment with furniture

## Troubleshooting

### Common Issues

1. **Gazebo not starting**: Install Gazebo dependencies
2. **Topics not publishing**: Check if nodes are running with `ros2 node list`
3. **Robot not moving**: Ensure motors are enabled via service call
4. **Simulation time issues**: Always use `use_sim_time:=true` with other packages

### Performance Tips

1. **Reduce GUI load**: Use `gui:=false` when not needed
2. **Sensor rates**: Adjust update rates in config files
3. **Physics settings**: Tune timestep for performance/accuracy balance

## Development

### Adding New Sensors
1. Add sensor to `orbibot_gazebo.urdf.xacro`
2. Configure Gazebo plugin parameters
3. Update simulation hardware node if needed

### Creating New Worlds
1. Create new `.world` file in `orbibot_gazebo/worlds/`
2. Update launch files to support new world
3. Test with basic simulation

## Compatibility

- **ROS 2 Distro**: Jazzy
- **Gazebo**: Ignition Gazebo (gz-sim)
- **Python**: 3.12+
- **Ubuntu**: 24.04

## Support

For issues or questions:
1. Check this README for common solutions
2. Review console output for error messages
3. Test with minimal simulation first
4. Ensure all dependencies are installed

## Future Enhancements

Planned features:
- Multiple robot simulation
- Advanced sensor models
- Realistic environment physics
- Performance optimization
- Additional world scenarios