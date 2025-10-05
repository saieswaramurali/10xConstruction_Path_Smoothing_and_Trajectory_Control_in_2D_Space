# Robot Bringup Package

**Package Name:** `robot_bringup`  
**Version:** 0.0.1  
**License:** Apache-2.0  
**Description:** Robot bringup package for launching TurtleBot3 simulation with Gazebo and RViz

---

## Overview

The `robot_bringup` package provides launch files and configurations to start the TurtleBot3 robot simulation environment. It integrates:

- **Gazebo Classic Simulator** - Physics simulation environment
- **TurtleBot3 Robot Model** - Burger model by default
- **RViz2 Visualization** - Real-time robot state and trajectory visualization

This package serves as the foundation for testing autonomous navigation, trajectory generation, and path following algorithms.

---

## Package Structure

```
robot_bringup/
├── CMakeLists.txt              # CMake build configuration
├── package.xml                 # ROS 2 package manifest
├── README.md                   # This documentation
├── launch/
│   └── turtlebot_sim.launch.py # Main simulation launch file
├── rviz/
│   ├── display.rviz            # Default RViz configuration
│   └── trajectory_display.rviz # Trajectory-specific RViz config
├── maps/                       # Map files (currently empty)
├── include/                    # C++ headers (if needed)
└── src/                        # C++ source files (if needed)
```

---

## Dependencies

### Runtime Dependencies
- `turtlebot3_gazebo` - TurtleBot3 Gazebo simulation package
- `gazebo_ros` - ROS 2 Gazebo integration
- `robot_state_publisher` - Robot state publishing
- `launch` - ROS 2 launch system
- `launch_ros` - ROS 2 launch actions

### System Requirements
- ROS 2 Humble Hawksbill
- Gazebo Classic 11.x
- Ubuntu 22.04 LTS
- TurtleBot3 packages installed

---

## Installation

### 1. Clone and Build

```bash
cd ~/Desktop/smooth_nav
colcon build --packages-select robot_bringup
source install/setup.bash
```

### 2. Verify Installation

```bash
ros2 pkg list | grep robot_bringup
```

---

## Usage

### Quick Start

```bash
# Source the workspace
source ~/Desktop/smooth_nav/install/setup.bash

# Launch the simulation
ros2 launch robot_bringup turtlebot_sim.launch.py
```

This command will:
1. Start Gazebo with an empty world
2. Spawn the TurtleBot3 Burger robot at the origin
3. Launch RViz2 with pre-configured displays

### Launch with Different Robot Model

```bash
# Launch with Waffle model
ros2 launch robot_bringup turtlebot_sim.launch.py model:=waffle

# Launch with Waffle Pi model
ros2 launch robot_bringup turtlebot_sim.launch.py model:=waffle_pi

# Default: Burger model
ros2 launch robot_bringup turtlebot_sim.launch.py model:=burger
```

---

## Launch Files

### `turtlebot_sim.launch.py`

**Purpose:** Main launch file to start complete simulation environment

**Launch Arguments:**

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `model` | string | `burger` | TurtleBot3 model type: `burger`, `waffle`, or `waffle_pi` |

**Nodes Launched:**
- **Gazebo Server** - Physics simulation backend
- **Gazebo Client** - 3D visualization window
- **Robot State Publisher** - Publishes robot TF transforms
- **RViz2** - Visualization interface with custom configuration

**Topics Published** (by simulation):
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/scan` (sensor_msgs/LaserScan) - LiDAR data
- `/imu` (sensor_msgs/Imu) - IMU sensor data
- `/joint_states` (sensor_msgs/JointState) - Wheel joint states
- `/tf` (tf2_msgs/TFMessage) - Transform tree

**Topics Subscribed** (by simulation):
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to control robot

**Environment Variables Set:**
- `TURTLEBOT3_MODEL` - Set to selected model type

---

## RViz Configurations

### `display.rviz`

**Default RViz configuration** with displays:
- Robot Model
- TF (coordinate frames)
- LaserScan visualization
- Odometry
- Grid reference

### `trajectory_display.rviz`

**Trajectory-specific configuration** with additional displays:
- Waypoints path visualization
- Smooth trajectory visualization
- Waypoint markers
- Trajectory markers

**To use:**
```bash
rviz2 -d $(ros2 pkg prefix robot_bringup)/share/robot_bringup/rviz/trajectory_display.rviz
```

---

## Integration with Other Packages

### With `robot_trajectory_generator`

```bash
# Terminal 1: Launch simulation
ros2 launch robot_bringup turtlebot_sim.launch.py

# Terminal 2: Launch trajectory system
ros2 launch robot_trajectory_generator complete_navigation.launch.py
```

### Manual Control (for testing)

```bash
# Launch simulation first
ros2 launch robot_bringup turtlebot_sim.launch.py

# In another terminal, use keyboard teleop
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## Coordinate Frames

The simulation uses the following coordinate frames:

```
odom (world frame)
  └── base_footprint (robot base)
       └── base_link
            ├── wheel_left_link
            ├── wheel_right_link
            ├── caster_back_link
            └── base_scan (LiDAR)
```

**Frame Descriptions:**
- `odom` - Fixed world frame, origin of simulation
- `base_footprint` - Robot's ground projection
- `base_link` - Robot's main body center
- `base_scan` - LiDAR sensor frame

---

## Common Topics

### Published by Simulation

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odom` | nav_msgs/Odometry | 50 Hz | Robot position and velocity |
| `/scan` | sensor_msgs/LaserScan | 5 Hz | 360° LiDAR data |
| `/imu` | sensor_msgs/Imu | 200 Hz | Accelerometer and gyroscope |
| `/joint_states` | sensor_msgs/JointState | 50 Hz | Wheel encoder data |
| `/tf` | tf2_msgs/TFMessage | 50 Hz | Transform tree |

### Subscribed by Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (linear x, angular z) |

---

## Troubleshooting

### Gazebo doesn't start

```bash
# Check if Gazebo is already running
killall -9 gazebo gzserver gzclient

# Try launching again
ros2 launch robot_bringup turtlebot_sim.launch.py
```

### Robot model not visible

```bash
# Ensure TURTLEBOT3_MODEL is set
echo $TURTLEBOT3_MODEL

# If not set, add to ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

### RViz not displaying properly

```bash
# Reset RViz configuration
rm -rf ~/.rviz2/

# Launch with default config
ros2 launch robot_bringup turtlebot_sim.launch.py
```

### "No robot model loaded" in RViz

- Check that `robot_state_publisher` node is running
- Verify `/robot_description` topic is being published
- Ensure TurtleBot3 packages are properly installed

### Gazebo crashes or freezes

```bash
# Clear Gazebo cache
rm -rf ~/.gazebo/

# Restart simulation
ros2 launch robot_bringup turtlebot_sim.launch.py
```

---

## Performance Tips

### Reduce CPU Usage

1. **Lower Gazebo update rate** - Edit world file
2. **Disable shadows** in Gazebo (View → Shadows)
3. **Close unnecessary applications**

### Improve Visualization

1. **Set Fixed Frame** to `odom` in RViz
2. **Adjust grid cell size** for better scale reference
3. **Enable anti-aliasing** in RViz (Global Options)

---

## Extending the Package

### Adding Custom Worlds

```bash
# Create world file in maps/ directory
touch maps/custom_world.world

# Modify launch file to include custom world
# Edit launch/turtlebot_sim.launch.py
```

### Adding Custom RViz Configs

```bash
# Save current RViz config
# File → Save Config As → robot_bringup/rviz/my_config.rviz

# Use in launch file
rviz_config_file = PathJoinSubstitution([
    FindPackageShare('robot_bringup'),
    'rviz',
    'my_config.rviz'
])
```

---

## Testing

### Verify Simulation is Running

```bash
# Check active nodes
ros2 node list

# Check available topics
ros2 topic list

# Test odometry
ros2 topic echo /odom --once

# Test velocity command
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"
```

### Verify TF Tree

```bash
# Install tf2_tools if needed
sudo apt install ros-humble-tf2-tools

# View TF tree
ros2 run tf2_tools view_frames

# View specific transform
ros2 run tf2_ros tf2_echo odom base_link
```

---

## Related Packages

- **robot_trajectory_generator** - Trajectory generation and path following
- **turtlebot3_gazebo** - TurtleBot3 Gazebo simulation
- **turtlebot3_description** - Robot URDF models

---

## Support

For issues or questions:
1. Check ROS 2 Humble documentation: https://docs.ros.org/en/humble/
2. TurtleBot3 documentation: https://emanual.robotis.com/docs/en/platform/turtlebot3/
3. Gazebo documentation: https://gazebosim.org/

---

## License

Apache-2.0

---

## Author

Sai - Robot Navigation System

**Last Updated:** October 2025
