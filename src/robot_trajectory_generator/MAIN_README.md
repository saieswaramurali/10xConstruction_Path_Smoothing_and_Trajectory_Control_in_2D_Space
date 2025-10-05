# Robot Trajectory Generator Package

**Package Name:** `robot_trajectory_generator`  
**Version:** 1.0.0  
**License:** Apache-2.0  
**Description:** Complete autonomous navigation system with smooth trajectory generation, velocity profiling, and path following control for ROS 2

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Package Structure](#package-structure)
- [Nodes Documentation](#nodes-documentation)
- [Launch Files](#launch-files)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Topics and Services](#topics-and-services)
- [Test Cases](#test-cases)
- [Usage Examples](#usage-examples)
- [Parameters](#parameters)
- [Algorithms](#algorithms)
- [Troubleshooting](#troubleshooting)

---

## Overview

The `robot_trajectory_generator` package provides a complete autonomous navigation pipeline that transforms discrete waypoints into smooth, velocity-optimized trajectories for robot navigation. It integrates path smoothing, velocity profiling, and path following control into a unified system.

### Pipeline Flow

```
Waypoints → Path Smoothing → Velocity Profile → Path Following → Robot Motion
  (Input)   (Catmull-Rom)   (Curvature-based)  (Pure Pursuit)    (cmd_vel)
```

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         USER INPUT                                │
│  • RViz Click Points                                              │
│  • Test Case Publisher (20 predefined patterns)                   │
│  • Manual Entry                                                   │
└────────────────┬─────────────────────────────────────────────────┘
                 │ /clicked_point
                 ▼
┌──────────────────────────────────────────────────────────────────┐
│  NODE 1: WAYPOINT COLLECTOR                                       │
│  • Collects clicked points from RViz                              │
│  • Stores waypoints [(x₀,y₀), (x₁,y₁), ..., (xₙ,yₙ)]            │
│  • Publishes as nav_msgs/Path                                     │
│  • Creates visualization markers                                  │
└────────────────┬─────────────────────────────────────────────────┘
                 │ /waypoints
                 ▼
┌──────────────────────────────────────────────────────────────────┐
│  NODE 2: CATMULL-ROM GENERATOR                                    │
│  • Generates smooth trajectory using Catmull-Rom splines          │
│  • Centripetal parameterization (α=0.5)                           │
│  • 200+ interpolated points                                       │
│  • Passes through ALL waypoints (no overshoot)                    │
└────────────────┬─────────────────────────────────────────────────┘
                 │ /smooth_trajectory
                 ▼
┌──────────────────────────────────────────────────────────────────┐
│  NODE 3: VELOCITY PROFILER                                        │
│  • Analyzes path curvature at each point                          │
│  • Assigns velocities: fast on straights, slow on curves          │
│  • Creates time-parameterized trajectory                          │
│  • Format: [(x₀,y₀,t₀,v₀), ..., (xₙ,yₙ,tₙ,vₙ)]                 │
└────────────────┬─────────────────────────────────────────────────┘
                 │ /trajectory_with_velocity
                 ▼
┌──────────────────────────────────────────────────────────────────┐
│  NODE 4: PURE PURSUIT CONTROLLER                                  │
│  • Subscribes to trajectory and robot odometry                    │
│  • Calculates lookahead point dynamically                         │
│  • Computes steering commands                                     │
│  • Publishes velocity commands to robot                           │
└────────────────┬─────────────────────────────────────────────────┘
                 │ /cmd_vel
                 ▼
┌──────────────────────────────────────────────────────────────────┐
│                    TURTLEBOT3 ROBOT                               │
│  • Executes motion commands                                       │
│  • Publishes odometry feedback                                    │
└──────────────────────────────────────────────────────────────────┘
```

---

## Features

### ✨ Core Capabilities

- **Smooth Trajectory Generation** - Catmull-Rom splines with centripetal parameterization
- **Velocity Optimization** - Curvature-aware speed assignment (faster on straights, slower on curves)
- **Path Following Control** - Pure Pursuit algorithm with dynamic lookahead
- **Interactive Waypoint Collection** - Click points in RViz or use predefined test cases
- **Real-time Visualization** - Complete path visualization with markers
- **Time Parameterization** - Full trajectory with timestamps `[(x,y,t,v)]`

### 🎯 Key Advantages

- **No Overshoot** - Trajectory passes through all waypoints exactly
- **Sharp Turn Handling** - Excellent performance on U-turns, zigzags, parking maneuvers
- **Modular Design** - Each node can be used independently
- **Easy Testing** - 20 predefined test cases included
- **Professional Quality** - Complete documentation and error handling

---

## Package Structure

```
robot_trajectory_generator/
├── package.xml                          # ROS 2 package manifest
├── setup.py                             # Python package configuration
├── setup.cfg                            # Python setup configuration
├── MAIN_README.md                       # This file (complete documentation)
├── README.md                            # Quick start guide
├── ARCHITECTURE.md                      # Architecture diagrams
├── COMPLETE_SYSTEM_GUIDE.md             # Detailed usage guide
│
├── launch/                              # Launch files
│   ├── complete_navigation.launch.py   # Full system (simulator + all nodes)
│   ├── trajectory_system.launch.py     # Trajectory generation only
│   └── waypoint_collector.launch.py    # Waypoint collector only
│
├── robot_trajectory_generator/          # Python package
│   ├── __init__.py                      # Package initialization
│   ├── waypoint_collector.py           # Node 1: Waypoint collection
│   ├── catmull_rom_generator.py        # Node 2: Trajectory generation
│   ├── velocity_profiler.py            # Node 3: Velocity assignment
│   ├── pure_pursuit_controller.py      # Node 4: Path following
│   ├── test_case_publisher.py          # Test case provider
│   ├── trajectory_analyzer.py          # Analysis and debugging tool
│   └── trajectory_generator_example.py # Example subscriber
│
└── resource/                            # ROS 2 resources
    └── robot_trajectory_generator
```

---

## Nodes Documentation

### 1. Waypoint Collector Node

**Executable:** `waypoint_collector`  
**Purpose:** Collects waypoints from RViz clicks or programmatic input

**Subscribed Topics:**
- `/clicked_point` (geometry_msgs/PointStamped) - RViz published points

**Published Topics:**
- `/waypoints` (nav_msgs/Path) - Collected waypoints as path
- `/waypoint_markers` (visualization_msgs/MarkerArray) - Visualization markers

**Services:**
- `/clear_waypoints` (std_srvs/Trigger) - Clear all collected waypoints

**Parameters:**
- `frame_id` (string, default: 'odom') - Reference frame
- `max_waypoints` (int, default: 100) - Maximum waypoints to collect
- `default_z` (float, default: 0.0) - Default Z coordinate

**Usage:**
```bash
ros2 run robot_trajectory_generator waypoint_collector
```

---

### 2. Catmull-Rom Generator Node

**Executable:** `catmull_rom_generator`  
**Purpose:** Generates smooth trajectories using Catmull-Rom splines

**Algorithm:** Catmull-Rom spline with centripetal parameterization (α=0.5)

**Subscribed Topics:**
- `/waypoints` (nav_msgs/Path) - Input waypoints

**Published Topics:**
- `/smooth_trajectory` (nav_msgs/Path) - Smooth interpolated trajectory
- `/trajectory_markers` (visualization_msgs/MarkerArray) - Trajectory visualization

**Parameters:**
- `num_points` (int, default: 200) - Number of interpolated points
- `alpha` (float, default: 0.5) - Parameterization (0.0=uniform, 0.5=centripetal, 1.0=chordal)
- `frame_id` (string, default: 'odom') - Reference frame
- `auto_generate` (bool, default: true) - Auto-generate on new waypoints

**Key Features:**
- ✅ Passes through ALL waypoints exactly
- ✅ No overshooting on sharp corners
- ✅ Best for varied curvature paths
- ✅ C1 continuous (smooth velocity)

**Usage:**
```bash
ros2 run robot_trajectory_generator catmull_rom_generator
```

---

### 3. Velocity Profiler Node

**Executable:** `velocity_profiler`  
**Purpose:** Assigns velocities based on path curvature

**Algorithm:** Curvature-based velocity assignment
- Higher curvature (tight turns) → Lower velocity
- Lower curvature (straight lines) → Higher velocity

**Subscribed Topics:**
- `/smooth_trajectory` (nav_msgs/Path) - Smooth trajectory input

**Published Topics:**
- `/trajectory_with_velocity` (nav_msgs/Path) - Trajectory with velocity profile

**Parameters:**
- `max_linear_vel` (float, default: 0.35) - Maximum linear velocity (m/s)
- `min_linear_vel` (float, default: 0.08) - Minimum linear velocity (m/s)
- `max_angular_vel` (float, default: 1.5) - Maximum angular velocity (rad/s)
- `curvature_threshold` (float, default: 0.3) - Curvature sensitivity (1/m)
- `lookahead_points` (int, default: 5) - Points to look ahead for curvature

**Output Format:**
- Time-parameterized trajectory: `[(x₀,y₀,t₀,v₀), (x₁,y₁,t₁,v₁), ..., (xₙ,yₙ,tₙ,vₙ)]`

**Usage:**
```bash
ros2 run robot_trajectory_generator velocity_profiler
```

---

### 4. Pure Pursuit Controller Node

**Executable:** `pure_pursuit_controller`  
**Purpose:** Path following control using Pure Pursuit algorithm

**Algorithm:** Pure Pursuit with dynamic lookahead
1. Find closest point on path to robot
2. Find lookahead point at distance L from robot
3. Calculate curvature to reach lookahead point
4. Convert to linear and angular velocities
5. Publish to /cmd_vel

**Subscribed Topics:**
- `/trajectory_with_velocity` (nav_msgs/Path) - Target trajectory
- `/odom` (nav_msgs/Odometry) - Robot odometry

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to robot

**Parameters:**
- `lookahead_distance` (float, default: 0.5) - Lookahead distance (m)
- `min_lookahead` (float, default: 0.25) - Minimum lookahead (m)
- `max_lookahead` (float, default: 0.8) - Maximum lookahead (m)
- `goal_tolerance` (float, default: 0.12) - Goal reached tolerance (m)
- `max_linear_vel` (float, default: 0.35) - Maximum velocity (m/s)
- `max_angular_vel` (float, default: 1.5) - Maximum angular velocity (rad/s)
- `use_dynamic_lookahead` (bool, default: true) - Scale lookahead with velocity

**Usage:**
```bash
ros2 run robot_trajectory_generator pure_pursuit_controller
```

---

### 5. Test Case Publisher Node

**Executable:** `test_case_publisher`  
**Purpose:** Publishes predefined test cases as waypoints

**Features:**
- 20 predefined test patterns
- Interactive menu selection
- Y-axis mirroring option (flip left/right)
- Custom manual entry

**Test Cases Available:**
1. Square - Basic 4-corner path
2. Triangle - 3-point navigation
3. Circle - Smooth circular path
4. Figure-8 - Complex curves
5. Zigzag - Sharp alternating turns
6. Star - 5-pointed star pattern
7. U-Turn - 180-degree turn
8. Slalom - Weaving pattern
9. Parking - Parallel parking maneuver
10. Corridor - Narrow corridor navigation
11. Roundabout - Circular intersection
12. Chicane - Racing chicane
13. Warehouse - Warehouse aisles
14. Obstacle - S-shaped avoidance
15. Line - Straight line
16. Rhombus - Diamond shape
17. S-Curve - Highway-like curves
18. Spiral - Expanding spiral
19. Hexagon - 6-sided polygon
20. Custom - Manual entry

**Usage:**
```bash
ros2 run robot_trajectory_generator test_case_publisher

# Then select test case number (e.g., 1 for Square)
# Add 'm' for mirrored version (e.g., 1m for mirrored square)
```

---

### 6. Trajectory Analyzer Node

**Executable:** `trajectory_analyzer`  
**Purpose:** Analyzes and displays time-parameterized trajectory information

**Subscribed Topics:**
- `/trajectory_with_velocity` (nav_msgs/Path) - Trajectory with velocities

**Output:**
- Displays trajectory in format: `[(x,y,t,v), ...]`
- Shows statistics: total distance, time, velocities
- Prints first 10 and last 5 points

**Usage:**
```bash
ros2 run robot_trajectory_generator trajectory_analyzer
```

---

## Launch Files

### 1. `complete_navigation.launch.py`

**Purpose:** Launch complete system including simulator

**What it starts:**
- TurtleBot3 Gazebo simulation
- RViz with trajectory visualization
- Waypoint collector node
- Catmull-Rom generator node
- Velocity profiler node
- Pure Pursuit controller node

**Usage:**
```bash
ros2 launch robot_trajectory_generator complete_navigation.launch.py
```

**Parameters:**
```bash
ros2 launch robot_trajectory_generator complete_navigation.launch.py \
    frame_id:=odom \
    num_points:=200 \
    alpha:=0.5 \
    max_linear_vel:=0.22 \
    lookahead_distance:=0.5
```

---

### 2. `trajectory_system.launch.py`

**Purpose:** Launch trajectory generation system only (no simulator)

**What it starts:**
- Waypoint collector node
- Catmull-Rom generator node

**Usage:**
```bash
ros2 launch robot_trajectory_generator trajectory_system.launch.py
```

**Use when:** You want trajectory generation without robot control

---

### 3. `waypoint_collector.launch.py`

**Purpose:** Launch waypoint collector only

**Usage:**
```bash
ros2 launch robot_trajectory_generator waypoint_collector.launch.py
```

---

## Installation

### Prerequisites

```bash
# ROS 2 Humble must be installed
source /opt/ros/humble/setup.bash

# Required Python packages
pip3 install numpy scipy
```

### Build the Package

```bash
# Navigate to workspace
cd ~/Desktop/smooth_nav

# Build
colcon build --packages-select robot_trajectory_generator

# Source
source install/setup.bash
```

### Verify Installation

```bash
# Check if package is available
ros2 pkg list | grep robot_trajectory_generator

# Check executables
ros2 pkg executables robot_trajectory_generator
```

---

## Quick Start

### Method 1: Complete System (Recommended)

```bash
# Terminal 1: Launch complete system
ros2 launch robot_trajectory_generator complete_navigation.launch.py

# Wait 15 seconds for simulation to start...

# Terminal 2: Publish test case
ros2 run robot_trajectory_generator test_case_publisher
# Select: 1 (Square)

# Watch the robot autonomously follow the trajectory!
```

### Method 2: Step-by-Step Launch

```bash
# Terminal 1: Launch simulator
ros2 launch robot_bringup turtlebot_sim.launch.py

# Terminal 2: Launch trajectory system
ros2 launch robot_trajectory_generator trajectory_system.launch.py

# Terminal 3: Launch velocity profiler
ros2 run robot_trajectory_generator velocity_profiler

# Terminal 4: Launch controller
ros2 run robot_trajectory_generator pure_pursuit_controller

# Terminal 5: Publish test case
ros2 run robot_trajectory_generator test_case_publisher
```

### Method 3: Manual Waypoint Entry (RViz)

```bash
# Terminal 1: Launch complete system
ros2 launch robot_trajectory_generator complete_navigation.launch.py

# In RViz:
# 1. Click "Publish Point" tool (top toolbar)
# 2. Click on grid to add waypoints
# 3. Watch trajectory generate automatically
```

---

## Topics and Services

### Published Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/waypoints` | nav_msgs/Path | waypoint_collector | Original waypoints |
| `/waypoint_markers` | visualization_msgs/MarkerArray | waypoint_collector | Waypoint visualization |
| `/smooth_trajectory` | nav_msgs/Path | catmull_rom_generator | Smooth trajectory |
| `/trajectory_markers` | visualization_msgs/MarkerArray | catmull_rom_generator | Trajectory visualization |
| `/trajectory_with_velocity` | nav_msgs/Path | velocity_profiler | Trajectory with velocities |
| `/cmd_vel` | geometry_msgs/Twist | pure_pursuit_controller | Robot velocity commands |

### Subscribed Topics

| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/clicked_point` | geometry_msgs/PointStamped | waypoint_collector | RViz clicked points |
| `/waypoints` | nav_msgs/Path | catmull_rom_generator | Input waypoints |
| `/smooth_trajectory` | nav_msgs/Path | velocity_profiler | Smooth trajectory |
| `/trajectory_with_velocity` | nav_msgs/Path | pure_pursuit_controller | Target trajectory |
| `/odom` | nav_msgs/Odometry | pure_pursuit_controller | Robot odometry |

### Services

| Service | Type | Provider | Description |
|---------|------|----------|-------------|
| `/clear_waypoints` | std_srvs/Trigger | waypoint_collector | Clear all waypoints |

---

## Test Cases

### Quick Reference

| # | Name | Difficulty | Best For |
|---|------|-----------|----------|
| 1 | Square | ⭐ Easy | Basic testing |
| 2 | Triangle | ⭐ Easy | Simple navigation |
| 3 | Circle | ⭐⭐ Medium | Smooth curves |
| 4 | Figure-8 | ⭐⭐⭐ Hard | Complex curves |
| 5 | Zigzag | ⭐⭐⭐ Hard | Sharp turns |
| 6 | Star | ⭐⭐⭐⭐ Expert | Very sharp angles |
| 7 | U-Turn | ⭐⭐ Medium | Parking maneuvers |
| 9 | Parking | ⭐⭐⭐ Hard | Real-world scenario |
| 12 | Chicane | ⭐⭐⭐⭐ Expert | Racing maneuvers |

### Testing Strategy

```bash
# Start with easy patterns
1. Square → Triangle → Circle

# Progress to medium difficulty
2. U-Turn → Corridor → Roundabout

# Challenge with hard patterns
3. Zigzag → Parking → Chicane → Star
```

---

## Usage Examples

### Example 1: Square Path with Custom Speed

```bash
ros2 launch robot_trajectory_generator complete_navigation.launch.py \
    max_linear_vel:=0.15 \
    lookahead_distance:=0.6

# In another terminal:
ros2 run robot_trajectory_generator test_case_publisher
# Select: 1
```

### Example 2: Trajectory Analysis

```bash
# Launch system
ros2 launch robot_trajectory_generator trajectory_system.launch.py

# In another terminal, run analyzer
ros2 run robot_trajectory_generator trajectory_analyzer

# Publish test case
ros2 run robot_trajectory_generator test_case_publisher
# Select: 3 (Circle)

# Analyzer will display time-parameterized trajectory
```

### Example 3: Clear and Restart

```bash
# Clear current waypoints
ros2 service call /clear_waypoints std_srvs/srv/Trigger

# Publish new test case
ros2 run robot_trajectory_generator test_case_publisher
```

### Example 4: Monitor Topics

```bash
# View waypoints
ros2 topic echo /waypoints

# View smooth trajectory (limited output)
ros2 topic echo /smooth_trajectory --once

# View velocity commands
ros2 topic echo /cmd_vel

# View trajectory with velocity
ros2 topic echo /trajectory_with_velocity --once
```

---

## Parameters

### Catmull-Rom Generator Parameters

```yaml
num_points: 200              # Trajectory resolution (higher = smoother)
alpha: 0.5                   # Parameterization (0.5 recommended)
frame_id: 'odom'            # Reference frame
auto_generate: true         # Auto-generate on new waypoints
```

### Velocity Profiler Parameters

```yaml
max_linear_vel: 0.35        # Maximum speed (m/s)
min_linear_vel: 0.08        # Minimum speed on curves (m/s)
max_angular_vel: 1.5        # Maximum turning rate (rad/s)
curvature_threshold: 0.3    # Sensitivity to curves (1/m)
lookahead_points: 5         # Points to analyze curvature
```

### Pure Pursuit Controller Parameters

```yaml
lookahead_distance: 0.5     # Base lookahead distance (m)
min_lookahead: 0.25         # Minimum lookahead (m)
max_lookahead: 0.8          # Maximum lookahead (m)
goal_tolerance: 0.12        # Distance to consider goal reached (m)
max_linear_vel: 0.35        # Maximum speed (m/s)
max_angular_vel: 1.5        # Maximum turning rate (rad/s)
use_dynamic_lookahead: true # Scale lookahead with velocity
```

---

## Algorithms

### Catmull-Rom Spline (Centripetal, α=0.5)

**Properties:**
- ✅ Passes through ALL control points
- ✅ No loop or overshoot on sharp turns
- ✅ C1 continuous (smooth velocity)
- ✅ Local control (modifying one point affects nearby segments)

**Why Centripetal (α=0.5)?**
- Better than uniform (α=0.0) - no cusps or self-intersections
- Better than chordal (α=1.0) - tighter fit to waypoints
- Best for varied curvature paths

**Formula:**
```
P(t) = A*t³ + B*t² + C*t + D
where t ∈ [0,1] between each pair of control points
```

### Velocity Profiling Algorithm

**Curvature Calculation:**
```python
curvature = |dθ/ds|
where:
  θ = heading angle
  s = arc length
```

**Velocity Assignment:**
```python
if curvature > threshold:
    velocity = min_vel + (max_vel - min_vel) * (1 - curvature/max_curvature)
else:
    velocity = max_vel
```

### Pure Pursuit Algorithm

**Lookahead Point:**
```python
L = lookahead_distance
Find point on path at distance L from robot
```

**Steering Curvature:**
```python
κ = 2 * sin(α) / L
where α = angle between robot heading and lookahead point
```

**Velocity Commands:**
```python
linear_vel = desired_velocity_at_point
angular_vel = linear_vel * κ
```

---

## Troubleshooting

### No trajectory generated

**Problem:** Waypoints collected but no smooth trajectory appears

**Solutions:**
```bash
# Check if catmull_rom_generator is running
ros2 node list | grep catmull_rom

# Check waypoints topic
ros2 topic echo /waypoints --once

# Need at least 2 waypoints
ros2 service call /clear_waypoints std_srvs/srv/Trigger
# Add more waypoints
```

### Robot not moving

**Problem:** Trajectory visible but robot doesn't move

**Solutions:**
```bash
# Check if controller is running
ros2 node list | grep pure_pursuit

# Check if trajectory has velocity
ros2 topic echo /trajectory_with_velocity --once

# Check cmd_vel is being published
ros2 topic hz /cmd_vel

# Restart controller
ros2 run robot_trajectory_generator pure_pursuit_controller
```

### Robot overshoots corners

**Problem:** Robot doesn't follow path accurately

**Solutions:**
```bash
# Reduce lookahead distance
ros2 run robot_trajectory_generator pure_pursuit_controller \
    --ros-args -p lookahead_distance:=0.3

# Reduce maximum velocity
ros2 run robot_trajectory_generator pure_pursuit_controller \
    --ros-args -p max_linear_vel:=0.15
```

### Trajectory too jagged

**Problem:** Generated path not smooth enough

**Solutions:**
```bash
# Increase number of points
ros2 run robot_trajectory_generator catmull_rom_generator \
    --ros-args -p num_points:=300

# Already using centripetal (α=0.5), which is best
```

### Test case publisher not working

**Problem:** Cannot select test cases

**Solutions:**
```bash
# Ensure waypoint_collector is running
ros2 node list | grep waypoint_collector

# Check /clicked_point topic exists
ros2 topic list | grep clicked_point

# Restart test case publisher
ros2 run robot_trajectory_generator test_case_publisher
```

---

## Performance Tips

### For Smooth Motion
- Use `lookahead_distance` between 0.3-0.7 m
- Keep `max_linear_vel` ≤ 0.35 m/s for TurtleBot3
- Enable `use_dynamic_lookahead`

### For Fast Execution
- Increase `max_linear_vel` to 0.5 m/s (if robot supports)
- Increase `lookahead_distance` to 0.8 m
- Reduce `num_points` to 100

### For Tight Corners
- Reduce `min_linear_vel` to 0.05 m/s
- Increase `curvature_threshold` to 0.5
- Reduce `lookahead_distance` to 0.3 m

---

## Integration with Other Packages

### With `robot_bringup`
```bash
# Simulation environment
ros2 launch robot_bringup turtlebot_sim.launch.py
```

### With Navigation Stack (Future)
- Can publish to `/goal_pose` for Nav2 integration
- Can subscribe to `/plan` from Nav2 planner

---

## RViz Visualization Setup

### Required Displays

1. **Path** → Topic: `/waypoints` → Color: Green
2. **Path** → Topic: `/smooth_trajectory` → Color: Magenta
3. **MarkerArray** → Topic: `/waypoint_markers`
4. **MarkerArray** → Topic: `/trajectory_markers`
5. **RobotModel** → Description Topic: `/robot_description`
6. **TF** → Show all frames

### Fixed Frame
Set to: `odom`

---

## Contributing

To extend functionality:

1. **Add new trajectory generators** - Create new node in `robot_trajectory_generator/`
2. **Add new controllers** - Implement alternative control algorithms
3. **Add new test cases** - Update `test_case_publisher.py`

---

## Related Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture diagrams
- [COMPLETE_SYSTEM_GUIDE.md](COMPLETE_SYSTEM_GUIDE.md) - Detailed usage guide
- [README.md](README.md) - Quick start guide

---

## Dependencies

### ROS 2 Packages
- `rclpy` - ROS 2 Python client library
- `geometry_msgs` - Geometry message types
- `nav_msgs` - Navigation message types
- `visualization_msgs` - Visualization markers
- `std_srvs` - Standard services

### Python Packages
- `numpy` - Numerical computations
- `scipy` - Scientific computing (for interpolation)

---

## License

Apache-2.0

---

## Authors

**Sai Eswar Amurali**  
Autonomous Navigation System - Path Smoothing and Trajectory Control

**Last Updated:** October 2025

---

## Version History

- **v1.0.0** (Oct 2025) - Complete system with all 4 nodes + test cases
- **v0.3.0** (Oct 2025) - Added Pure Pursuit controller
- **v0.2.0** (Oct 2025) - Added Velocity Profiler
- **v0.1.0** (Oct 2025) - Initial release with waypoint collector and Catmull-Rom generator
