# Robot Trajectory Generator - Waypoint Collector

This package provides a complete trajectory generation system for robot navigation using RViz and Catmull-Rom splines.

## Features

- **Subscribe to RViz clicked points** (`/clicked_point`)
- **19 Predefined test cases** with interactive selection
- **Y-axis mirroring** to flip paths (left↔right)
- **Catmull-Rom trajectory generation** (best for sharp turns)
- **Visualize waypoints** in RViz with numbered markers
- **Publish smooth trajectories** for robot navigation
- **Single command launch** - opens everything in separate terminals

## 🚀 Super Quick Start

### 1. Build the package (one time only)

```bash
cd ~/Desktop/smooth_nav
colcon build --packages-select robot_trajectory_generator
source install/setup.bash
```

### 2. Launch everything with ONE command

```bash
ros2 launch robot_trajectory_generator complete_system.launch.py
```

This opens 3 terminals automatically:
- **Terminal 1:** Robot Simulator (Gazebo + RViz)
- **Terminal 2:** Trajectory System (Waypoint Collector + Catmull-Rom Generator)
- **Terminal 3:** Shape Selector ⭐ **USE THIS ONE!**

### 3. Select a shape

In **Terminal 3**, type a number:

```
1  - Square (start with this!)
2  - Triangle
3  - Circle
4  - Figure-8
5  - Zigzag
...and 14 more!
```

**Want to mirror?** Add `m` after the number:
- `1` = Normal square (turns left)
- `1m` = Mirrored square (turns right!)

## Test Cases Available

1. Open RViz (if not already running)
2. Add the following displays:
   - **Path**: Set topic to `/waypoints`
   - **MarkerArray**: Set topic to `/waypoint_markers`
3. Make sure the Fixed Frame is set to `odom` or `map`

### 4. Add waypoints

1. Click the **"Publish Point"** button in the RViz toolbar (top menu)
2. Click anywhere on the grid/map to add waypoints
3. Each click will add a new waypoint with a numbered marker

## Topics

### Subscribed Topics

- `/clicked_point` (geometry_msgs/PointStamped)
  - RViz publishes to this when you use the "Publish Point" tool

### Published Topics

- `/waypoints` (nav_msgs/Path)
  - All collected waypoints as a path
  - Updates every 0.5 seconds

- `/waypoint_markers` (visualization_msgs/MarkerArray)
  - Visualization markers for RViz
  - Shows numbered spheres and connecting lines

## Services

- `/clear_waypoints` (std_srvs/Trigger)
  - Clears all collected waypoints

```bash
ros2 service call /clear_waypoints std_srvs/srv/Trigger
```

## Parameters

- `frame_id` (string, default: "odom")
  - Reference frame for waypoints
  
- `max_waypoints` (int, default: 100)
  - Maximum number of waypoints to collect
  
- `default_z` (float, default: 0.0)
  - Default z-coordinate for waypoints

### Example with parameters:

```bash
ros2 launch robot_trajectory_generator waypoint_collector.launch.py frame_id:=map max_waypoints:=50
```

## Usage Example

```bash
# Terminal 1: Launch the waypoint collector
ros2 launch robot_trajectory_generator waypoint_collector.launch.py

# Terminal 2: View the collected waypoints
ros2 topic echo /waypoints

# Terminal 3: Clear waypoints when needed
ros2 service call /clear_waypoints std_srvs/srv/Trigger
```

## Visualization

The waypoint markers use a color gradient:
- **Green**: First waypoint
- **Yellow-Orange**: Middle waypoints
- **Red**: Last waypoint
- **Blue line**: Connects waypoints in order
- **White numbers**: Waypoint sequence numbers

## Next Steps

After collecting waypoints, you can:
1. Subscribe to `/waypoints` in your trajectory generation node
2. Process the waypoints to generate smooth trajectories
3. Send trajectories to your robot controller
