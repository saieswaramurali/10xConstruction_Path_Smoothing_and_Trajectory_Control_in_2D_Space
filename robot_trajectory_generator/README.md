# Robot Trajectory Generator - Waypoint Collector

This package provides a waypoint collection system for robot trajectory generation using RViz.

## Features

- **Subscribe to RViz clicked points** (`/clicked_point`)
- **Visualize waypoints** in RViz with numbered markers
- **Publish waypoints** as a Path message (`/waypoints`)
- **Clear waypoints** via service call

## Quick Start

### 1. Build the package

```bash
cd /home/sai/Desktop/smooth_nav
colcon build --packages-select robot_trajectory_generator
source install/setup.bash
```

### 2. Launch the waypoint collector

```bash
ros2 launch robot_trajectory_generator waypoint_collector.launch.py
```

Or run the node directly:

```bash
ros2 run robot_trajectory_generator waypoint_collector
```

### 3. Set up RViz

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
