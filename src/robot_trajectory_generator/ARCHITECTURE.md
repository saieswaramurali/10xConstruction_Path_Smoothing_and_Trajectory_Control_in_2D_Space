# Waypoint Collection System - Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         RViz Interface                          │
│                                                                 │
│  ┌──────────────────┐                                          │
│  │  User clicks     │                                          │
│  │  "Publish Point" │                                          │
│  │  tool and clicks │                                          │
│  │  on the grid     │                                          │
│  └────────┬─────────┘                                          │
└───────────┼──────────────────────────────────────────────────────┘
            │
            │ publishes to
            │
            ▼
    /clicked_point
    (geometry_msgs/PointStamped)
            │
            │ subscribes
            │
            ▼
┌─────────────────────────────────────────────────────────────────┐
│              Waypoint Collector Node                            │
│              (robot_trajectory_generator)                       │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  • Receives clicked points                               │  │
│  │  • Stores waypoints in list                              │  │
│  │  • Converts to Path message                              │  │
│  │  • Creates visualization markers                         │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│  Storage: waypoints = [(x1,y1,z1), (x2,y2,z2), ...]           │
└───────────┬───────────────────────────────┬─────────────────────┘
            │                               │
            │ publishes                     │ publishes
            │                               │
            ▼                               ▼
      /waypoints                    /waypoint_markers
    (nav_msgs/Path)              (visualization_msgs/
                                    MarkerArray)
            │                               │
            │                               │
            │                               └──────────────┐
            │                                              │
            ▼                                              ▼
┌─────────────────────────────────┐           ┌──────────────────┐
│  Your Trajectory Generator      │           │   RViz Display   │
│  (Future Implementation)        │           │                  │
│                                 │           │  Shows:          │
│  • Subscribe to /waypoints      │           │  • Numbered dots │
│  • Generate smooth trajectory   │           │  • Colored path  │
│  • Send cmd_vel commands        │           │  • Blue line     │
│  • Control robot motion         │           │                  │
└─────────────────────────────────┘           └──────────────────┘


                    Services Available:
                    ═══════════════════
                    
        /clear_waypoints (std_srvs/Trigger)
                    │
                    │ clears all
                    │
                    ▼
        ┌───────────────────────┐
        │ Waypoint Collector    │
        │ waypoints = []        │
        └───────────────────────┘
```

## Data Flow

### 1. User Interaction
```
User clicks in RViz → RViz publishes PointStamped → /clicked_point topic
```

### 2. Waypoint Collection
```
/clicked_point → Waypoint Collector Node → Stores in list
                                         → Publishes Path
                                         → Creates Markers
```

### 3. Visualization
```
/waypoint_markers → RViz → Display colored spheres, numbers, lines
```

### 4. Data Usage
```
/waypoints → Your trajectory node → Generate smooth path → cmd_vel
```

## Message Types

### Input: PointStamped
```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z
```

### Output: Path
```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id
geometry_msgs/PoseStamped[] poses
  std_msgs/Header header
  geometry_msgs/Pose pose
    geometry_msgs/Point position (x, y, z)
    geometry_msgs/Quaternion orientation (x, y, z, w)
```

## Node Communication Diagram

```
       RViz                Waypoint              Your
     Interface            Collector             Code
        │                    │                    │
        │  /clicked_point    │                    │
        ├───────────────────>│                    │
        │                    │                    │
        │                    │ process            │
        │                    │ & store            │
        │                    │                    │
        │  /waypoint_markers │                    │
        │<───────────────────┤                    │
        │                    │                    │
        │                    │  /waypoints        │
        │                    ├───────────────────>│
        │                    │                    │
        │                    │                    │ generate
        │                    │                    │ trajectory
        │                    │                    │
```

## Integration with Your Assignment

```
Step 1: Collect Waypoints
    RViz clicks → Waypoint Collector → Store points

Step 2: Generate Trajectory (YOUR CODE HERE)
    Waypoints → Smooth curve generation → Trajectory points
    
    Algorithms you can use:
    • Bezier curves
    • Cubic splines
    • B-splines
    • Minimum jerk trajectory

Step 3: Execute Motion
    Trajectory points → Path following controller → cmd_vel
    
    Controllers you can use:
    • Pure Pursuit
    • Stanley Controller
    • MPC (Model Predictive Control)
    • PID controller

Step 4: Visualization
    Planned path → RViz markers → Verify smooth motion
```

## File Structure

```
robot_trajectory_generator/
├── robot_trajectory_generator/
│   ├── __init__.py
│   ├── waypoint_collector.py          ← Main waypoint collection node
│   └── trajectory_generator_example.py ← Example subscriber
├── launch/
│   └── waypoint_collector.launch.py   ← Easy launch file
├── package.xml                         ← ROS 2 dependencies
├── setup.py                            ← Python package setup
├── setup.cfg
├── README.md                           ← Full documentation
├── WAYPOINT_SETUP_COMPLETE.md         ← Quick start guide
└── build_and_info.sh                  ← Build helper script
```
