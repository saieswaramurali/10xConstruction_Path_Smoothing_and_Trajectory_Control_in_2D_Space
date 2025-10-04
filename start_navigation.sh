#!/bin/bash

# Complete Navigation System Launcher
# Launches all required nodes in separate terminals

WORKSPACE_DIR="$HOME/Desktop/smooth_nav"

echo "==============================================================="
echo "  [LAUNCH] Launching Complete Navigation System"
echo "==============================================================="
echo ""
echo "Opening 5 terminals:"
echo "  Terminal 1: Robot Simulator (Gazebo + RViz)"
echo "  Terminal 2: Trajectory System (Waypoint + Catmull-Rom)"
echo "  Terminal 3: Velocity Profiler (Curvature-based)"
echo "  Terminal 4: Pure Pursuit Controller (Path Following)"
echo "  Terminal 5: Test Case Publisher [*] YOU USE THIS ONE!"
echo ""
echo "==============================================================="
sleep 2

# Terminal 1: Robot Simulator
x-terminal-emulator -T "Terminal 1: Robot Simulator" -e bash -c "\
    cd $WORKSPACE_DIR; \
    source install/setup.bash; \
    echo ''; \
    echo '==============================================================='; \
    echo '  Terminal 1: Robot Simulator'; \
    echo '==============================================================='; \
    echo ''; \
    echo 'Starting Gazebo + RViz...'; \
    echo ''; \
    ros2 launch robot_bringup turtlebot_sim.launch.py; \
    exec bash" &

# Wait for simulator to start
sleep 5

# Terminal 2: Trajectory System
x-terminal-emulator -T "Terminal 2: Trajectory System" -e bash -c "\
    cd $WORKSPACE_DIR; \
    source install/setup.bash; \
    echo ''; \
    echo '==============================================================='; \
    echo '  Terminal 2: Trajectory System'; \
    echo '==============================================================='; \
    echo ''; \
    echo 'Starting Waypoint Collector + Catmull-Rom Generator...'; \
    echo ''; \
    ros2 launch robot_trajectory_generator trajectory_system.launch.py; \
    exec bash" &

# Wait a bit
sleep 2

# Terminal 3: Velocity Profiler
x-terminal-emulator -T "Terminal 3: Velocity Profiler" -e bash -c "\
    cd $WORKSPACE_DIR; \
    source install/setup.bash; \
    echo ''; \
    echo '==============================================================='; \
    echo '  Terminal 3: Velocity Profiler'; \
    echo '==============================================================='; \
    echo ''; \
    echo 'Starting Velocity Profiler (Curvature-based)...'; \
    echo ''; \
    ros2 run robot_trajectory_generator velocity_profiler; \
    exec bash" &

# Wait a bit
sleep 2

# Terminal 4: Pure Pursuit Controller
x-terminal-emulator -T "Terminal 4: Pure Pursuit Controller" -e bash -c "\
    cd $WORKSPACE_DIR; \
    source install/setup.bash; \
    echo ''; \
    echo '==============================================================='; \
    echo '  Terminal 4: Pure Pursuit Controller'; \
    echo '==============================================================='; \
    echo ''; \
    echo 'Starting Pure Pursuit Controller...'; \
    echo ''; \
    ros2 run robot_trajectory_generator pure_pursuit_controller; \
    exec bash" &

# Wait a bit
sleep 2

# Terminal 5: Test Case Publisher (Interactive - User Terminal)
x-terminal-emulator -T "Terminal 5: Test Case Publisher ⭐ USE THIS!" -e bash -c "\
    cd $WORKSPACE_DIR; \
    source install/setup.bash; \
    echo ''; \
    echo '╔═══════════════════════════════════════════════════════════╗'; \
    echo '║                                                           ║'; \
    echo '║     🎯 TEST CASE PUBLISHER - USE THIS TERMINAL! 🎯        ║'; \
    echo '║                                                           ║'; \
    echo '╚═══════════════════════════════════════════════════════════╝'; \
    echo ''; \
    echo 'Waiting for all systems to initialize...'; \
    sleep 3; \
    echo ''; \
    echo '✅ System Ready!'; \
    echo ''; \
    echo 'Instructions:'; \
    echo '  1. Type a number (1-19) to select a test case'; \
    echo '  2. Add \"m\" to mirror (e.g., 1m for mirrored square)'; \
    echo '  3. Type 20 for manual waypoint entry'; \
    echo '  4. Watch the robot move in Gazebo!'; \
    echo ''; \
    ros2 run robot_trajectory_generator test_case_publisher; \
    exec bash" &

echo ""
echo "✅ All terminals launched!"
echo ""
echo "📋 Quick Guide:"
echo "  1. Wait ~15 seconds for everything to start"
echo "  2. Use Terminal 5 to select shapes"
echo "  3. Type '1' for square, '6' for star, '20' for manual"
echo "  4. Watch the robot follow the path autonomously!"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "💡 To stop all: Close all terminals or press Ctrl+C in each"
echo ""
