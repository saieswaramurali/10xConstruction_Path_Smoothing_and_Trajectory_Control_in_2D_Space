#!/bin/bash

# Complete Trajectory System Launcher
# Opens 3 separate terminals for easy system management

WORKSPACE_DIR="$HOME/Desktop/smooth_nav"

echo "═══════════════════════════════════════════════════════════════"
echo "  🚀 Launching Complete Trajectory System 🚀"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Opening 3 terminals:"
echo "  Terminal 1: Robot Simulator (Gazebo + RViz)"
echo "  Terminal 2: Trajectory System (Waypoint + Generator)"  
echo "  Terminal 3: Shape Selector ⭐ YOU USE THIS ONE!"
echo ""
echo "═══════════════════════════════════════════════════════════════"
sleep 2

# Terminal 1: Robot Simulator
x-terminal-emulator -T "Terminal 1: Robot Simulator" -e bash -c "\
    source $WORKSPACE_DIR/install/setup.bash; \
    echo ''; \
    echo '═══════════════════════════════════════════════════════════'; \
    echo '  Terminal 1: Robot Simulator'; \
    echo '═══════════════════════════════════════════════════════════'; \
    echo ''; \
    echo 'Starting Gazebo + RViz...'; \
    echo ''; \
    ros2 launch robot_bringup turtlebot_sim.launch.py model:=burger; \
    exec bash" &

# Wait a bit before launching trajectory system
sleep 5

# Terminal 2: Trajectory System  
x-terminal-emulator -T "Terminal 2: Trajectory System" -e bash -c "\
    source $WORKSPACE_DIR/install/setup.bash; \
    echo ''; \
    echo '═══════════════════════════════════════════════════════════'; \
    echo '  Terminal 2: Trajectory System'; \
    echo '═══════════════════════════════════════════════════════════'; \
    echo ''; \
    echo 'Starting Waypoint Collector + Catmull-Rom Generator...'; \
    echo ''; \
    ros2 launch robot_trajectory_generator trajectory_system.launch.py; \
    exec bash" &

# Wait a bit before launching test case publisher
sleep 3

# Terminal 3: Test Case Publisher (Interactive)
x-terminal-emulator -T "Terminal 3: Shape Selector ⭐ USE THIS!" -e bash -c "\
    source $WORKSPACE_DIR/install/setup.bash; \
    echo ''; \
    echo '╔═══════════════════════════════════════════════════════════╗'; \
    echo '║                                                           ║'; \
    echo '║     🎯 SHAPE SELECTOR - USE THIS TERMINAL! 🎯             ║'; \
    echo '║                                                           ║'; \
    echo '╚═══════════════════════════════════════════════════════════╝'; \
    echo ''; \
    echo 'Waiting for trajectory system to initialize...'; \
    sleep 5; \
    echo ''; \
    echo '✅ System Ready!'; \
    echo ''; \
    echo 'Now you can select shapes:'; \
    echo '  - Type a number (1-19) for test case'; \
    echo '  - Add \"m\" to mirror (e.g., 1m)'; \
    echo '  - Type 0 to exit'; \
    echo ''; \
    ros2 run robot_trajectory_generator test_case_publisher; \
    exec bash" &

echo ""
echo "✅ All terminals launched!"
echo ""
echo "📋 Quick Guide:"
echo "  1. Wait ~10 seconds for everything to start"
echo "  2. Use Terminal 3 to select shapes"
echo "  3. Type '1' for square, '1m' for mirrored square"
echo "  4. Watch the magic in RViz!"
echo ""
echo "═══════════════════════════════════════════════════════════════"
