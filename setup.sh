#!/bin/bash

##############################################################################
# Autonomous Navigation System - Setup Script
##############################################################################
#
# Description:
#   This script automates the complete setup process for the autonomous
#   navigation system including dependency installation, workspace building,
#   and environment configuration.
#
# Usage:
#   chmod +x setup.sh
#   ./setup.sh
#
# Requirements:
#   - Ubuntu 22.04
#   - ROS 2 Humble
#   - Python 3.10+
#
##############################################################################

set -e  # Exit on error

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "================================================================================"
echo "  Autonomous Navigation System - Setup"
echo "================================================================================"
echo ""
echo "Workspace: $WORKSPACE_DIR"
echo ""

# Function to print colored status messages
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if ROS 2 Humble is installed
print_status "Checking ROS 2 Humble installation..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    print_status "ROS 2 Humble found and sourced"
else
    print_error "ROS 2 Humble not found!"
    print_error "Please install ROS 2 Humble first:"
    print_error "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    exit 1
fi

# Check Python version
print_status "Checking Python version..."
PYTHON_VERSION=$(python3 --version | awk '{print $2}')
print_status "Python version: $PYTHON_VERSION"

# Initialize rosdep if not already initialized
print_status "Checking rosdep initialization..."
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    print_warning "rosdep not initialized. Initializing now..."
    sudo rosdep init
    print_status "rosdep initialized"
else
    print_status "rosdep already initialized"
fi

# Update rosdep
print_status "Updating rosdep..."
rosdep update

# Install dependencies using rosdep
print_status "Installing workspace dependencies..."
cd "$WORKSPACE_DIR"
rosdep install --from-paths src --ignore-src -r -y

# Install Python dependencies
print_status "Installing Python dependencies..."
pip3 install --upgrade numpy scipy

# Check for Gazebo
print_status "Checking Gazebo installation..."
if ! command -v gazebo &> /dev/null; then
    print_warning "Gazebo not found. Installing gazebo..."
    sudo apt-get update
    sudo apt-get install -y gazebo ros-humble-gazebo-ros-pkgs
fi

# Install TurtleBot3 packages if not present
print_status "Checking TurtleBot3 packages..."
if ! ros2 pkg list | grep -q "turtlebot3_gazebo"; then
    print_warning "TurtleBot3 packages not found in system. Using workspace version..."
fi

# Export TurtleBot3 model
print_status "Setting TurtleBot3 model to 'burger'..."
if ! grep -q "export TURTLEBOT3_MODEL=burger" ~/.bashrc; then
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    print_status "Added TURTLEBOT3_MODEL to ~/.bashrc"
fi
export TURTLEBOT3_MODEL=burger

# Build the workspace
print_status "Building workspace..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install

if [ $? -eq 0 ]; then
    print_status "Workspace built successfully"
else
    print_error "Build failed!"
    exit 1
fi

# Source the workspace
print_status "Sourcing workspace..."
source "$WORKSPACE_DIR/install/setup.bash"

# Create convenience alias
print_status "Setting up convenience commands..."
if ! grep -q "alias smooth_nav=" ~/.bashrc; then
    echo "alias smooth_nav='cd $WORKSPACE_DIR && source install/setup.bash'" >> ~/.bashrc
    print_status "Added 'smooth_nav' alias to ~/.bashrc"
fi

echo ""
echo "================================================================================"
echo "  Setup Complete!"
echo "================================================================================"
echo ""
echo "Next steps:"
echo ""
echo "  1. Source your workspace:"
echo "     source ~/.bashrc"
echo "     cd $WORKSPACE_DIR"
echo "     source install/setup.bash"
echo ""
echo "  2. Launch the navigation system (2 terminals required):"
echo ""
echo "     Terminal 1:"
echo "     ros2 launch robot_trajectory_generator complete_navigation.launch.py"
echo ""
echo "     Terminal 2:"
echo "     ros2 run robot_trajectory_generator test_case_publisher"
echo ""
echo "  Or use the convenience command:"
echo "     smooth_nav"
echo ""
echo "================================================================================"
echo ""
