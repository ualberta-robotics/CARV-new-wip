#!/bin/bash
rm -rf debug/
rm -rf export_dataset/

# Source ROS
source /opt/ros/humble/setup.bash

# ACTIVATE THE VENV (Crucial Meta Prerequisite)
source /opt/aria_venv/bin/activate

# Source the workspace
source install/setup.bash

# Launch
ros2 launch quest3carv quest3carv.launch.py