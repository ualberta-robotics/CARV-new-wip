#!/bin/bash
rm -rf build/ install/ log/

# Source ROS
source /opt/ros/humble/setup.bash

# ACTIVATE THE VENV (Crucial Meta Prerequisite)
source /opt/aria_venv/bin/activate

# Build interfaces first (like Meta does with aria_data_types)
colcon build --packages-select quest3carv_interfaces
source install/setup.bash

export PYTHONPATH=/opt/aria_venv/lib/python3.10/site-packages:$PYTHONPATH

# Build the rest
colcon build