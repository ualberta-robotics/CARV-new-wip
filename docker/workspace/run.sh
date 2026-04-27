set -e

#rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
# Build the message package first so the C++ and Python nodes can find the header/module
colcon build --packages-select quest3carv_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the new message overlay
source install/setup.bash

# Build the rest
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# remove debug output
rm -rf debug/
rm -rf export_dataset/

# debug: print topics in background after startup
(sleep 5 && ros2 topic list) &

# run
ros2 launch quest3carv quest3carv.launch.py

# keep alive
/bin/bash