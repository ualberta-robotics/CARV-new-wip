# Start with ROS 2 Humble (Ubuntu 22.04)
FROM ros:humble-ros-base

# Install system dependencies for OpenCV and FFmpeg
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv\
    libgl1-mesa-glx \
    libglib2.0-0 \
    netcat nano\
    ros-humble-cv-bridge ros-humble-message-filters ros-humble-image-transport \
    libcgal-dev libeigen3-dev libboost-dev libboost-serialization-dev libboost-graph-dev\
    && rm -rf /var/lib/apt/lists/*

# Create the virtual environment
RUN python3 -m venv --system-site-packages /opt/aria_venv

# Add it to the path so pip uses it
ENV PATH="/opt/aria_venv/bin:$PATH"

# Install everything INTO the venv
RUN pip install --upgrade pip setuptools wheel
RUN pip install --no-cache-dir "numpy<2.0" opencv-python scipy projectaria-tools'[all]'==2.1.0 projectaria-client-sdk==2.3.0 colcon-common-extensions

# Setup the workspace directory
WORKDIR /workspace

# Source ROS 2 automatically when opening a bash shell in the container
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
