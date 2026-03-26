# 1. Base Image: Includes ROS 2 and RViz natively!
FROM osrf/ros:humble-desktop

# 2. System & Network Tools (Crucial for Livox Ethernet)
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    iputils-ping \
    net-tools \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 3. Research Pipeline Dependencies (OctoMap & PCL)
RUN apt-get update && apt-get install -y \
    ros-humble-octomap \
    ros-humble-octomap-server \
    ros-humble-octomap-rviz-plugins \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# 4. Python Data Science Libraries
RUN pip3 install numpy scipy open3d

# 5. Livox SDK2 (The core C++ driver for the Mid-360)
WORKDIR /opt
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /opt/Livox-SDK2/build
RUN cmake .. && make -j$(nproc) && make install

# 6. VoxelSensors SDK (VXSDK2)
# Copy the local folder into the container's /opt directory
COPY VXSDK2 /opt/VXSDK2
WORKDIR /opt/VXSDK2
# Make a fresh build folder and compile it system-wide
RUN mkdir build && cd build && cmake .. && make -j$(nproc) && make install

# 7. Workspace Setup
# Set the default working directory inside the container
RUN mkdir -p /home/vxs/ros2_ws/src
WORKDIR /home/vxs/ros2_ws

# 8. Automatic Sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /home/vxs/ros2_ws/install/setup.bash ]; then source /home/vxs/ros2_ws/install/setup.bash; fi" >> ~/.bashrc