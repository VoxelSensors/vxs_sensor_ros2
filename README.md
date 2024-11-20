# ROS2 Package for the VXS Sensor

A ROS2 package for publishing sensor data in various forms (depth imagbe, pointcloud, event stream, etc.).

## ROS2 Workspace setup \& Build

### Setting up the workspace directories

First, create the appropriate workspace directories (if not there yet; if present, skip to the build section). In the home directory of the Ubuntu host (or someplace else if on Windows), 

``cd ~``

Create a **sandbox** directory that will be shared netween the host and the ROS2 docker container. It will be useful for storing data.

``mkdir sandbox``

Now create the **vxs  workspace** directory:

``mkdir vxs_ws && cd vxs_ws``

Create the **ROS2 workspace** below:

``mkdir ros_ws && cd ros_ws``

``mkdir src``

### Download ROS2 packages that must be built from source, including the [vxs_sensor_ros2](https://github.com/VoxelSensors/vxs_ros_workspace_install) package

To setup the workspace, execute the steps described in the README of the workspace setup repository, [here](https://github.com/VoxelSensors/vxs_ros_workspace_install).

The `rosinstall` script should have installed all repositories correctly in the `ros_ws/src`. To build everything, execute the following inside `ros_ws`:

``colcon build``

Alternatively, provided that the ROS2 packages are built, then the `vx_sensor_ros2` package can be specifically built,

``colcon build --packages-select vxs_sensor_ros2``

## Running the node

To run the `vxs_node` connecft the sensor. You need to enable access to the USB:

``sudo chmod -R 7777 /dev/bus/usb/``

Now run the node with the following:

``ros2 run vxs_sensor_ros2 vxs_node --ros-args -p "config_json:=/home/vxs/vxs_ws/ros_ws/src/vxs_sensor_ros2/config/and2_median_golden.json" -p "calib_json:=/home/vxs/vxs_ws/ros_ws/src/vxs_sensor_ros2/config/default_calib.json" -p "fps:=20"``

You can now start a new docker window and get observe the data in the ros topics published by the node (`/depth/image` and `/depth/camera_info`):

``ro2 topic list``

You should see something like,

![image](https://github.com/user-attachments/assets/1dd4a3a1-e3e3-4cdb-a967-a2315cd96a2e)

Note that the depth image is published as a 16-bit integer image. You can display that image with,

``ros2 run image_view image_view --ros-args --remap image:=/depth/image``

Another way is to print the cotents of topics, e.g.,

``ros2 topic echo /depth/camera_info``

or,

``ros2 topic echo /depth/image``

