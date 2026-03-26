# ROS 2 Package for the VXS Sensor

A ROS 2 package for publishing sensor data in various forms (depth image, pointcloud, event stream, etc.) from VoxelSensors hardware. 

---

## Getting Started

### 1. Prerequisites
You need [Docker](https://docs.docker.com/engine/install/) installed on your host machine (Linux or Windows). 

### 2. Clone the Repository

Because this project links to the `VXSDK2` via a Git Submodule, you **must** use the `--recursive` flag when cloning so it downloads the SDK source code simultaneously:

```bash
git clone --recursive git@github.com:VoxelSensors/vxs_sensor_ros2.git
cd vxs_sensor_ros2
```

### 3. Boot the Environment
Navigate to the infrastructure folder and launch the container. This will automatically pull ROS 2 Humble, compile the Livox SDK, and compile the VXSDK natively.
Navigate to the docker/ folder and build it. This will pull ROS2 Humble, and compile the libraries that you specify in the Dockerfile.

```bash
cd docker
xhost +local:  # Allows GUI to pass through to your screen
docker compose up -d --build
```
*(Note: Building the image will take a few minutes the very first time it is run).*

### 4. Step Inside and Compile
Enter the running container:
```bash
docker compose exec vxs_env bash
```
You should drop directly into the ROS2 workspace (`/home/vxs/ros2_ws`). Compile your package:
```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Repository Architecture & Data Sandbox

This repository uses a clean volume-mapping architecture:
* **Code:** Edit Python/C++ files on your host machine. The Docker container sees the changes instantly, and any "build artifacts" (`build/`, `install/`) are trapped inside the container.
* **Data Sandbox** (`bags/` & `rviz_configs/`): Shared space between your laptop and the container. Anything you save here stays permanently on your laptop. Git ignores these folders!

---

## Running the Node

To run the `vxs_node`, connect the sensor to your PC. 

*(Troubleshooting: If the sensor is not detected via USB, you may need to grant USB bus access on your host machine: `sudo chmod -R 7777 /dev/bus/usb/`)*

Run the node with the default configuration files (paths are already mapped inside the container):

```bash
ros2 run vxs_sensor_ros2 vxs_node --ros-args \
  -p "config_json:=/home/vxs/ros2_ws/src/vxs_sensor_ros2/config/and2_median_golden.json" \
  -p "calib_json:=/home/vxs/ros2_ws/src/vxs_sensor_ros2/config/and2_125_5mm.json" \
  -p "fps:=20"
```

### Viewing the Data
Open a second terminal, enter the container (`docker compose exec vxs_env bash`).

You can now start a new docker window and get observe the data in the ros topics published by the node (`/depth/image` and `/depth/camera_info`):

``ros2 topic list``

You should see something like this:

![image](https://github.com/user-attachments/assets/1dd4a3a1-e3e3-4cdb-a967-a2315cd96a2e)

---

### ROS publisher node (vxs_node) arguments

- **publish_depth_image (bool)** : Will work only in **frame-based** communications mode with the sensor and publish a *depth image* in topic `depth/image`. Will be overriden (forced **false**) if **publish_events** is set to **true**.
- **publish_pointcloud (bool)**  : Will work only in **frame-based** communications with the sensor and publish a *pointcloud* in topic `pcloud/cloud`. As above with `publish_depth_image`, it will be overriden (forced **false**) if **publish_events** is set to **true**.
- **publish_events (bool)**      : Set this argument to **true** to force the node to initialize communications in **streaming mode** with the sensor. In this communications mode, the nose will publish a **stamped pointcloud** which will represent events (`XYZt`) in 3D space and time between two time instances defined by a a period `1000/fps (ms)` (see below about argument **fps**). Will override `publish_depth_image` and `publish_pointcloud`.     
- **publish_imu**                : If set, IMU samples from the sensor will be published. The flag can be set only when using streaming mode (i.e., publish_events = true).
- **fps (int)**                  : If using **frame-based mode** (see first two arguments), it specifies the frame-rate. For frame-based mode, then **valid fps values are 1, 15, 30, 60, 90, 180**. Otherwise, if the node is on **streaming mode**, then **fps** can have any positive value and will determine the **period throughout which it will capture events (i.e. `XYZt` data).
- **config_json (string)**       : The full path to the SDK configuration json.
- **calib_json (string)**        : The full path to the calibration json.
- **sleep_time_ms (int)**        : Time to set the polling thread to sleep while waiting for a new frame/batch of events.
- **binning_amount (int)**              : (Filtering arg. 1). Default: 0
- **prefiltering_threshold (float)**    : (Filtering arg. 2). Default: 2.0
- **postfiltering_threshold (int)**     : (Filtering arg. 3). Default: 5
- **filterP1X (float)**                 : (Filtering arg. 4). Default: 0.1
- **filterP1X (float)**                 : (Filtering arg. 5). Default: 0.1
- **temporal_threshold (int)**          : (Filtering arg. 6). Default: 4
- **spatial_threshold (int)**           : (Filtering arg. 7). Default: 10
- **median_rejection_threshold (int)**  : (Filtering arg. 8). Default: 5
- **observation_window_on_time (int)**      : Observation window `on_time` in nanoseconds (e.g., 100)
- **observation_window_perior_time (int)**  : Observation window `period_time` in nanoseconds. usually double the `on_time` (e.g., 200)
- **sleep_time_ms (int)**               : Sleep time for the mainloop thread when waiting for data in ms. Default: 1

**NOTE**: If none of the three first arguments that determine sensor communication mode are set, then the node will internally set **publish_pointcloud** to **true**.
