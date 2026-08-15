# handsfree_ros2_imu

**ROS2 port of [HANDS-FREE/handsfree_ros_imu](https://github.com/HANDS-FREE/handsfree_ros_imu)**

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-darkred)](https://docs.ros.org/en/foxy/index.html)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)

This package provides ROS2 support for HandsFree IMU devices, allowing you to publish IMU and magnetometer data to ROS2 topics. It is based on the original ROS1 package and adapted for ROS2 Python nodes and launch files. For more details, visit the [GitHub repository](https://github.com/3bdul1ah/handsfree_ros2_imu) and [official documentation](https://docs.taobotics.com/docs/hfi-imu/).

## Features

- Publishes IMU data (`sensor_msgs/Imu`) on `/handsfree/imu`
- Publishes magnetometer data (`sensor_msgs/MagneticField`) on `/handsfree/mag`
- Example listener node to print roll, pitch, and yaw
- ROS2 launch file 
---

## Installation

1. **Clone the repository into your ROS2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/3bdul1ah/handsfree_ros2_imu
   ```

2. **Install dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-$ROS_DISTRO-rclpy ros-$ROS_DISTRO-sensor-msgs python3-serial python3-tf-transformations
   ```

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select handsfree_ros2_imu
   source install/setup.bash
   ```

---

## Device Setup

1. **Set up udev rules for the IMU device:**
   ```bash
   cd handsfree_ros2_imu
   chmod +x set_handsfree_rules.sh
   ./set_handsfree_rules.sh
   ```
   This will copy the necessary rules to `/etc/udev/rules.d/` and reload udev. You may need to unplug and replug your IMU device.

2. **Check device path:**
   The default device path is `/dev/HFRobotIMU`. You can change this in the launch file or as a parameter. To verify the device path, run:
   ```bash
   ls /dev/HFRobotIMU
   ```

---

## Usage

### 1. Launch the IMU node

```bash
ros2 launch handsfree_ros2_imu handsfree_imu.launch.py
```

This will start the IMU node, which publishes:

- `/handsfree/imu` (`sensor_msgs/Imu`)
- `/handsfree/mag` (`sensor_msgs/MagneticField`)

### 2. Listen to IMU data

You can use the provided listener node to print roll, pitch, and yaw:

```bash
ros2 run handsfree_ros2_imu get_imu_rclpy
```

Or, echo the topics directly:

```bash
ros2 topic echo /handsfree/imu
ros2 topic echo /handsfree/mag
```

---

## Parameters

You can set parameters in the launch file or via the command line:

- `port` (string): Serial port device (default: `/dev/HFRobotIMU`)
- `baudrate` (int): Serial baudrate (default: `921600`)
- `gra_normalization` (bool): Normalize gravity (default: `True`)

Example (override port):

```bash
ros2 launch handsfree_ros2_imu handsfree_imu.launch.py port:=/dev/ttyUSB0
```

---

## Credit

Original work by [HANDS-FREE/handsfree_ros_imu](https://github.com/HANDS-FREE/handsfree_ros_imu). 