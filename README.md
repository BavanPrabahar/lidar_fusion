# LiDAR Fusion

This repository provides a ROS 2 package to fuse scan data from two LiDAR sensors into a single, unified `LaserScan` message. 

## Overview

The primary goal of this project is to generate a single scan data stream from two separate LiDARs. This is particularly useful for mobile robots where a single LiDAR cannot provide 360-degree coverage without blind spots. 

**Key Features:**
* **Diagonal Placement:** Designed for LiDARs placed at diagonally opposite corners of a rectangular mobile robot.
* **No Direct Line of Sight:** Accommodates setups where the sensors have no direct line of sight between them.
* **Centralized Output:** The fused output (`/scan`) is transformed and treated as if a single LiDAR is positioned at the geometric center of the robot.
* **ROS 2 Integration:** Built using `rclpy`, providing seamless integration with the ROS 2 ecosystem.

## Dependencies

* ROS 2
* `rclpy`
* `sensor_msgs`
* `geometry_msgs`
* `tf2_ros`
* `numpy`

## Usage

### 1. `Lidar_fusion_working.py`

This node subscribes to `scan1` and `scan2` and combines the readings relative to a central `base_link` frame. It accounts for the physical offsets of the front and back LiDARs.

**Published Topics:**
* `scan` (`sensor_msgs/LaserScan`): The fused scan data.
* TF transform: `base_link` -> `laser`.

**Subscribed Topics:**
* `scan1` (`sensor_msgs/LaserScan`): Front LiDAR data.
* `scan2` (`sensor_msgs/LaserScan`): Back LiDAR data.

**Run the node:**
```bash
python3 Lidar_fusion_working.py
```

### 2. `error.py`

An alternative script demonstrating multithreaded merging capabilities (Note: this file contains experimental logic for handling callback synchronization).

**Run the node:**
```bash
python3 error.py
```

## Configuration

You can adjust the LiDAR offset configurations directly in `Lidar_fusion_working.py` under the `__init__` function of the `DataFuse` class:
```python
self.front_offset_x = 0.19
self.front_offset_y = 0.09
self.back_offset_x = 0.17
self.back_offset_y = 0.09
```
