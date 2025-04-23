# **hex_livox_preprocess**

## **Overview**

This **hex_livox_preprocess** repository provides an implementation of livox pointcloud undistortion and filter.

### **License**

This project is licensed under the terms of the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

### **Maintainer**

**[Dong Zhaorui](https://github.com/IBNBlank)**
**[Zhang Jianhuai](https://github.com/aalicecc)**

### **Supported Platform**

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [x] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**

### **Supported ROS Version**

- [x] **ROS Noetic**
- [ ] **ROS Humble**

---

## **Public APIs**

### **Publish**

| Topic              | Msg Type                  | Description                                |
| ------------------ | ------------------------- | ------------------------------------------ |
| `/raw_cloud`       | `sensor_msgs/PointCloud2` | Raw point cloud                 |
| `/processed_cloud` | `sensor_msgs/PointCloud2` | Undistorted point cloud      |

### **Subscribe**

| Topic    | Msg Type                      | Description                           |
| -------- | ----------------------------- | ------------------------------------- |
| `/lidar` | `livox_ros_driver2/CustomMsg` | Livox point cloud data  |
| `/imu`   | `sensor_msgs/Imu`             | IMU for correction          |
| `/odom`  | `nav_msgs/Odometry`           | Fusion odometry            |

### **Parameters**

| Name                    | Data Type             | Description                                                                                |
| ----------------------- | --------------------- | ------------------------------------------------------------------------------------------ |
| `filter_height`         | `std::vector<double>` | Height filter threshold                                                       |
| `filter_fov`           | `std::vector<double>` | FOV filter threshold                                                          |
| `filter_distance`       | `std::vector<double>` | Distance filter threshold                                                     |
| `flag_mode`            | `int32`               | Processing mode (0: raw_mode, 1: imu_mode, 2: odom_mode)                         |
| `sensor_frame`         | `std::string`         | Sensor frame name                                                        |
| `sensor_lidar_period`  | `double`              | LiDAR period                                                                    |
| `sensor_imu_period`    | `double`              | IMU period                                                                      |
| `sensor_odom_period`   | `double`              | Fusion odometry period                                                      |
| `sensor_imu_in_sensor` | `std::vector<double>` | IMU orientation in LiDAR frame (qw, qx, qy, qz)                      |
| `sensor_sensor_in_base`| `std::vector<double>` | LiDAR pose in odometry frame (x, y, z, qw, qx, qy, qz)  |

---

## **Getting Started**

### **Dependencies**

- **ROS Noetic** or **ROS Humble**
- **livox_ros_driver2**

### **Install**

1. Create a workspace `catkin_ws` and get into the `src`.

   ```shell
   mdkir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone this repo.

   ```shell
   git clone git@github.com:hexfellow/hex_livox_preprocess.git
   ```

3. Go to `catkin_ws` directory and build the repo.

   ```shell
   cd ../
   catkin_make
   ```

4. Source the `setup.bash` and run the test blow

   ```shell
   source devel/setup.bash --extend
   ```

### **Usage**

1. Launch the main node:

   ```shell
   roslaunch hex_livox_preprocess hex_livox_undistorted.launch
   ```
