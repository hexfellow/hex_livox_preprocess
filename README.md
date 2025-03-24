# hex_livox_preprocess

This repo provides an implementation of livox pointcloud undistortion and filter.

## Maintainer

Dong Zhaorui(<847235539@qq.com>)

## Public APIs

### Publish

| Topic              | Msg Type                  | Description  |
| ------------------ | ------------------------- | ------------ |
| `/raw_cloud`       | `sensor_msgs/PointCloud2` | 原始的点云   |
| `/processed_cloud` | `sensor_msgs/PointCloud2` | 校正后的点云 |

### Subscribe

| Topic    | Msg Type                      | Description    |
| -------- | ----------------------------- | -------------- |
| `/lidar` | `livox_ros_driver2/CustomMsg` | livox 点云数据 |
| `/imu`   | `sensor_msgs/Imu`             | 校正用IMU      |
| `/odom`  | `nav_msgs/Odometry`           | 融合里程计     |

### Parameters

| Name                    | Data Type             | Description                                              |
| ----------------------- | --------------------- | -------------------------------------------------------- |
| `filter_height`         | `std::vector<double>` | 高度滤波阈值                                             |
| `filter_fov`            | `std::vector<double>` | FOV滤波阈值                                              |
| `filter_distance`       | `std::vector<double>` | 距离滤波阈值                                             |
| `flag_mode`             | `int`                 | `0`表示`raw_mode`; `1`表示`imu_mode`; `2`表示`odom_mode` |
| `sensor_frame`          | `std::string`         | Lidar坐标系                                              |
| `sensor_lidar_period`   | `double`              | Lidar周期                                                |
| `sensor_imu_period`     | `double`              | IMU周期                                                  |
| `sensor_odom_period`    | `double`              | 融合里程计周期                                           |
| `sensor_imu_in_sensor`  | `std::vector<double>` | IMU在Lidar系下的方向                                     |
| `sensor_sensor_in_base` | `std::vector<double>` | Lidar在融合里程计`child_frame_id`下的坐标变化            |

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

1. Create a workspace `catkin_ws` and get into the `src`.

   ```shell
   mdkir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone this repo.

   ```shell
   git clone git@gitlab.hexmove.cn:IBN_Blank/hex_livox_preprocess.git
   ```

3. Run install script `hex_install.py`.

   ```shell
   python3 hex_livox_preprocess/tools/hex_install.py
   ```

4. Go to `catkin_ws` dir and build the repo.

   ```shell
   cd ../
   catkin_make
   ```

5. Source the `setup.bash` and run the test blow

   ```shell
   source devel/setup.bash --extend
   ```

### Platforms

* [ ] **Jetson Orin NX**
* [ ] **Jetson Orin Nano**
* [ ] **Jetson AGX Orin**
* [ ] **RK3588**

### Prerequisites

What additional things you need to use the software

**Livox ROS Driver 2:**
Follow the steps in `https://github.com/Livox-SDK/livox_ros_driver2`

**Foxglove: (optional)**

```shell
wget https://get.foxglove.dev/desktop/latest/foxglove-studio-2.4.0-linux-amd64.deb
sudo apt install ./foxglove-studio-*.deb
```

### Usage

```shell
roslaunch hex_livox_preprocess hex_livox_preprocess.launch
```

## Running the tests

```shell
terminal_1:
$ roslaunch hex_livox_preprocess hex_livox_preprocess.launch

terminal_2:
$ roscd hex_livox_preprocess/tests/
$ rosbag play livox.bag --clock
```

## Reminder

1. 使用`ModeUndistortion::kImuMode`时，确保`/imu`话题正常
2. 使用`ModeUndistortion::kOdomMode`时，确保`/odom`话题正常
3. `/odom`不建议使用 **全局定位**
