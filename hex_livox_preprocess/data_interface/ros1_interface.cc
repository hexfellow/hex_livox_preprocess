/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-13
 ****************************************************************/

#include "hex_livox_preprocess/data_interface/ros1_interface.h"

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>

namespace hex {
namespace preprocess {

void DataInterface::Log(HexLogLevel level, const char* format, ...) {
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0) {
    ROS_FATAL("### Wrong Log Message ###");
    return;
  }

  switch (level) {
    case HexLogLevel::kDebug: {
      ROS_DEBUG("%s", buffer);
      break;
    }
    case HexLogLevel::kInfo: {
      ROS_INFO("%s", buffer);
      break;
    }
    case HexLogLevel::kWarn: {
      ROS_WARN("%s", buffer);
      break;
    }
    case HexLogLevel::kError: {
      ROS_ERROR("%s", buffer);
      break;
    }
    case HexLogLevel::kFatal: {
      ROS_FATAL("%s", buffer);
      break;
    }
    default: {
      ROS_FATAL("### Wrong Log Level ###");
      ROS_FATAL("%s", buffer);
      break;
    }
  }

  free(buffer);
}

void DataInterface::Init(int argc, char* argv[], std::string name,
                         double period, void (*handle)()) {
  ros::init(argc, argv, name);
  static ros::NodeHandle nh;
  static ros::NodeHandle nh_local("~");
  nh_ptr_ = &nh;
  nh_local_ptr_ = &nh_local;

  ParameterInit();
  VariableInit();
  PublisherInit();
  SubscriberInit();
  TimerInit(period, handle);

  Log(HexLogLevel::kInfo,
      "\033[1;32m %s: ### data interface init finish ### \033[0m", name.data());
}

void DataInterface::Deinit() {
  spinner_->stop();
  ros::shutdown();
  spinner_.reset();
  timer_.reset();
}

void DataInterface::ParameterInit() {
  // filter parameter
  std::vector<double> param_filter_height;
  std::vector<double> param_filter_fov;
  std::vector<double> param_filter_distance;
  nh_local_ptr_->param<std::vector<double>>(
      "filter_height", param_filter_height, std::vector<double>({-20.0, 20.0}));
  nh_local_ptr_->param<std::vector<double>>("filter_fov", param_filter_fov,
                                            std::vector<double>({-M_PI, M_PI}));
  nh_local_ptr_->param<std::vector<double>>("filter_distance",
                                            param_filter_distance,
                                            std::vector<double>({0.1, 100.0}));
  kparameter_filter_.height =
      Eigen::Vector2d(param_filter_height[0], param_filter_height[1]);
  kparameter_filter_.fov =
      Eigen::Vector2d(param_filter_fov[0], param_filter_fov[1]);
  kparameter_filter_.distance =
      Eigen::Vector2d(param_filter_distance[0], param_filter_distance[1]);

  // flag parameter
  int32_t param_flag_mode;
  nh_local_ptr_->param<int32_t>("flag_mode", param_flag_mode, 0);
  switch (param_flag_mode) {
    case 0: {
      kparameter_flag_.mode = ModeUndistortion::kRawMode;
      break;
    }
    case 1: {
      kparameter_flag_.mode = ModeUndistortion::kImuMode;
      break;
    }
    case 2: {
      kparameter_flag_.mode = ModeUndistortion::kOdomMode;
      break;
    }
    default: {
      Log(HexLogLevel::kWarn, "### Wrong Mode ###");
      kparameter_flag_.mode = ModeUndistortion::kRawMode;
      break;
    }
  }

  // sensor parameter
  std::vector<double> param_sensor_imu_in_sensor;
  std::vector<double> param_sensor_sensor_in_base;
  nh_local_ptr_->param<std::string>("sensor_frame", kparameter_sensor_.frame,
                                    "livox");
  nh_local_ptr_->param<double>("sensor_lidar_period",
                               kparameter_sensor_.lidar_period, 0.1);
  nh_local_ptr_->param<double>("sensor_imu_period",
                               kparameter_sensor_.imu_period, 0.005);
  nh_local_ptr_->param<double>("sensor_odom_period",
                               kparameter_sensor_.odom_period, 0.02);
  nh_local_ptr_->param<std::vector<double>>(
      "sensor_imu_in_sensor", param_sensor_imu_in_sensor,
      std::vector<double>({1.0, 0.0, 0.0, 0.0}));
  nh_local_ptr_->param<std::vector<double>>(
      "sensor_sensor_in_base", param_sensor_sensor_in_base,
      std::vector<double>({0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}));
  kparameter_sensor_.imu_in_sensor = Eigen::Quaterniond(
      param_sensor_imu_in_sensor[0], param_sensor_imu_in_sensor[1],
      param_sensor_imu_in_sensor[2], param_sensor_imu_in_sensor[3]);
  kparameter_sensor_.sensor_in_base.matrix().block<3, 3>(0, 0) =
      Eigen::Quaterniond(
          param_sensor_sensor_in_base[3], param_sensor_sensor_in_base[4],
          param_sensor_sensor_in_base[5], param_sensor_sensor_in_base[6])
          .toRotationMatrix();
  kparameter_sensor_.sensor_in_base.matrix().block<3, 1>(0, 3) =
      Eigen::Vector3d(param_sensor_sensor_in_base[0],
                      param_sensor_sensor_in_base[1],
                      param_sensor_sensor_in_base[2]);
}

void DataInterface::VariableInit() {
  lidar_flag_ = false;
  lidar_.time = HexStamp();
  lidar_.points =
      pcl::PointCloud<PointXYZRTLT>::Ptr(new pcl::PointCloud<PointXYZRTLT>);

  imu_flag_ = false;
  imu_buffer_.clear();

  odom_flag_ = false;
  odom_buffer_.clear();
}

void DataInterface::PublisherInit() {
  raw_cloud_pub_ =
      nh_ptr_->advertise<sensor_msgs::PointCloud2>("/raw_cloud", 1);
  processed_cloud_pub_ =
      nh_ptr_->advertise<sensor_msgs::PointCloud2>("/processed_cloud", 1);
}

void DataInterface::SubscriberInit() {
  lidar_sub_ =
      nh_ptr_->subscribe("/lidar", 1, &DataInterface::LidarHandle, this);
  imu_sub_ = nh_ptr_->subscribe("/imu", 50, &DataInterface::ImuHandle, this);
  odom_sub_ = nh_ptr_->subscribe("/odom", 10, &DataInterface::OdomHandle, this);
}

void DataInterface::TimerInit(double period, void (*handle)()) {
  // spin thread
  spinner_ = std::make_unique<ros::AsyncSpinner>(1);
  spinner_->start();

  // work thread
  timer_handle_ = handle;
  timer_ = std::unique_ptr<ros::Rate>(new ros::Rate(1000.0 / period));
}

void DataInterface::PublishRawCloud(const HexStampedPoints& data) {
  sensor_msgs::PointCloud2Ptr raw_cloud_ptr(new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*data.points, *raw_cloud_ptr);
  raw_cloud_ptr->header.stamp = ros::Time(data.time.sec, data.time.nsec);
  raw_cloud_ptr->header.frame_id = kparameter_sensor_.frame;

  raw_cloud_pub_.publish(raw_cloud_ptr);
}

void DataInterface::PublishProcessedCloud(const HexStampedPoints& data) {
  sensor_msgs::PointCloud2Ptr processed_cloud_ptr(new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*data.points, *processed_cloud_ptr);
  processed_cloud_ptr->header.stamp = ros::Time(data.time.sec, data.time.nsec);
  processed_cloud_ptr->header.frame_id = kparameter_sensor_.frame;

  processed_cloud_pub_.publish(processed_cloud_ptr);
}

void DataInterface::LidarHandle(const livox_ros_driver2::CustomMsgPtr& msg) {
  ros::Time msg_time;
  msg_time.fromNSec(msg->timebase);

  if (ros::Time::now().toSec() - msg_time.toSec() < 0.2 && !lidar_flag_) {
    std::lock_guard<std::mutex> lock(lidar_mutex_);

    lidar_.time = HexStamp(msg_time.sec, msg_time.nsec);
    lidar_.points->clear();

    for (size_t i = 0; i < msg->points.size(); i++) {
      PointXYZRTLT point;
      point.x = msg->points[i].x;
      point.y = msg->points[i].y;
      point.z = msg->points[i].z;
      point.reflectivity = static_cast<float>(msg->points[i].reflectivity);
      point.tag = msg->points[i].tag;
      point.line = msg->points[i].line;
      point.timestamp = msg_time.fromNSec(msg->points[i].offset_time).toSec();
      lidar_.points->push_back(point);
    }

    lidar_flag_ = true;
  }
}

void DataInterface::ImuHandle(const sensor_msgs::ImuPtr& msg) {
  static HexStampedImu curr_imu;
  if (ros::Time::now().toSec() - msg->header.stamp.toSec() < 0.005) {
    std::lock_guard<std::mutex> lock(imu_mutex_);

    curr_imu.time = HexStamp(msg->header.stamp.sec, msg->header.stamp.nsec);
    curr_imu.acc_lin =
        Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z);
    curr_imu.vel_ang =
        Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                        msg->angular_velocity.z);

    imu_buffer_.emplace_back(curr_imu);
    imu_flag_ = true;
  }
}

void DataInterface::OdomHandle(const nav_msgs::OdometryPtr& msg) {
  static HexStampedOdom curr_odom;
  if (ros::Time::now().toSec() - msg->header.stamp.toSec() < 0.1) {
    std::lock_guard<std::mutex> lock(odom_mutex_);

    curr_odom.time = HexStamp(msg->header.stamp.sec, msg->header.stamp.nsec);
    curr_odom.vel_lin =
        Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
    curr_odom.vel_ang =
        Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                        msg->twist.twist.angular.z);

    curr_odom.base_in_odom = Eigen::Affine3d::Identity();
    curr_odom.base_in_odom.matrix().block<3, 3>(0, 0) =
        Eigen::Quaterniond(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
            .toRotationMatrix();
    curr_odom.base_in_odom.matrix().block<3, 1>(0, 3) =
        Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);

    odom_buffer_.emplace_back(curr_odom);
    odom_flag_ = true;
  }
}

}  // namespace preprocess
}  // namespace hex
