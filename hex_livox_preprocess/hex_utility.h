/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-12
 ****************************************************************/

#ifndef HEX_LIVOX_PREPROCESS_HEX_UTILITY_H_
#define HEX_LIVOX_PREPROCESS_HEX_UTILITY_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <utility>
#include <vector>

struct PointXYZRTLT {
  PCL_ADD_POINT4D
  float reflectivity;
  uint8_t tag = 0;
  uint8_t line = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRTLT,
    (float, x, x)(float, y, y)(float, z, z)(float, reflectivity, reflectivity)(
        uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))

namespace hex {
namespace utility {

enum class ModeUndistortion { kRawMode = 0, kImuMode, kOdomMode };

struct HexParameterFilter {
  Eigen::Vector2d height;
  Eigen::Vector2d fov;
  Eigen::Vector2d distance;
};

struct HexParameterFlag {
  ModeUndistortion mode = ModeUndistortion::kRawMode;
};

struct HexParameterSensor {
  std::string frame;
  double lidar_period;
  double imu_period;
  double odom_period;
  Eigen::Matrix3d imu_in_sensor;
  Eigen::Affine3d sensor_in_base;
};

struct HexStamp {
  uint32_t sec;
  uint32_t nsec;
  // constructor
  HexStamp() : sec(0), nsec(0) {}
  HexStamp(uint32_t sec, uint64_t nsec) : sec(sec), nsec(nsec) {}
  // copy operator
  HexStamp& operator=(const HexStamp& that) {
    this->sec = that.sec;
    this->nsec = that.nsec;
    return *this;
  }
  // compare operator
  bool operator<(const HexStamp& that) const {
    if (this->sec < that.sec) {
      return true;
    } else if ((this->sec == that.sec) && (this->nsec < that.nsec)) {
      return true;
    } else {
      return false;
    }
  }
  bool operator>(const HexStamp& that) const {
    if (this->sec > that.sec) {
      return true;
    } else if ((this->sec == that.sec) && (this->nsec > that.nsec)) {
      return true;
    } else {
      return false;
    }
  }
  bool operator==(const HexStamp& that) const {
    return (this->sec == that.sec) && (this->nsec == that.nsec);
  }
  bool operator!=(const HexStamp& that) const { return !(*this == that); }
  bool operator<=(const HexStamp& that) const { return !(*this > that); }
  bool operator>=(const HexStamp& that) const { return !(*this < that); }
  // plus operator
  HexStamp operator+(double delta) const {
    if ((this->sec + this->nsec * 1e-9 + delta) < 0) {
      throw "Negative time is not allowed!";
    }

    HexStamp result = HexStamp();
    int64_t delta_sec = static_cast<int64_t>(delta);
    int64_t delta_nsec = static_cast<int64_t>((delta - delta_sec) * 1000000000);
    if (this->nsec + delta_nsec >= 1000000000) {
      result.sec = static_cast<uint32_t>(static_cast<int64_t>(this->sec) +
                                         delta_sec + 1);
      result.nsec = static_cast<uint32_t>(static_cast<int64_t>(this->nsec) +
                                          delta_nsec - 1000000000);
    } else if (this->nsec + delta_nsec < 0) {
      result.sec = static_cast<uint32_t>(static_cast<int64_t>(this->sec) +
                                         delta_sec - 1);
      result.nsec = static_cast<uint32_t>(static_cast<int64_t>(this->nsec) +
                                          delta_nsec + 1000000000);
    } else {
      result.sec =
          static_cast<uint32_t>(static_cast<int64_t>(this->sec) + delta_sec);
      result.nsec =
          static_cast<uint32_t>(static_cast<int64_t>(this->nsec) + delta_nsec);
    }
    return result;
  }
  // minus operator
  double operator-(const HexStamp& that) const {
    int64_t diff_sec =
        static_cast<int64_t>(this->sec) - static_cast<int64_t>(that.sec);
    int64_t diff_nsec =
        static_cast<int64_t>(this->nsec) - static_cast<int64_t>(that.nsec);
    double result = static_cast<double>(diff_sec) + diff_nsec * 1e-9;
    return result;
  }
  HexStamp operator-(double delta) const { return *this + (-delta); }
};

struct HexStampedPoints {
  HexStamp time;
  pcl::PointCloud<PointXYZRTLT>::Ptr points;
  // constructor
  HexStampedPoints() {
    this->time = HexStamp();
    this->points =
        pcl::PointCloud<PointXYZRTLT>::Ptr(new pcl::PointCloud<PointXYZRTLT>);
  }
  // copy operator
  HexStampedPoints& operator=(const HexStampedPoints& that) {
    this->time = that.time;
    this->points = pcl::PointCloud<PointXYZRTLT>::Ptr(
        new pcl::PointCloud<PointXYZRTLT>(*that.points));
    return *this;
  }
};

struct HexStampedOdom {
  HexStamp time;
  Eigen::Vector3d vel_lin;
  Eigen::Vector3d vel_ang;
  Eigen::Affine3d base_in_odom;
  // constructor
  HexStampedOdom() {
    this->time = HexStamp();
    this->vel_lin = Eigen::Vector3d::Zero();
    this->vel_ang = Eigen::Vector3d::Zero();
    this->base_in_odom = Eigen::Affine3d::Identity();
  }
  // copy operator
  HexStampedOdom& operator=(const HexStampedOdom& that) {
    this->time = that.time;
    this->vel_lin = Eigen::Vector3d(that.vel_lin);
    this->vel_ang = Eigen::Vector3d(that.vel_ang);
    this->base_in_odom = Eigen::Affine3d(that.base_in_odom);
    return *this;
  }
};

struct HexStampedImu {
  HexStamp time;
  Eigen::Vector3d acc_lin;
  Eigen::Vector3d vel_ang;
  // constructor
  HexStampedImu() {
    this->time = HexStamp();
    this->acc_lin = Eigen::Vector3d::Zero();
    this->vel_ang = Eigen::Vector3d::Zero();
  }
  // copy operator
  HexStampedImu& operator=(const HexStampedImu& that) {
    this->time = that.time;
    this->acc_lin = Eigen::Vector3d(that.acc_lin);
    this->vel_ang = Eigen::Vector3d(that.vel_ang);
    return *this;
  }
};

struct HexStampedPose {
  HexStamp time;
  Eigen::Affine3d new_in_old;
  // constructor
  HexStampedPose() {
    this->time = HexStamp();
    this->new_in_old = Eigen::Affine3d::Identity();
  }
  // copy operator
  HexStampedPose operator=(const HexStampedPose& that) {
    this->time = that.time;
    this->new_in_old = Eigen::Affine3d(that.new_in_old);
    return *this;
  }
};

}  // namespace utility
}  // namespace hex

#endif  // HEX_LIVOX_PREPROCESS_HEX_UTILITY_H_
