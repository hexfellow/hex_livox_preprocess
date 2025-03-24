/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-13
 ****************************************************************/

#include "hex_livox_preprocess/lidar_filter.h"

#include <pcl/common/transforms.h>

#include "hex_livox_preprocess/data_interface/data_interface.h"

namespace hex {
namespace preprocess {

bool LidarFilter::Init() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  // Parameter
  kparameter_filter_ = data_interface.GetParameterFilter();
  kparameter_sensor_ = data_interface.GetParameterSensor();
  kdistance_min_square_ =
      kparameter_filter_.distance[0] * kparameter_filter_.distance[0];
  kdistance_max_square_ =
      kparameter_filter_.distance[1] * kparameter_filter_.distance[1];

  // Variable
  sensor_in_base_ = kparameter_sensor_.sensor_in_base;
  processed_lidar_ = HexStampedPoints();

  return true;
}

const HexStampedPoints& LidarFilter::FilterCloud(
    const HexStampedPoints& raw_lidar) {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  if (raw_lidar.points->empty()) {
    data_interface.Log(HexLogLevel::kWarn, "Raw lidar empty");
    processed_lidar_.time = HexStamp();
    return processed_lidar_;
  }

  // trans: P_out = T * P_in
  pcl::transformPointCloud(*raw_lidar.points, *processed_lidar_.points,
                           sensor_in_base_);

  // filter
  pcl::PointCloud<PointXYZRTLT>::Ptr cloud_filtered(
      new pcl::PointCloud<PointXYZRTLT>);

  for (const auto& point : processed_lidar_.points->points) {
    if (point.z < kparameter_filter_.height[0] ||
        point.z > kparameter_filter_.height[1]) {
      continue;
    }

    double distance = point.x * point.x + point.y * point.y;
    if (distance < kdistance_min_square_ || distance > kdistance_max_square_) {
      continue;
    }

    double angle = atan2(point.y, point.x);
    if (angle < kparameter_filter_.fov[0] ||
        angle > kparameter_filter_.fov[1]) {
      continue;
    }

    cloud_filtered->push_back(point);
  }

  processed_lidar_.time = raw_lidar.time;
  processed_lidar_.points = cloud_filtered;
  return processed_lidar_;
}

}  // namespace preprocess
}  // namespace hex
