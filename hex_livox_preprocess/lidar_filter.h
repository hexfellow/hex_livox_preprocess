/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-13
 ****************************************************************/

#ifndef HEX_LIVOX_PREPROCESS_LIDAR_FILTER_H_
#define HEX_LIVOX_PREPROCESS_LIDAR_FILTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <vector>

#include "hex_livox_preprocess/hex_utility.h"

using hex::utility::HexParameterFilter;
using hex::utility::HexParameterSensor;
using hex::utility::HexStamp;
using hex::utility::HexStampedPoints;
using hex::utility::HexStampedPose;

namespace hex {
namespace preprocess {

class LidarFilter {
 public:
  static LidarFilter& GetSingleton() {
    static LidarFilter singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  const HexStampedPoints& FilterCloud(const HexStampedPoints&);

 private:
  LidarFilter() = default;
  virtual ~LidarFilter() = default;

  // Parameters
  HexParameterFilter kparameter_filter_;
  HexParameterSensor kparameter_sensor_;
  double kdistance_min_square_;
  double kdistance_max_square_;

  // Variables
  Eigen::Affine3d sensor_in_base_;
  HexStampedPoints processed_lidar_;
};

}  // namespace preprocess
}  // namespace hex

#endif  // HEX_LIVOX_PREPROCESS_LIDAR_FILTER_H_
