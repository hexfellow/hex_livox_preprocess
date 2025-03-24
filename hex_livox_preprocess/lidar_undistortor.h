/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-13
 ****************************************************************/

#ifndef HEX_LIVOX_PREPROCESS_LIDAR_UNDISTORTOR_H_
#define HEX_LIVOX_PREPROCESS_LIDAR_UNDISTORTOR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <vector>

#include "hex_livox_preprocess/hex_utility.h"

using hex::utility::HexParameterFlag;
using hex::utility::HexParameterSensor;
using hex::utility::HexStamp;
using hex::utility::HexStampedImu;
using hex::utility::HexStampedOdom;
using hex::utility::HexStampedPoints;
using hex::utility::HexStampedPose;
using hex::utility::ModeUndistortion;

namespace hex {
namespace preprocess {

class LidarUndistortor {
 public:
  static LidarUndistortor& GetSingleton() {
    static LidarUndistortor singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  const HexStampedPoints& UndistortCloud(const HexStampedPoints&,
                                         std::deque<HexStampedImu>&);
  const HexStampedPoints& UndistortCloud(const HexStampedPoints&,
                                         std::deque<HexStampedOdom>&);

 private:
  LidarUndistortor() = default;
  virtual ~LidarUndistortor() = default;

  // Work Handle
  void CalculatePoses(const HexStamp&, std::deque<HexStampedImu>&);
  void CalculatePoses(const HexStamp&, std::deque<HexStampedOdom>&);
  void Undistort(const HexStampedPoints&);

  // Help Handle
  Eigen::Affine3d CurrentTrans(const HexStamp&);
  Eigen::Affine3d InterpolateTrans(const Eigen::Affine3d&,
                                   const Eigen::Affine3d&, double);
  Eigen::Affine3d PredictTrans(const Eigen::Affine3d&, const Eigen::Affine3d&,
                               double);

  // Parameters
  HexParameterFlag kparameter_flag_;
  HexParameterSensor kparameter_sensor_;

  // Variables
  Eigen::Affine3d sensor_in_pose_;
  std::vector<HexStampedPose> pose_list_;
  HexStampedPoints processed_lidar_;
};

}  // namespace preprocess
}  // namespace hex

#endif  // HEX_LIVOX_PREPROCESS_LIDAR_UNDISTORTOR_H_
