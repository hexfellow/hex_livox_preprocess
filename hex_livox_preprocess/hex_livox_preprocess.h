/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-13
 ****************************************************************/

#ifndef HEX_LIVOX_PREPROCESS_HEX_LIVOX_PREPROCESS_H_
#define HEX_LIVOX_PREPROCESS_HEX_LIVOX_PREPROCESS_H_

#include <deque>

#include "hex_livox_preprocess/hex_utility.h"

using hex::utility::HexParameterFilter;
using hex::utility::HexParameterFlag;
using hex::utility::HexParameterSensor;
using hex::utility::HexStamp;
using hex::utility::HexStampedImu;
using hex::utility::HexStampedOdom;
using hex::utility::HexStampedPoints;
using hex::utility::ModeUndistortion;

namespace hex {
namespace preprocess {

class HexLivoxPreprocessed {
 public:
  static HexLivoxPreprocessed& GetSingleton() {
    static HexLivoxPreprocessed singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  HexLivoxPreprocessed() = default;
  virtual ~HexLivoxPreprocessed() = default;

  // Work Handle
  void RawModeLoop();
  void ImuModeLoop();
  void OdomModeLoop();

  // Help Handle
  bool ImuUndistortCloud();
  bool OdomUndistortCloud();
  bool FilterCloud();

  // Parameters
  HexParameterFilter kparameter_filter_;
  HexParameterFlag kparameter_flag_;
  HexParameterSensor kparameter_sensor_;

  // Variables
  HexStampedPoints raw_lidar_;
  HexStampedPoints processed_lidar_;
  std::deque<HexStampedImu> imu_queue_;
  std::deque<HexStampedOdom> odom_queue_;
};

}  // namespace preprocess
}  // namespace hex

#endif  // HEX_LIVOX_PREPROCESS_HEX_LIVOX_PREPROCESS_H_
