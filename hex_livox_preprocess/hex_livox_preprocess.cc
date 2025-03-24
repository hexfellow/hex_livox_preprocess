/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-13
 ****************************************************************/

#include "hex_livox_preprocess/hex_livox_preprocess.h"

#include <vector>

#include "hex_livox_preprocess/data_interface/data_interface.h"
#include "hex_livox_preprocess/lidar_filter.h"
#include "hex_livox_preprocess/lidar_undistortor.h"

namespace hex {
namespace preprocess {

bool HexLivoxPreprocessed::Init() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarUndistortor& lidar_undistortor = LidarUndistortor::GetSingleton();
  static LidarFilter& lidar_filter = LidarFilter::GetSingleton();

  // Parameter
  kparameter_sensor_ = data_interface.GetParameterSensor();
  kparameter_flag_ = data_interface.GetParameterFlag();

  // Variable
  raw_lidar_ = HexStampedPoints();
  processed_lidar_ = HexStampedPoints();
  imu_queue_.clear();
  odom_queue_.clear();

  // Other Components
  if (kparameter_flag_.mode != ModeUndistortion::kRawMode) {
    lidar_undistortor.Init();
  }
  lidar_filter.Init();

  return true;
}

bool HexLivoxPreprocessed::Work() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  switch (kparameter_flag_.mode) {
    case ModeUndistortion::kRawMode: {
      RawModeLoop();
      break;
    }
    case ModeUndistortion::kImuMode: {
      ImuModeLoop();
      break;
    }
    case ModeUndistortion::kOdomMode: {
      OdomModeLoop();
      break;
    }

    default: {
      data_interface.Log(HexLogLevel::kWarn, "### Wrong Mode ###");
      RawModeLoop();
      break;
    }
  }

  return true;
}

void HexLivoxPreprocessed::RawModeLoop() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarFilter& lidar_filter = LidarFilter::GetSingleton();

  if (data_interface.GetLidarFlag()) {
    raw_lidar_ = data_interface.GetLidar();
    data_interface.ResetLidarFlag();

    processed_lidar_ = raw_lidar_;
    if (!FilterCloud()) return;
    data_interface.PublishRawCloud(processed_lidar_);
  }
}

void HexLivoxPreprocessed::ImuModeLoop() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static HexStamp imu_last_time = HexStamp();

  if (data_interface.GetImuFlag()) {
    std::vector<HexStampedImu> imu_vector =
        std::vector<HexStampedImu>(data_interface.GetImuBuffer());
    data_interface.ResetImuFlag();

    for (auto& imu : imu_vector) {
      if (imu.time < imu_last_time) {
        data_interface.Log(HexLogLevel::kWarn, "imu.time < imu_last_time");
        continue;
      }

      imu_last_time = imu.time;
      imu_queue_.emplace_back(imu);
    }
  }

  if (data_interface.GetLidarFlag()) {
    raw_lidar_ = data_interface.GetLidar();
    if (raw_lidar_.time == HexStamp()) {
      data_interface.Log(HexLogLevel::kError, "Lidar time error");
    }

    data_interface.ResetLidarFlag();

    if (!ImuUndistortCloud()) return;
    if (!FilterCloud()) return;
    data_interface.PublishProcessedCloud(processed_lidar_);
  }
}

void HexLivoxPreprocessed::OdomModeLoop() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static HexStamp odom_last_time = HexStamp();

  if (data_interface.GetOdomFlag()) {
    std::vector<HexStampedOdom> odom_vector =
        std::vector<HexStampedOdom>(data_interface.GetOdomBuffer());
    data_interface.ResetOdomFlag();

    for (auto& odom : odom_vector) {
      if (odom.time < odom_last_time) {
        data_interface.Log(HexLogLevel::kWarn, "Odom time error");
        odom_queue_.clear();
      }

      odom_last_time = odom.time;
      odom_queue_.emplace_back(odom);
    }
  }

  if (data_interface.GetLidarFlag()) {
    raw_lidar_ = data_interface.GetLidar();
    data_interface.ResetLidarFlag();

    if (!OdomUndistortCloud()) return;
    if (!FilterCloud()) return;
    data_interface.PublishProcessedCloud(processed_lidar_);
  }
}

bool HexLivoxPreprocessed::ImuUndistortCloud() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarUndistortor& lidar_undistortor = LidarUndistortor::GetSingleton();

  processed_lidar_ = lidar_undistortor.UndistortCloud(raw_lidar_, imu_queue_);

  if (processed_lidar_.time == HexStamp()) {
    data_interface.Log(HexLogLevel::kWarn, "Undistort cloud failed");
    return false;
  }

  return true;
}

bool HexLivoxPreprocessed::OdomUndistortCloud() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarUndistortor& lidar_undistortor = LidarUndistortor::GetSingleton();

  processed_lidar_ = lidar_undistortor.UndistortCloud(raw_lidar_, odom_queue_);

  if (processed_lidar_.time == HexStamp()) {
    data_interface.Log(HexLogLevel::kWarn, "Undistort cloud failed");
    return false;
  }

  return true;
}

bool HexLivoxPreprocessed::FilterCloud() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarFilter& lidar_filter = LidarFilter::GetSingleton();

  processed_lidar_ = lidar_filter.FilterCloud(processed_lidar_);

  if (processed_lidar_.time == HexStamp()) {
    data_interface.Log(HexLogLevel::kWarn, "Filter cloud failed");
    return false;
  }

  return true;
}

}  // namespace preprocess
}  // namespace hex
