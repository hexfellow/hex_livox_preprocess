/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-12
 ****************************************************************/

#ifndef HEX_LIVOX_PREPROCESS_DATA_INTERFACE_BASE_INTERFACE_H_
#define HEX_LIVOX_PREPROCESS_DATA_INTERFACE_BASE_INTERFACE_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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

enum class HexLogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

class BaseInterface {
 public:
  BaseInterface() : lidar_flag_(false), imu_flag_(false), odom_flag_(false) {}
  virtual ~BaseInterface() {}
  BaseInterface(const BaseInterface&) = delete;
  BaseInterface& operator=(const BaseInterface&) = delete;

  // Interface Handle
  virtual void Log(HexLogLevel, const char*, ...) = 0;
  virtual void Shutdown() = 0;
  virtual bool Ok() = 0;
  virtual HexStamp GetTime() = 0;
  virtual void Work() = 0;

  // Publisher Handle
  virtual void PublishRawCloud(const HexStampedPoints&) = 0;
  virtual void PublishProcessedCloud(const HexStampedPoints&) = 0;

  // Initialization Handle
  virtual void Init(int, char*[], std::string, double, void (*)()) = 0;
  virtual void Deinit() = 0;

  // Parameter Handle
  inline const HexParameterFilter& GetParameterFilter() const {
    return kparameter_filter_;
  }
  inline const HexParameterFlag& GetParameterFlag() const {
    return kparameter_flag_;
  }
  inline const HexParameterSensor& GetParameterSensor() const {
    return kparameter_sensor_;
  }

  // Subscriber Handle
  inline bool GetLidarFlag() { return lidar_flag_; }
  inline void ResetLidarFlag() { lidar_flag_ = false; }
  inline const HexStampedPoints& GetLidar() {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    return lidar_;
  }
  inline bool GetImuFlag() { return imu_flag_; }
  inline void ResetImuFlag() { imu_flag_ = false; }
  inline const std::vector<HexStampedImu>& GetImuBuffer() {
    static std::vector<HexStampedImu> imu_buffer;
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer = std::vector<HexStampedImu>(imu_buffer_);
    imu_buffer_.clear();
    return imu_buffer;
  }
  inline void ResetImuBuffer() { imu_buffer_.clear(); }
  inline bool GetOdomFlag() { return odom_flag_; }
  inline void ResetOdomFlag() { odom_flag_ = false; }
  inline const std::vector<HexStampedOdom>& GetOdomBuffer() {
    static std::vector<HexStampedOdom> odom_buffer;
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer = std::vector<HexStampedOdom>(odom_buffer_);
    odom_buffer_.clear();
    return odom_buffer;
  }

 protected:
  // Initialization Handle
  virtual void ParameterInit() = 0;
  virtual void VariableInit() = 0;
  virtual void PublisherInit() = 0;
  virtual void SubscriberInit() = 0;
  virtual void TimerInit(double, void (*)()) = 0;

  // Time Handle
  void (*timer_handle_)();

  // Parameters
  HexParameterFilter kparameter_filter_;
  HexParameterFlag kparameter_flag_;
  HexParameterSensor kparameter_sensor_;

  // Variables
  std::atomic<bool> lidar_flag_;
  mutable std::mutex lidar_mutex_;
  HexStampedPoints lidar_;
  std::atomic<bool> imu_flag_;
  mutable std::mutex imu_mutex_;
  std::vector<HexStampedImu> imu_buffer_;
  std::atomic<bool> odom_flag_;
  mutable std::mutex odom_mutex_;
  std::vector<HexStampedOdom> odom_buffer_;
};

}  // namespace preprocess
}  // namespace hex

#endif  // HEX_LIVOX_PREPROCESS_DATA_INTERFACE_BASE_INTERFACE_H_
