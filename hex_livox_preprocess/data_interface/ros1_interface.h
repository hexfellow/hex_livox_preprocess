/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-13
 ****************************************************************/

#ifndef HEX_LIVOX_PREPROCESS_DATA_INTERFACE_ROS1_INTERFACE_H_
#define HEX_LIVOX_PREPROCESS_DATA_INTERFACE_ROS1_INTERFACE_H_

#include <ros/ros.h>

#include <memory>
#include <string>

#include "hex_livox_preprocess/data_interface/base_interface.h"
#include "hex_livox_preprocess/hex_utility.h"
#include "livox_ros_driver2/CustomMsg.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"

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

class DataInterface : public BaseInterface {
 public:
  static DataInterface& GetSingleton() {
    static DataInterface singleton;
    return singleton;
  }

  // Interface Handle
  void Log(HexLogLevel, const char*, ...) override;
  inline void Shutdown() override { ros::shutdown(); }
  inline bool Ok() override { return ros::ok(); }
  inline HexStamp GetTime() override {
    ros::Time time = ros::Time::now();
    return HexStamp(time.sec, time.nsec);
  }
  inline void Work() override {
    timer_->reset();
    while (ros::ok()) {
      timer_handle_();
      timer_->sleep();
    }
  }

  // Publisher Handle
  void PublishRawCloud(const HexStampedPoints&) override;
  void PublishProcessedCloud(const HexStampedPoints&) override;

  // Initialization Handle
  void Init(int, char*[], std::string, double, void (*)()) override;
  void Deinit() override;

 protected:
  // Subscriber Handle
  void LidarHandle(const livox_ros_driver2::CustomMsgPtr&);
  void ImuHandle(const sensor_msgs::ImuPtr&);
  void OdomHandle(const nav_msgs::OdometryPtr&);

 private:
  DataInterface() = default;
  virtual ~DataInterface() = default;

  // Initialization Handle
  void ParameterInit() override;
  void VariableInit() override;
  void PublisherInit() override;
  void SubscriberInit() override;
  void TimerInit(double, void (*)()) override;

  // Node Handle
  ros::NodeHandle* nh_ptr_;
  ros::NodeHandle* nh_local_ptr_;

  // Timer Handle
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::unique_ptr<ros::Rate> timer_;

  // Publisher Handle
  ros::Publisher raw_cloud_pub_;
  ros::Publisher processed_cloud_pub_;

  // Subscriber Handle
  ros::Subscriber lidar_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub_;
};

}  // namespace preprocess
}  // namespace hex

#endif  // HEX_LIVOX_PREPROCESS_DATA_INTERFACE_ROS1_INTERFACE_H_
