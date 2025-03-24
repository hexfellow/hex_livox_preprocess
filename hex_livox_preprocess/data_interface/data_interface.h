/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-13
 ****************************************************************/

#ifndef HEX_LIVOX_PREPROCESS_DATA_INTERFACE_DATA_INTERFACE_H_
#define HEX_LIVOX_PREPROCESS_DATA_INTERFACE_DATA_INTERFACE_H_

#define HEX_ROS_VERSION 1
#if HEX_ROS_VERSION == 1
#include "hex_livox_preprocess/data_interface/ros1_interface.h"
#elif HEX_ROS_VERSION == 2
#include "hex_livox_preprocess/data_interface/ros2_interface.h"
#else
#warning "HEX_ROS_VERSION is unknown"
#endif

#endif  // HEX_LIVOX_PREPROCESS_DATA_INTERFACE_DATA_INTERFACE_H_
