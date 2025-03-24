#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2023 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2023-11-21
################################################################

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hex_livox_preprocess_param = os.path.join(
        get_package_share_directory("hex_livox_preprocess"), "config/ros2",
        "hex_livox_preprocess.yaml")

    hex_livox_preprocess = Node(name="hex_livox_preprocess",
                       package="hex_livox_preprocess",
                       executable="hex_livox_preprocess",
                       output="screen",
                       parameters=[hex_livox_preprocess_param, {
                           "max_count": 20
                       }],
                       remappings=[("/in_string", "/in"),
                                   ("/out_string", "/out")])

    return LaunchDescription([hex_livox_preprocess])
