#!/usr/bin/env bash
set -e

sleep 10

source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_rgbd:=true \
  enable_sync:=true \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  rgb_camera.color_profile:=640,480,30 \
  depth_module.depth_profile:=640,480,30 \
  clip_distance:=5.0