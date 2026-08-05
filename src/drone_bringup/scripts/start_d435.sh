#!/usr/bin/env bash
set -e

# Wait for the D435i to finish USB enumeration after boot before opening it.
sleep 10

source /opt/ros/humble/setup.bash
source ~/warehouse_ws/install/setup.bash

# Direct-capture perception chain (stages A-D): qr_vision_node opens the D435i
# itself via librealsense (YUYV 640x480@30), preprocesses through Nano2D into
# 640x640 NV12 for the BPU, and runs QR/OCR/capture. This replaces the old
# realsense2_camera_node ROS image path.
#
# Rollback to the old ROS path: restore the realsense2_camera launch below and
# set camera_input_mode back to "ros" in the params file. No systemd change is
# needed - this same service launches either chain.
exec ros2 run drone_perception qr_vision_node \
  --ros-args \
  --params-file /home/sunrise/warehouse_ws/src/drone_perception/config/d435_direct_detection.yaml

# --- rollback: old realsense2_camera_node path (do not delete) ---
# ros2 launch realsense2_camera rs_launch.py \
#   enable_color:=true \
#   rgb_camera.color_profile:=640,480,30 \
#   enable_depth:=false \
#   enable_rgbd:=false \
#   enable_sync:=false \
#   align_depth.enable:=false
