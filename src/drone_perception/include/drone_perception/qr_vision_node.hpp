#pragma once

#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "drone_perception/rknn_yolo_detector.hpp"
#include "drone_perception/depth_processor.hpp"

class QrVisionNode : public rclcpp::Node
{
public:
  QrVisionNode();

  ~QrVisionNode() override;

private:
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>
      ColorDepthSyncPolicy;

  void declareParameters();

  void initializeSubscriptions();

  void handleSyncedFrame(
      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg);

  void handleCameraInfo(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg);

  void displayDebugFrame(
    const cv::Mat &color_image,
    const std::vector<Detection> &detections);

  void updateFps();

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string window_name_;

  bool debug_view_ = true;
  mutable bool debug_window_created_ = false;
  int log_throttle_ms_ = 2000;
  int sample_radius_px_ = 3;

  rclcpp::Time last_frame_time_;
  double smoothed_fps_ = 0.0;

  DepthProcessor depth_processor_;
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool has_camera_info_ = false;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<ColorDepthSyncPolicy>> color_depth_sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  std::string model_path_;
  std::unique_ptr<RknnYoloDetector> detector_;
};
