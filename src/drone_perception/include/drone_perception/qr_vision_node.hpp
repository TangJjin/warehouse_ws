#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "drone_perception/depth_processor.hpp"

#ifndef DRONE_PERCEPTION_HAS_BPU
#define DRONE_PERCEPTION_HAS_BPU 0
#endif

#if DRONE_PERCEPTION_HAS_BPU
#include "drone_perception/bpu_yolo_detector.hpp"
#endif

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

  void initializeBpuDetector();

  bool prepareBpuInput(const cv::Mat &color_image);

  void handleSyncedFrame(
      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg);

  void handleCameraInfo(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg);

  void displayDebugFrame(
      const cv::Mat &color_image,
      const DepthSampleResult &center_depth);

#if DRONE_PERCEPTION_HAS_BPU
  void drawBpuDetections(cv::Mat &display) const;
#endif

  void updateFps();

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string window_name_;

  std::string bpu_model_path_;

  std::vector<uint8_t> bpu_input_nv12_;

  bool debug_view_ = true;
  bool enable_bpu_ = false;
  mutable bool debug_window_created_ = false;
  int log_throttle_ms_ = 500;
  int sample_radius_px_ = 3;

  rclcpp::Time last_frame_time_;
  double smoothed_fps_ = 0.0;

  DepthProcessor depth_processor_;

#if DRONE_PERCEPTION_HAS_BPU
  std::unique_ptr<BpuYoloDetector> bpu_detector_;
  std::vector<BpuYoloDetection> last_bpu_detections_;
#endif

  bool has_camera_info_ = false;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<ColorDepthSyncPolicy>> color_depth_sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
};
