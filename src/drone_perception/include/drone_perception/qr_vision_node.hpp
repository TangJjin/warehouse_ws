#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "drone_perception/depth_processor.hpp"

#ifndef DRONE_PERCEPTION_HAS_BPU
#define DRONE_PERCEPTION_HAS_BPU 0
#endif

#if DRONE_PERCEPTION_HAS_BPU
#include "drone_perception/bpu_ocr_pipeline.hpp"
#include "drone_perception/bpu_yolo_detector.hpp"
#endif

class QrVisionNode : public rclcpp::Node
{
public:
  QrVisionNode();

  ~QrVisionNode() override;

private:
  struct DecodedVisualCode
  {
    std::string code;
    std::string category;
    std::string symbol_type;
    double center_distance_sq{0.0};
  };

  struct CodeStabilityState
  {
    std::string candidate_code;
    std::string candidate_symbol_type;
    std::string stable_code;
    std::string stable_symbol_type;
    int candidate_count{0};
    int lost_count{0};
  };

  struct BpuImageRect
  {
    float x_min{0.0F};
    float y_min{0.0F};
    float x_max{0.0F};
    float y_max{0.0F};
  };

  struct BpuLetterboxState
  {
    float scale{1.0F};
    int pad_x{0};
    int pad_y{0};
  };

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>
      ColorDepthSyncPolicy;

  void declareParameters();

  void initializeSubscriptions();

  void initializeBpuDetector();

  void initializeBpuOcrPipeline();

  bool prepareBpuInput(const cv::Mat &color_image);

  void updateVisualCodeStability(const std::vector<DecodedVisualCode> &decoded_codes);

  void updateVisualCodeCategoryStability(
      const std::vector<DecodedVisualCode> &decoded_codes,
      const std::string &category,
      CodeStabilityState &state,
      const char *source_name);

  void handleSyncedFrame(
      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg);

  void handleRgbdFrame(
      const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr &rgbd_msg);

  void processFrame(
      const cv_bridge::CvImageConstPtr &color_bridge,
      const cv_bridge::CvImageConstPtr &depth_bridge,
      const std::chrono::steady_clock::time_point &callback_t0,
      const char *input_mode);

  void handleCameraInfo(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg);

  void displayDebugFrame(
      const cv::Mat &color_image,
      const DepthSampleResult &center_depth);

#if DRONE_PERCEPTION_HAS_BPU
  void drawBpuDetections(cv::Mat &display) const;

  BpuImageRect mapBpuDetectionToImageRect(
      const BpuYoloDetection &detection,
      int image_width,
      int image_height) const;

  BpuImageRect expandImageRect(
      const BpuImageRect &image_rect,
      float padding_ratio,
      int image_width,
      int image_height) const;

  void recognizeShelfTagFromDetections(
      const cv::Mat &color_image,
      const std::vector<BpuYoloDetection> &detections,
      const char *input_mode);

  std::vector<DecodedVisualCode> decodeVisualCodesFromDetections(
      const cv::Mat &color_image,
      const std::vector<BpuYoloDetection> &detections);

  void drawOcrRegions(cv::Mat &display) const;
#endif

  void updateFps();

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string rgbd_topic_;
  std::string window_name_;

  std::string bpu_model_path_;
  std::string ocr_rec_model_path_;

  std::vector<uint8_t> bpu_input_nv12_;
  BpuLetterboxState bpu_letterbox_;

  bool debug_view_ = true;
  bool enable_bpu_ = false;
  bool enable_bpu_ocr_ = false;
  bool use_rgbd_ = false;
  mutable bool debug_window_created_ = false;
  int log_throttle_ms_ = 500;
  int sample_radius_px_ = 10;
  int shelf_code_stable_frames_ = 3;
  int shelf_code_lost_tolerance_frames_ = 2;
  float ocr_yolo_padding_ratio_ = 0.15F;

  rclcpp::Time last_frame_time_;
  double smoothed_fps_ = 0.0;

  CodeStabilityState shelf_code_state_;
  CodeStabilityState sku_code_state_;
  CodeStabilityState pkg_code_state_;

  DepthProcessor depth_processor_;

#if DRONE_PERCEPTION_HAS_BPU
  std::unique_ptr<BpuYoloDetector> bpu_detector_;
  std::unique_ptr<BpuOcrPipeline> bpu_ocr_pipeline_;
  std::vector<BpuYoloDetection> last_bpu_detections_;
  std::vector<OcrTextRegion> last_ocr_regions_;
  std::string debug_raw_symbol_;
  std::string debug_raw_symbol_type_;
#endif

  bool has_camera_info_ = false;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<ColorDepthSyncPolicy>> color_depth_sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_sub_;
};
