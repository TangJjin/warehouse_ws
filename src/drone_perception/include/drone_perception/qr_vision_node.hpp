#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/types.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include "drone_msgs/msg/barcode_capture.hpp"

namespace zbar
{
class ImageScanner;
}

namespace cv
{
class CLAHE;
}

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

  struct BpuPreprocessTimingStats
  {
    double resize_ms{0.0};
    double letterbox_ms{0.0};
    double color_convert_ms{0.0};
    double nv12_pack_ms{0.0};
    double total_ms{0.0};
  };

  struct VisualCodeTimingStats
  {
    double gray_ms{0.0};
    double raw_zbar_ms{0.0};
    double clahe_ms{0.0};
    double clahe_zbar_ms{0.0};
    double adaptive_ms{0.0};
    double adaptive_zbar_ms{0.0};
    int roi_count{0};
    int raw_scan_count{0};
    int clahe_scan_count{0};
    int adaptive_scan_count{0};
  };

  struct TimingWindow
  {
    void push(double value);
    double percentile(double ratio) const;

    static constexpr std::size_t kCapacity = 120U;
    std::deque<double> samples;
  };

  struct PendingColorFrame
  {
    std::uint64_t sequence{0U};
    sensor_msgs::msg::Image::ConstSharedPtr message;
    std::chrono::steady_clock::time_point enqueue_time{};
  };

  struct PendingHoverEvent
  {
    std::uint64_t sequence{0U};
    bool active{false};
  };

  struct CaptureFrameCandidate
  {
    cv::Mat image;
    rclcpp::Time stamp{0, 0, RCL_SYSTEM_TIME};
    float package_score{0.0F};
    double center_distance_sq{0.0};
    bool valid{false};
  };

  struct CameraNumericControl
  {
    std::string parameter_name;
    std::string label;
    int fallback_min{0};
    int fallback_max{100};
    int fallback_default{0};
    int min_value{0};
    int max_value{100};
    int current_value{0};
    int last_sent_value{0};
    int trackbar_value{0};
    rclcpp::ParameterType parameter_type{rclcpp::ParameterType::PARAMETER_INTEGER};
    bool enabled{true};
    bool send_pending{false};
    int restore_value{0};
    bool restore_value_initialized{false};
  };

  struct CameraBoolControl
  {
    std::string parameter_name;
    std::string label;
    bool current_value{false};
    bool last_sent_value{false};
    bool enabled{true};
    bool send_pending{false};
    bool restore_value{false};
    bool restore_value_initialized{false};
    cv::Rect true_rect{};
    cv::Rect false_rect{};
  };

  struct CameraTrackbarContext
  {
    QrVisionNode *node{nullptr};
    std::size_t index{0U};
  };

  static constexpr std::size_t kCameraNumericControlCount = 3U;
  static constexpr std::size_t kCameraBoolControlCount = 3U;

  void declareParameters();

  void initializeSubscriptions();

  void initializeBpuDetector();

  void initializeBpuOcrPipeline();

  void initializeVisualCodeDecoder();

  void startVisionWorker();

  void stopVisionWorker();

  void visionWorkerLoop();

  bool prepareBpuInput(const cv::Mat &color_image);

  void updateVisualCodeStability(const std::vector<DecodedVisualCode> &decoded_codes);

  std::string composeCaptureBarcode(bool require_complete) const;

  std::string composeCaptureBarcodeWithNan(bool allow_all_nan) const;

  void handleHoverActive(const std_msgs::msg::Bool::SharedPtr msg);

  void applyHoverActive(bool hover_active);

  void resetHoverCaptureState();

  void publishBarcodeCapture(const cv::Mat &color_image);

  void publishTextOnlyCapture(const std::string &barcode);

  bool publishFullFrameCapture(
      const std::string &barcode,
      const cv::Mat &color_image);

  void updateVisualCodeCategoryStability(
      const std::vector<DecodedVisualCode> &decoded_codes,
      const std::string &category,
      CodeStabilityState &state,
      const char *source_name);

  void handleColorFrame(
      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg);

  void processFrame(
      const cv_bridge::CvImageConstPtr &color_bridge,
      const std::chrono::steady_clock::time_point &process_t0,
      const char *input_mode);

  void updateFrameAge(const sensor_msgs::msg::Image &color_msg);

  void handleCameraInfo(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg);

  void displayDebugFrame(const cv::Mat &color_image);

  void initializeCameraControls();

  void initializeCameraControlDefaults();

  void drawCameraControlsPanel(cv::Mat &display);

  void updateCameraControlClient();

  void requestCameraControlDescriptors();

  void requestCameraControlValues();

  void applyCameraControlDescriptors(
      const std::vector<rcl_interfaces::msg::ParameterDescriptor> &descriptors);

  void applyCameraControlValues(
      const std::vector<rclcpp::Parameter> &parameters);

  void updateCameraControlEnableStates();

  void resetCameraControlsToDefaults();

  void sendPendingCameraControlParameters();

  void handleCameraParameterSetResults(
      const std::vector<rclcpp::Parameter> &parameters,
      const std::vector<rcl_interfaces::msg::SetParametersResult> &results);

  void handleCameraTrackbarChange(std::size_t index, int position);

  void handleCameraControlsClick(int x, int y);

  void setCameraBoolControl(std::size_t index, bool value);

  void updateCameraTrackbarPosition(std::size_t index);

  void drawCameraBoolControl(
      cv::Mat &panel,
      CameraBoolControl &control,
      int y);

  void drawCameraResetButton(cv::Mat &panel);

  static void handleCameraTrackbarCallback(int position, void *userdata);

  static void handleCameraControlsMouseCallback(
      int event,
      int x,
      int y,
      int flags,
      void *userdata);

#if DRONE_PERCEPTION_HAS_BPU
  void drawBpuDetections(cv::Mat &display) const;

  void drawBpuPerformanceHud(cv::Mat &display);

  void resetCaptureCandidateState();

  void bufferPackageCaptureCandidate(const cv::Mat &color_image);

  bool publishBestBufferedCapture();

  bool selectBestPackageInDeadzone(
      const cv::Mat &color_image,
      CaptureFrameCandidate &candidate) const;

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

  bool encodeFrameCaptureJpeg(
      const cv::Mat &color_image,
      std::vector<uint8_t> &jpeg_data);

  void drawOcrRegions(cv::Mat &display) const;

  void drawQrPreprocessPreview(cv::Mat &display) const;
#endif

  void updateFps();

  void reportPerformanceBaseline();

  std::string color_topic_;
  std::string camera_info_topic_;
  std::string window_name_;
  std::string camera_param_node_;
  std::string barcode_capture_topic_;
  std::string hover_active_topic_;

  std::string bpu_model_path_;
  std::string ocr_rec_model_path_;

  std::vector<uint8_t> bpu_input_nv12_;
  BpuLetterboxState bpu_letterbox_;
  BpuPreprocessTimingStats bpu_preprocess_timing_;
  VisualCodeTimingStats visual_code_timing_;

  bool debug_view_ = true;
  bool enable_bpu_ = false;
  bool enable_bpu_ocr_ = false;
  bool qr_preprocess_enabled_ = true;
  bool hover_active_ = false;
  bool full_capture_sent_in_hover_ = false;
  bool camera_controls_enabled_ = false;
  mutable bool debug_window_created_ = false;
  bool camera_controls_attached_ = false;
  bool camera_controls_updating_trackbar_ = false;
  bool camera_param_node_online_ = false;
  bool camera_param_describe_requested_ = false;
  bool camera_param_describe_in_flight_ = false;
  bool camera_param_get_requested_ = false;
  bool camera_param_get_in_flight_ = false;
  bool camera_param_set_in_flight_ = false;
  int log_throttle_ms_ = 500;
  int camera_controls_update_period_ms_ = 200;
  int shelf_code_stable_frames_ = 3;
  int shelf_code_lost_tolerance_frames_ = 2;
  int barcode_capture_jpeg_quality_ = 90;
  float ocr_yolo_padding_ratio_ = 0.15F;
  float package_capture_padding_ratio_ = 0.05F;
  double capture_collection_duration_s_ = 2.0;
  double package_capture_deadzone_ratio_ = 0.08;
  double package_capture_min_score_ = 0.50;

  rclcpp::Time last_frame_time_;
  double smoothed_fps_ = 0.0;
  double last_frame_age_ms_ = -1.0;
  double last_callback_ms_ = 0.0;
  double last_process_ms_ = 0.0;
  double last_queue_wait_ms_ = 0.0;
  double last_input_fps_ = 0.0;
  double last_process_fps_ = 0.0;
  std::chrono::steady_clock::time_point baseline_report_time_{};
  std::atomic<std::uint64_t> input_frame_count_{0U};
  std::atomic<std::uint64_t> dropped_frame_count_{0U};
  std::uint64_t processed_frame_count_{0U};
  std::uint64_t failed_frame_count_{0U};
  std::uint64_t baseline_last_input_count_{0U};
  std::uint64_t baseline_last_dropped_count_{0U};
  std::uint64_t baseline_last_processed_count_{0U};
  std::uint64_t baseline_last_failed_count_{0U};
  TimingWindow frame_age_window_;
  TimingWindow callback_window_;
  TimingWindow process_window_;
  TimingWindow queue_wait_window_;
  std::mutex performance_mutex_;
  std::string last_published_package_barcode_;

  CodeStabilityState shelf_code_state_;
  CodeStabilityState sku_code_state_;
  CodeStabilityState pkg_code_state_;

  rclcpp::AsyncParametersClient::SharedPtr camera_param_client_;
  rclcpp::TimerBase::SharedPtr camera_controls_timer_;
  std::array<CameraNumericControl, kCameraNumericControlCount> camera_numeric_controls_{};
  std::array<CameraBoolControl, kCameraBoolControlCount> camera_bool_controls_{};
  std::array<CameraTrackbarContext, kCameraNumericControlCount> camera_trackbar_contexts_{};
  cv::Rect camera_controls_panel_rect_{};
  cv::Rect camera_reset_button_rect_{};
  rclcpp::Time last_camera_param_send_time_;
  std::string camera_controls_status_text_;

#if DRONE_PERCEPTION_HAS_BPU
  std::unique_ptr<BpuYoloDetector> bpu_detector_;
  std::unique_ptr<BpuOcrPipeline> bpu_ocr_pipeline_;
  std::vector<BpuYoloDetection> last_bpu_detections_;
  std::vector<OcrTextRegion> last_ocr_regions_;
  std::string debug_raw_symbol_;
  std::string debug_raw_symbol_type_;
  cv::Mat debug_qr_preprocess_preview_;
  std::string debug_qr_preprocess_mode_;
  rclcpp::Time hover_capture_start_time_;
  CaptureFrameCandidate best_package_capture_candidate_;
  int package_capture_candidate_count_{0};
  bool ocr_invoked_this_frame_{false};
  std::unique_ptr<zbar::ImageScanner> visual_code_scanner_;
  cv::Ptr<cv::CLAHE> qr_clahe_;
#endif

  std::atomic_bool has_camera_info_{false};

  std::mutex vision_worker_mutex_;
  std::condition_variable vision_worker_cv_;
  std::thread vision_worker_;
  PendingColorFrame latest_color_frame_;
  std::deque<PendingHoverEvent> pending_hover_events_;
  std::uint64_t next_work_sequence_{1U};
  bool vision_worker_running_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hover_active_sub_;
  rclcpp::Publisher<drone_msgs::msg::BarcodeCapture>::SharedPtr barcode_capture_pub_;
};
