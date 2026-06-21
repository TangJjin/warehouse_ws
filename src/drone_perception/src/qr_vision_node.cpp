#include "drone_perception/qr_vision_node.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
using SteadyClock = std::chrono::steady_clock;
static constexpr int kBpuInputWidthPx = 640;
static constexpr int kBpuInputHeightPx = 640;
static constexpr std::size_t kBpuInputYSize =
    static_cast<std::size_t>(kBpuInputWidthPx) *
    static_cast<std::size_t>(kBpuInputHeightPx);
static constexpr std::size_t kBpuInputUvSize = kBpuInputYSize / 4;
static constexpr std::size_t kBpuInputNv12Size = kBpuInputYSize + kBpuInputUvSize * 2;

double elapsedMs(
    const SteadyClock::time_point &start,
    const SteadyClock::time_point &end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}
}  // namespace

QrVisionNode::QrVisionNode()
    : Node("qr_vision_node")
{
  declareParameters();
  initializeBpuDetector();
  initializeSubscriptions();

  if (debug_view_)
  {
    try
    {
      cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      debug_window_created_ = true;
    }
    catch (const cv::Exception &e)
    {
      debug_view_ = false;
      RCLCPP_WARN(get_logger(), "Cannot open debug window: %s", e.what());
    }
  }

  RCLCPP_INFO(
      get_logger(),
      "QR D435i video stream ready. color=%s depth=%s camera_info=%s",
      color_topic_.c_str(),
      depth_topic_.c_str(),
      camera_info_topic_.c_str());
}

QrVisionNode::~QrVisionNode()
{
  if (debug_window_created_)
  {
    cv::destroyWindow(window_name_);
  }
}

void QrVisionNode::declareParameters()
{
  color_topic_ = this->declare_parameter<std::string>(
      "color_topic", "/camera/camera/color/image_raw");
  depth_topic_ = this->declare_parameter<std::string>(
      "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/color/camera_info");
  window_name_ = this->declare_parameter<std::string>(
      "window_name", "QR D435i View");
  debug_view_ = this->declare_parameter<bool>("debug_view", true);
  log_throttle_ms_ = this->declare_parameter<int>("log_throttle_ms", 500);
  sample_radius_px_ = this->declare_parameter<int>("sample_radius_px", 3);

  enable_bpu_ = this->declare_parameter<bool>(
      "enable_bpu",
      DRONE_PERCEPTION_HAS_BPU != 0);
  bpu_model_path_ = this->declare_parameter<std::string>(
      "bpu_model_path",
      "/home/sunrise/drone_ws/src/drone_perception/rdk_best_bayese_640x640_nv12.bin");

  if (log_throttle_ms_ <= 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "log_throttle_ms must be greater than 0, reset to 500");
    log_throttle_ms_ = 500;
  }

  if (sample_radius_px_ < 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "sample_radius_px must be greater than or equal to 0, reset to 3");
    sample_radius_px_ = 3;
  }
}

void QrVisionNode::initializeBpuDetector()
{
#if DRONE_PERCEPTION_HAS_BPU
  if (!enable_bpu_) {
    return;
  }

  if (bpu_model_path_.empty()) {
    RCLCPP_WARN(
        get_logger(),
        "BPU inference is enabled but bpu_model_path is empty");
    enable_bpu_ = false;
    return;
  }

  try {
    bpu_detector_ = std::make_unique<BpuYoloDetector>(bpu_model_path_);

    RCLCPP_INFO(
        get_logger(),
        "BPU detector initialized. model=%s",
        bpu_model_path_.c_str());
  } catch (const std::exception &e) {
    bpu_detector_.reset();
    enable_bpu_ = false;

    RCLCPP_ERROR(
        get_logger(),
        "Failed to initialize BPU detector: %s",
        e.what());
  }
#else
  if (enable_bpu_) {
    RCLCPP_WARN(
        get_logger(),
        "BPU inference requested but this build has no RDK BPU SDK");
    enable_bpu_ = false;
  }
#endif
}

bool QrVisionNode::prepareBpuInput(const cv::Mat &color_image)
{
  if (color_image.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "BPU input image is empty");

    return false;
  }

  if (color_image.channels() != 3) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "BPU input expects BGR image with 3 channels, got %d",
        color_image.channels());

    return false;
  }

  try {
    cv::Mat resized_bgr;
    cv::resize(
        color_image,
        resized_bgr,
        cv::Size(kBpuInputWidthPx, kBpuInputHeightPx),
        0.0,
        0.0,
        cv::INTER_LINEAR);

    cv::Mat yuv_i420;
    cv::cvtColor(resized_bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);

    if (!yuv_i420.isContinuous()) {
      yuv_i420 = yuv_i420.clone();
    }

    bpu_input_nv12_.resize(kBpuInputNv12Size);

    const uint8_t *y_plane = yuv_i420.ptr<uint8_t>(0);
    const uint8_t *u_plane = y_plane + kBpuInputYSize;
    const uint8_t *v_plane = u_plane + kBpuInputUvSize;
    uint8_t *nv12_data = bpu_input_nv12_.data();

    std::memcpy(nv12_data, y_plane, kBpuInputYSize);

    uint8_t *uv_plane = nv12_data + kBpuInputYSize;

    for (std::size_t i = 0; i < kBpuInputUvSize; ++i) {
      uv_plane[i * 2] = u_plane[i];
      uv_plane[i * 2 + 1] = v_plane[i];
    }
  } catch (const cv::Exception &e) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "Failed to prepare BPU input: %s",
        e.what());

    return false;
  }

  return true;
}

void QrVisionNode::initializeSubscriptions()
{
  color_sub_.subscribe(this, color_topic_, rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);

  color_depth_sync_ = std::make_shared<message_filters::Synchronizer<ColorDepthSyncPolicy>>(
      ColorDepthSyncPolicy(10),
      color_sub_,
      depth_sub_);
  color_depth_sync_->registerCallback(std::bind(
      &QrVisionNode::handleSyncedFrame,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
          &QrVisionNode::handleCameraInfo,
          this,
          std::placeholders::_1));
}

void QrVisionNode::handleCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg)
{
  (void)camera_info_msg;
  has_camera_info_ = true;
}

void QrVisionNode::updateFps()
{
  const rclcpp::Time current_frame_time = this->now();
  if (last_frame_time_.nanoseconds() > 0)
  {
    const double frame_interval_s = (current_frame_time - last_frame_time_).seconds();
    if (frame_interval_s > 0.0)
    {
      const double current_fps = 1.0 / frame_interval_s;
      smoothed_fps_ = smoothed_fps_ <= 0.0 ? current_fps : smoothed_fps_ * 0.9 + current_fps * 0.1;
    }
  }
  last_frame_time_ = current_frame_time;
}

void QrVisionNode::handleSyncedFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
{
  const auto callback_t0 = SteadyClock::now();

  cv_bridge::CvImageConstPtr color_bridge;
  cv_bridge::CvImageConstPtr depth_bridge;

  try
  {
    color_bridge = cv_bridge::toCvShare(color_msg, "bgr8");
    depth_bridge = cv_bridge::toCvShare(depth_msg);
  }
  catch (const cv_bridge::Exception &ex)
  {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "cv_bridge synced conversion failed: %s",
        ex.what());
    return;
  }

  if (color_bridge->image.cols != depth_bridge->image.cols ||
      color_bridge->image.rows != depth_bridge->image.rows)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "Color and depth image sizes do not match: color=%dx%d depth=%dx%d",
        color_bridge->image.cols,
        color_bridge->image.rows,
        depth_bridge->image.cols,
        depth_bridge->image.rows);
    return;
  }

  updateFps();

  const int center_u = color_bridge->image.cols / 2;
  const int center_v = color_bridge->image.rows / 2;
  const DepthSampleResult center_depth = depth_processor_.sampleAt(
      depth_bridge->image,
      center_u,
      center_v,
      sample_radius_px_);

#if DRONE_PERCEPTION_HAS_BPU
  if (enable_bpu_ && bpu_detector_) {
    last_bpu_detections_.clear();

    const auto preprocess_t0 = SteadyClock::now();
    const bool input_ready = prepareBpuInput(color_bridge->image);
    const auto preprocess_t1 = SteadyClock::now();

    if (input_ready) {
      try {
        const auto infer_t0 = SteadyClock::now();

        const auto detections = bpu_detector_->inferNv12(
            bpu_input_nv12_.data(),
            bpu_input_nv12_.size());
        last_bpu_detections_ = detections;

        const auto infer_t1 = SteadyClock::now();

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "BPU inference ok. preprocess_ms=%.2f infer_ms=%.2f input_size=%zu detections=%zu",
            elapsedMs(preprocess_t0, preprocess_t1),
            elapsedMs(infer_t0, infer_t1),
            bpu_input_nv12_.size(),
            detections.size());
      } catch (const std::exception &e) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "BPU inference failed: %s",
            e.what());
      }
    }
  }
#endif

  if (debug_view_)
  {
    displayDebugFrame(color_bridge->image, center_depth);
  }

  const auto callback_t1 = SteadyClock::now();
  if (center_depth.has_valid_depth)
  {
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "video stream fps=%.1f size=%dx%d center_depth=%.3fm camera_info=%s "
        "callback_ms=%.2f debug_view=%s",
        smoothed_fps_,
        color_bridge->image.cols,
        color_bridge->image.rows,
        center_depth.depth_m,
        has_camera_info_ ? "true" : "false",
        elapsedMs(callback_t0, callback_t1),
        debug_view_ ? "true" : "false");
  }
  else
  {
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "video stream fps=%.1f size=%dx%d center_depth=invalid camera_info=%s "
        "callback_ms=%.2f debug_view=%s",
        smoothed_fps_,
        color_bridge->image.cols,
        color_bridge->image.rows,
        has_camera_info_ ? "true" : "false",
        elapsedMs(callback_t0, callback_t1),
        debug_view_ ? "true" : "false");
  }
}

void QrVisionNode::displayDebugFrame(
    const cv::Mat &color_image,
    const DepthSampleResult &center_depth)
{
  cv::Mat display = color_image.clone();

  cv::Mat overlay = display.clone();
  cv::rectangle(overlay, cv::Point(12, 12), cv::Point(320, 98), cv::Scalar(30, 30, 30), cv::FILLED);
  cv::addWeighted(overlay, 0.45, display, 0.55, 0.0, display);

  cv::putText(
      display,
      cv::format("FPS: %.1f", smoothed_fps_),
      cv::Point(24, 38),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(0, 255, 255),
      1);
  cv::putText(
      display,
      cv::format("Size: %dx%d", color_image.cols, color_image.rows),
      cv::Point(24, 60),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(255, 255, 255),
      1);
  cv::putText(
      display,
      center_depth.has_valid_depth
        ? std::string(cv::format("Center depth: %.3fm", center_depth.depth_m))
        : std::string("Center depth: invalid"),
      cv::Point(24, 82),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(180, 255, 180),
      1);

#if DRONE_PERCEPTION_HAS_BPU
  drawBpuDetections(display);
#endif

  cv::imshow(window_name_, display);
  cv::waitKey(1);
}

#if DRONE_PERCEPTION_HAS_BPU
void QrVisionNode::drawBpuDetections(cv::Mat &display) const
{
  const float scale_x =
      static_cast<float>(display.cols) / static_cast<float>(kBpuInputWidthPx);
  const float scale_y =
      static_cast<float>(display.rows) / static_cast<float>(kBpuInputHeightPx);

  for (const BpuYoloDetection &detection : last_bpu_detections_) {
    const cv::Point top_left(
        static_cast<int>(detection.x_min_px * scale_x),
        static_cast<int>(detection.y_min_px * scale_y));
    const cv::Point bottom_right(
        static_cast<int>(detection.x_max_px * scale_x),
        static_cast<int>(detection.y_max_px * scale_y));

    cv::rectangle(
        display,
        top_left,
        bottom_right,
        cv::Scalar(0, 255, 0),
        2);

    const std::string label = cv::format(
        "cls%d %.2f",
        detection.class_id,
        detection.score);
    const int label_y = top_left.y > 18 ? top_left.y - 6 : top_left.y + 18;

    cv::putText(
        display,
        label,
        cv::Point(top_left.x, label_y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 255, 0),
        1);
  }
}
#endif
