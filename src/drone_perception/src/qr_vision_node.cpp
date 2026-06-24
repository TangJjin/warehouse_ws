#include "drone_perception/qr_vision_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
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
#include <zbar.h>

namespace
{
using SteadyClock = std::chrono::steady_clock;
static constexpr int kBpuInputWidthPx = 640;
static constexpr int kBpuInputHeightPx = 640;
static constexpr int kDefaultSampleRadiusPx = 10;
static constexpr int kDefaultShelfCodeStableFrames = 3;
static constexpr int kDefaultShelfCodeLostToleranceFrames = 2;
#if DRONE_PERCEPTION_HAS_BPU
static constexpr float kShelfCodeRoiPaddingRatio = 0.15F;
#endif
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

#if DRONE_PERCEPTION_HAS_BPU
std::string trimAndUppercase(const std::string &text)
{
  const auto begin = std::find_if_not(
      text.begin(),
      text.end(),
      [](unsigned char value) {
        return std::isspace(value) != 0;
      });

  const auto end = std::find_if_not(
      text.rbegin(),
      text.rend(),
      [](unsigned char value) {
        return std::isspace(value) != 0;
      }).base();

  if (begin >= end) {
    return {};
  }

  std::string normalized(begin, end);
  std::transform(
      normalized.begin(),
      normalized.end(),
      normalized.begin(),
      [](unsigned char value) {
        return static_cast<char>(std::toupper(value));
      });

  return normalized;
}

bool isAlphaString(const std::string &text)
{
  if (text.empty()) {
    return false;
  }

  return std::all_of(
      text.begin(),
      text.end(),
      [](unsigned char value) {
        return std::isalpha(value) != 0;
      });
}

bool isDigitString(const std::string &text)
{
  if (text.empty()) {
    return false;
  }

  return std::all_of(
      text.begin(),
      text.end(),
      [](unsigned char value) {
        return std::isdigit(value) != 0;
      });
}

bool isValidShelfCode(const std::string &code)
{
  const std::size_t first_dash = code.find('-');

  if (first_dash == std::string::npos) {
    return false;
  }

  const std::size_t second_dash = code.find('-', first_dash + 1U);

  if (second_dash == std::string::npos) {
    return false;
  }

  if (code.find('-', second_dash + 1U) != std::string::npos) {
    return false;
  }

  const std::string area = code.substr(0U, first_dash);
  const std::string row = code.substr(first_dash + 1U, second_dash - first_dash - 1U);
  const std::string col = code.substr(second_dash + 1U);

  return isAlphaString(area) && isDigitString(row) && isDigitString(col);
}
#endif
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
      "QR D435i video stream ready. mode=%s color=%s depth=%s rgbd=%s camera_info=%s",
      use_rgbd_ ? "rgbd" : "synced",
      color_topic_.c_str(),
      depth_topic_.c_str(),
      rgbd_topic_.c_str(),
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
  rgbd_topic_ = this->declare_parameter<std::string>(
      "rgbd_topic", "/camera/camera/rgbd");
  window_name_ = this->declare_parameter<std::string>(
      "window_name", "QR D435i View");
  debug_view_ = this->declare_parameter<bool>("debug_view", true);
  use_rgbd_ = this->declare_parameter<bool>("use_rgbd", false);
  log_throttle_ms_ = this->declare_parameter<int>("log_throttle_ms", 500);
  sample_radius_px_ = this->declare_parameter<int>(
      "sample_radius_px",
      kDefaultSampleRadiusPx);
  shelf_code_stable_frames_ = this->declare_parameter<int>(
      "shelf_code_stable_frames",
      kDefaultShelfCodeStableFrames);
  shelf_code_lost_tolerance_frames_ = this->declare_parameter<int>(
      "shelf_code_lost_tolerance_frames",
      kDefaultShelfCodeLostToleranceFrames);

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
        "sample_radius_px must be greater than or equal to 0, reset to %d",
        kDefaultSampleRadiusPx);
    sample_radius_px_ = kDefaultSampleRadiusPx;
  }

  if (shelf_code_stable_frames_ <= 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "shelf_code_stable_frames must be greater than 0, reset to %d",
        kDefaultShelfCodeStableFrames);
    shelf_code_stable_frames_ = kDefaultShelfCodeStableFrames;
  }

  if (shelf_code_lost_tolerance_frames_ < 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "shelf_code_lost_tolerance_frames must be greater than or equal to 0, reset to %d",
        kDefaultShelfCodeLostToleranceFrames);
    shelf_code_lost_tolerance_frames_ = kDefaultShelfCodeLostToleranceFrames;
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
    const float scale = std::min(
        static_cast<float>(kBpuInputWidthPx) / static_cast<float>(color_image.cols),
        static_cast<float>(kBpuInputHeightPx) / static_cast<float>(color_image.rows));
    const int resized_width = std::clamp(
        static_cast<int>(std::round(static_cast<float>(color_image.cols) * scale)),
        1,
        kBpuInputWidthPx);
    const int resized_height = std::clamp(
        static_cast<int>(std::round(static_cast<float>(color_image.rows) * scale)),
        1,
        kBpuInputHeightPx);
    const int pad_x = (kBpuInputWidthPx - resized_width) / 2;
    const int pad_y = (kBpuInputHeightPx - resized_height) / 2;

    cv::Mat resized_bgr;
    cv::resize(
        color_image,
        resized_bgr,
        cv::Size(resized_width, resized_height),
        0.0,
        0.0,
        cv::INTER_LINEAR);

    cv::Mat letterbox_bgr(
        kBpuInputHeightPx,
        kBpuInputWidthPx,
        color_image.type(),
        cv::Scalar(114, 114, 114));
    resized_bgr.copyTo(letterbox_bgr(cv::Rect(
        pad_x,
        pad_y,
        resized_width,
        resized_height)));

    bpu_letterbox_.scale = scale;
    bpu_letterbox_.pad_x = pad_x;
    bpu_letterbox_.pad_y = pad_y;

    cv::Mat yuv_i420;
    cv::cvtColor(letterbox_bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);

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

void QrVisionNode::updateShelfCodeStability(
    const std::vector<DecodedShelfCode> &decoded_codes)
{
  if (decoded_codes.empty()) {
    ++lost_shelf_code_count_;

    if (lost_shelf_code_count_ > shelf_code_lost_tolerance_frames_) {
      candidate_shelf_code_.clear();
      candidate_shelf_code_type_.clear();
      stable_shelf_code_.clear();
      stable_shelf_code_type_.clear();
      candidate_shelf_code_count_ = 0;
    }

    return;
  }

  lost_shelf_code_count_ = 0;

  const DecodedShelfCode &selected_code = decoded_codes.front();

  if (selected_code.code == candidate_shelf_code_) {
    ++candidate_shelf_code_count_;
  } else {
    candidate_shelf_code_ = selected_code.code;
    candidate_shelf_code_type_ = selected_code.type;
    candidate_shelf_code_count_ = 1;
  }

  if (candidate_shelf_code_count_ < shelf_code_stable_frames_) {
    return;
  }

  stable_shelf_code_ = candidate_shelf_code_;
  stable_shelf_code_type_ = candidate_shelf_code_type_;

  RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "stable shelf code type=%s code=%s stable_frames=%d",
      stable_shelf_code_type_.c_str(),
      stable_shelf_code_.c_str(),
      candidate_shelf_code_count_);
}

#if DRONE_PERCEPTION_HAS_BPU
std::vector<QrVisionNode::DecodedShelfCode> QrVisionNode::decodeShelfCodesFromDetections(
    const cv::Mat &color_image,
    const std::vector<BpuYoloDetection> &detections) const
{
  std::vector<DecodedShelfCode> decoded_codes;

  if (color_image.empty() || detections.empty()) {
    return decoded_codes;
  }

  zbar::ImageScanner scanner;
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

  const cv::Point2f image_center(
      static_cast<float>(color_image.cols) * 0.5F,
      static_cast<float>(color_image.rows) * 0.5F);

  for (const BpuYoloDetection &detection : detections) {
    const BpuImageRect image_rect = mapBpuDetectionToImageRect(
        detection,
        color_image.cols,
        color_image.rows);
    const float x_min = image_rect.x_min;
    const float y_min = image_rect.y_min;
    const float x_max = image_rect.x_max;
    const float y_max = image_rect.y_max;
    const float width = x_max - x_min;
    const float height = y_max - y_min;

    if (width <= 1.0F || height <= 1.0F) {
      continue;
    }

    const float pad_x = width * kShelfCodeRoiPaddingRatio;
    const float pad_y = height * kShelfCodeRoiPaddingRatio;
    const int roi_x_min = std::max(0, static_cast<int>(x_min - pad_x));
    const int roi_y_min = std::max(0, static_cast<int>(y_min - pad_y));
    const int roi_x_max = std::min(color_image.cols, static_cast<int>(x_max + pad_x));
    const int roi_y_max = std::min(color_image.rows, static_cast<int>(y_max + pad_y));
    const cv::Rect roi(
        roi_x_min,
        roi_y_min,
        roi_x_max - roi_x_min,
        roi_y_max - roi_y_min);

    if (roi.width <= 1 || roi.height <= 1) {
      continue;
    }

    cv::Mat gray_roi;
    cv::cvtColor(color_image(roi), gray_roi, cv::COLOR_BGR2GRAY);

    if (!gray_roi.isContinuous()) {
      gray_roi = gray_roi.clone();
    }

    zbar::Image zbar_image(
        static_cast<unsigned int>(gray_roi.cols),
        static_cast<unsigned int>(gray_roi.rows),
        "Y800",
        gray_roi.data,
        static_cast<unsigned long>(gray_roi.total()));

    scanner.scan(zbar_image);

    for (auto symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
      const std::string code = trimAndUppercase(symbol->get_data());

      if (!isValidShelfCode(code)) {
        continue;
      }

      cv::Point2f code_center(
          static_cast<float>(roi.x) + static_cast<float>(roi.width) * 0.5F,
          static_cast<float>(roi.y) + static_cast<float>(roi.height) * 0.5F);

      const int location_size = symbol->get_location_size();

      if (location_size > 0) {
        code_center = cv::Point2f(0.0F, 0.0F);

        for (int i = 0; i < location_size; ++i) {
          code_center.x += static_cast<float>(roi.x + symbol->get_location_x(i));
          code_center.y += static_cast<float>(roi.y + symbol->get_location_y(i));
        }

        code_center.x /= static_cast<float>(location_size);
        code_center.y /= static_cast<float>(location_size);
      }

      const float dx = code_center.x - image_center.x;
      const float dy = code_center.y - image_center.y;

      decoded_codes.push_back({
          code,
          symbol->get_type_name(),
          static_cast<double>(dx * dx + dy * dy)});
    }

    zbar_image.set_data(nullptr, 0U);
  }

  std::sort(
      decoded_codes.begin(),
      decoded_codes.end(),
      [](const DecodedShelfCode &a, const DecodedShelfCode &b) {
        return a.center_distance_sq < b.center_distance_sq;
      });

  return decoded_codes;
}
#endif

void QrVisionNode::initializeSubscriptions()
{
  if (use_rgbd_) {
    rgbd_sub_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
        rgbd_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
            &QrVisionNode::handleRgbdFrame,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Using RGBD input. rgbd=%s",
        rgbd_topic_.c_str());
    return;
  }

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

  RCLCPP_INFO(
      get_logger(),
      "Using synced image input. color=%s depth=%s camera_info=%s",
      color_topic_.c_str(),
      depth_topic_.c_str(),
      camera_info_topic_.c_str());
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

  processFrame(color_bridge, depth_bridge, callback_t0, "synced");
}

void QrVisionNode::handleRgbdFrame(
    const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr &rgbd_msg)
{
  const auto callback_t0 = SteadyClock::now();

  cv_bridge::CvImageConstPtr color_bridge;
  cv_bridge::CvImageConstPtr depth_bridge;

  try
  {
    color_bridge = cv_bridge::toCvShare(rgbd_msg->rgb, rgbd_msg, "bgr8");
    depth_bridge = cv_bridge::toCvShare(rgbd_msg->depth, rgbd_msg);
  }
  catch (const cv_bridge::Exception &ex)
  {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "cv_bridge RGBD conversion failed: %s",
        ex.what());
    return;
  }

  has_camera_info_ = true;
  processFrame(color_bridge, depth_bridge, callback_t0, "rgbd");
}

void QrVisionNode::processFrame(
    const cv_bridge::CvImageConstPtr &color_bridge,
    const cv_bridge::CvImageConstPtr &depth_bridge,
    const std::chrono::steady_clock::time_point &callback_t0,
    const char *input_mode)
{
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
            "BPU inference ok. mode=%s preprocess_ms=%.2f infer_ms=%.2f "
            "input_size=%zu detections=%zu letterbox_scale=%.4f letterbox_pad=%d,%d",
            input_mode,
            elapsedMs(preprocess_t0, preprocess_t1),
            elapsedMs(infer_t0, infer_t1),
            bpu_input_nv12_.size(),
            detections.size(),
            bpu_letterbox_.scale,
            bpu_letterbox_.pad_x,
            bpu_letterbox_.pad_y);
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

#if DRONE_PERCEPTION_HAS_BPU
  updateShelfCodeStability(
      decodeShelfCodesFromDetections(color_bridge->image, last_bpu_detections_));
#else
  updateShelfCodeStability({});
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
        "video stream mode=%s fps=%.1f size=%dx%d center_depth=%.3fm camera_info=%s "
        "callback_ms=%.2f debug_view=%s",
        input_mode,
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
        "video stream mode=%s fps=%.1f size=%dx%d center_depth=invalid camera_info=%s "
        "callback_ms=%.2f debug_view=%s",
        input_mode,
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
QrVisionNode::BpuImageRect QrVisionNode::mapBpuDetectionToImageRect(
    const BpuYoloDetection &detection,
    int image_width,
    int image_height) const
{
  if (bpu_letterbox_.scale <= 0.0F || image_width <= 0 || image_height <= 0) {
    return {};
  }

  const float image_width_f = static_cast<float>(image_width);
  const float image_height_f = static_cast<float>(image_height);
  const float pad_x = static_cast<float>(bpu_letterbox_.pad_x);
  const float pad_y = static_cast<float>(bpu_letterbox_.pad_y);

  const float x_min = (detection.x_min_px - pad_x) / bpu_letterbox_.scale;
  const float y_min = (detection.y_min_px - pad_y) / bpu_letterbox_.scale;
  const float x_max = (detection.x_max_px - pad_x) / bpu_letterbox_.scale;
  const float y_max = (detection.y_max_px - pad_y) / bpu_letterbox_.scale;

  return {
      std::clamp(std::min(x_min, x_max), 0.0F, image_width_f),
      std::clamp(std::min(y_min, y_max), 0.0F, image_height_f),
      std::clamp(std::max(x_min, x_max), 0.0F, image_width_f),
      std::clamp(std::max(y_min, y_max), 0.0F, image_height_f)};
}

void QrVisionNode::drawBpuDetections(cv::Mat &display) const
{
  for (const BpuYoloDetection &detection : last_bpu_detections_) {
    const BpuImageRect image_rect = mapBpuDetectionToImageRect(
        detection,
        display.cols,
        display.rows);

    if (image_rect.x_max - image_rect.x_min <= 1.0F ||
        image_rect.y_max - image_rect.y_min <= 1.0F) {
      continue;
    }

    const cv::Point top_left(
        static_cast<int>(image_rect.x_min),
        static_cast<int>(image_rect.y_min));
    const cv::Point bottom_right(
        static_cast<int>(image_rect.x_max),
        static_cast<int>(image_rect.y_max));

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
