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
static constexpr float kVisualCodeRoiPaddingXRatio = 0.45F;
static constexpr float kVisualCodeRoiPaddingYRatio = 0.20F;
static constexpr double kVisualCodeRetryScale = 2.0;

// Barcode preprocessing constants (Mou Jiacun pipeline)
static constexpr int kBarcodeSobelKsize = -1;
static constexpr int kBarcodeMeanBlurSize = 9;
static constexpr int kBarcodeBinaryThreshold = 200;
static constexpr int kBarcodeMorphWidth = 22;
static constexpr int kBarcodeMorphHeight = 8;
static constexpr int kBarcodeMorphIterations = 6;
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
struct ParsedVisualCode
{
  std::string category;
  std::string code;
};

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

bool isValidShortPrefixedCode(
    const std::string &code,
    const std::string &prefix)
{
  if (code.rfind(prefix, 0U) != 0U || code.size() <= prefix.size()) {
    return false;
  }

  bool has_payload = false;

  for (std::size_t i = prefix.size(); i < code.size(); ++i) {
    const unsigned char value = static_cast<unsigned char>(code[i]);

    if (std::isalnum(value) != 0 || code[i] == '_') {
      has_payload = true;
      continue;
    }

    return false;
  }

  return has_payload;
}

void appendParsedVisualCode(
    const std::string &code,
    std::vector<ParsedVisualCode> &parsed_codes)
{
  if (isValidShelfCode(code)) {
    parsed_codes.push_back({"shelf", code});
    return;
  }

  if (isValidShortPrefixedCode(code, "SKU-")) {
    parsed_codes.push_back({"sku", code});
    return;
  }

  if (isValidShortPrefixedCode(code, "PKG-")) {
    parsed_codes.push_back({"pkg", code});
    return;
  }
}

std::vector<ParsedVisualCode> parseVisualCodes(const std::string &code)
{
  std::vector<ParsedVisualCode> parsed_codes;
  std::size_t begin = 0U;

  while (begin <= code.size()) {
    const std::size_t separator = code.find('|', begin);
    const std::size_t end =
        separator == std::string::npos ? code.size() : separator;
    const std::string token = trimAndUppercase(code.substr(begin, end - begin));

    if (!token.empty()) {
      appendParsedVisualCode(token, parsed_codes);
    }

    if (separator == std::string::npos) {
      break;
    }

    begin = separator + 1U;
  }

  return parsed_codes;
}

std::string formatParsedVisualCodeCategories(
    const std::vector<ParsedVisualCode> &parsed_codes)
{
  if (parsed_codes.empty()) {
    return "ignored";
  }

  std::string categories;

  for (const ParsedVisualCode &parsed_code : parsed_codes) {
    if (!categories.empty()) {
      categories += ",";
    }

    categories += parsed_code.category;
  }

  return categories;
}

void preprocessBarcodeRoi(cv::Mat &gray_roi)
{
  // Sobel X−Y gradient: enhance horizontal stripe edges, suppress non-barcode textures
  cv::Mat grad_x, grad_y;
  cv::Sobel(gray_roi, grad_x, CV_32F, 1, 0, kBarcodeSobelKsize);
  cv::Sobel(gray_roi, grad_y, CV_32F, 0, 1, kBarcodeSobelKsize);
  cv::Mat gradient = cv::abs(grad_x) - cv::abs(grad_y);
  cv::convertScaleAbs(gradient, gray_roi);

  // Mean blur: suppress high-frequency noise from gradient operation
  cv::blur(gray_roi, gray_roi, cv::Size(kBarcodeMeanBlurSize, kBarcodeMeanBlurSize));

  // Binary threshold: isolate barcode stripes into clean foreground / background
  cv::threshold(gray_roi, gray_roi, kBarcodeBinaryThreshold, 255, cv::THRESH_BINARY);

  // Morphological close with horizontal kernel: fill gaps within barcode stripes
  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(kBarcodeMorphWidth, kBarcodeMorphHeight));
  cv::morphologyEx(gray_roi, gray_roi, cv::MORPH_CLOSE, kernel);

  // Erode then dilate: remove isolated noise blobs while preserving stripe dimensions
  cv::erode(gray_roi, gray_roi, cv::Mat(), cv::Point(-1, -1), kBarcodeMorphIterations);
  cv::dilate(gray_roi, gray_roi, cv::Mat(), cv::Point(-1, -1), kBarcodeMorphIterations);
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
      "QR D435i video stream ready. mode=%s decode_mode=%s color=%s depth=%s rgbd=%s camera_info=%s",
      use_rgbd_ ? "rgbd" : "synced",
      use_barcode_format_ ? "barcode" : "qr",
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
  use_barcode_format_ = this->declare_parameter<bool>("use_barcode_format", false);
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

void QrVisionNode::updateVisualCodeStability(
    const std::vector<DecodedVisualCode> &decoded_codes)
{
  updateVisualCodeCategoryStability(decoded_codes, "shelf", shelf_code_state_);
  updateVisualCodeCategoryStability(decoded_codes, "sku", sku_code_state_);
  updateVisualCodeCategoryStability(decoded_codes, "pkg", pkg_code_state_);
}

void QrVisionNode::updateVisualCodeCategoryStability(
    const std::vector<DecodedVisualCode> &decoded_codes,
    const std::string &category,
    CodeStabilityState &state)
{
  const auto selected_code = std::find_if(
      decoded_codes.begin(),
      decoded_codes.end(),
      [&category](const DecodedVisualCode &decoded_code) {
        return decoded_code.category == category;
      });

  if (selected_code == decoded_codes.end()) {
    ++state.lost_count;

    if (state.lost_count > shelf_code_lost_tolerance_frames_) {
      state.candidate_code.clear();
      state.candidate_symbol_type.clear();
      state.stable_code.clear();
      state.stable_symbol_type.clear();
      state.candidate_count = 0;
    }

    return;
  }

  state.lost_count = 0;

  if (selected_code->code == state.candidate_code) {
    ++state.candidate_count;
  } else {
    state.candidate_code = selected_code->code;
    state.candidate_symbol_type = selected_code->symbol_type;
    state.candidate_count = 1;
  }

  if (state.candidate_count < shelf_code_stable_frames_) {
    return;
  }

  state.stable_code = state.candidate_code;
  state.stable_symbol_type = state.candidate_symbol_type;

  RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "stable visual code category=%s symbol_type=%s code=%s stable_frames=%d",
      category.c_str(),
      state.stable_symbol_type.c_str(),
      state.stable_code.c_str(),
      state.candidate_count);
}

#if DRONE_PERCEPTION_HAS_BPU
std::vector<QrVisionNode::DecodedVisualCode> QrVisionNode::decodeVisualCodesFromDetections(
    const cv::Mat &color_image,
    const std::vector<BpuYoloDetection> &detections)
{
  std::vector<DecodedVisualCode> decoded_codes;

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

    const float pad_x = width * kVisualCodeRoiPaddingXRatio;
    const float pad_y = height * kVisualCodeRoiPaddingYRatio;
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

    if (use_barcode_format_) {
      preprocessBarcodeRoi(gray_roi);
    }

    auto scan_gray_roi = [&](
        const cv::Mat &scan_roi,
        float coordinate_scale,
        const char *scan_mode) {
      cv::Mat continuous_roi = scan_roi;

      if (!continuous_roi.isContinuous()) {
        continuous_roi = continuous_roi.clone();
      }

      zbar::Image zbar_image(
          static_cast<unsigned int>(continuous_roi.cols),
          static_cast<unsigned int>(continuous_roi.rows),
          "Y800",
          continuous_roi.data,
          static_cast<unsigned long>(continuous_roi.total()));

      const int raw_count = scanner.scan(zbar_image);
      int accepted_count = 0;

      if (raw_count <= 0) {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "zbar no code scan=%s roi=%dx%d at=%d,%d detection_score=%.3f",
            scan_mode,
            continuous_roi.cols,
            continuous_roi.rows,
            roi.x,
            roi.y,
            detection.score);
      }

      for (auto symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
        const std::string raw_code = symbol->get_data();
        const std::string code = trimAndUppercase(raw_code);
        const std::string symbol_type = symbol->get_type_name();
        const bool is_qr_code = symbol_type.find("QR") != std::string::npos;
        const bool symbol_matches_mode =
            use_barcode_format_ ? !is_qr_code : is_qr_code;
        const std::vector<ParsedVisualCode> parsed_codes = parseVisualCodes(code);
        const std::string categories = formatParsedVisualCodeCategories(parsed_codes);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "zbar raw code scan=%s decode_mode=%s symbol_type=%s mode_match=%s raw=%s normalized=%s category=%s roi=%dx%d at=%d,%d",
            scan_mode,
            use_barcode_format_ ? "barcode" : "qr",
            symbol_type.c_str(),
            symbol_matches_mode ? "true" : "false",
            raw_code.c_str(),
            code.c_str(),
            categories.c_str(),
            continuous_roi.cols,
            continuous_roi.rows,
            roi.x,
            roi.y);

        if (!symbol_matches_mode || parsed_codes.empty()) {
          continue;
        }

        cv::Point2f code_center(
            static_cast<float>(roi.x) + static_cast<float>(roi.width) * 0.5F,
            static_cast<float>(roi.y) + static_cast<float>(roi.height) * 0.5F);

        const int location_size = symbol->get_location_size();

        if (location_size > 0) {
          code_center = cv::Point2f(0.0F, 0.0F);

          for (int i = 0; i < location_size; ++i) {
            code_center.x += static_cast<float>(roi.x) +
                static_cast<float>(symbol->get_location_x(i)) / coordinate_scale;
            code_center.y += static_cast<float>(roi.y) +
                static_cast<float>(symbol->get_location_y(i)) / coordinate_scale;
          }

          code_center.x /= static_cast<float>(location_size);
          code_center.y /= static_cast<float>(location_size);
        }

        const float dx = code_center.x - image_center.x;
        const float dy = code_center.y - image_center.y;

        for (const ParsedVisualCode &parsed_code : parsed_codes) {
          decoded_codes.push_back({
              parsed_code.code,
              parsed_code.category,
              symbol_type,
              static_cast<double>(dx * dx + dy * dy)});
          ++accepted_count;
        }
      }

      zbar_image.set_data(nullptr, 0U);

      return accepted_count;
    };

    const int accepted_count = scan_gray_roi(gray_roi, 1.0F, "original");

    if (accepted_count <= 0) {
      cv::Mat resized_gray_roi;
      cv::resize(
          gray_roi,
          resized_gray_roi,
          cv::Size(),
          kVisualCodeRetryScale,
          kVisualCodeRetryScale,
          cv::INTER_LINEAR);
      scan_gray_roi(
          resized_gray_roi,
          static_cast<float>(kVisualCodeRetryScale),
          "scale2");
    }
  }

  std::sort(
      decoded_codes.begin(),
      decoded_codes.end(),
      [](const DecodedVisualCode &a, const DecodedVisualCode &b) {
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
  updateVisualCodeStability(
      decodeVisualCodesFromDetections(color_bridge->image, last_bpu_detections_));
#else
  updateVisualCodeStability({});
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
  const int panel_right = std::min(display.cols - 12, 620);
  cv::rectangle(
      overlay,
      cv::Point(12, 12),
      cv::Point(panel_right, 164),
      cv::Scalar(30, 30, 30),
      cv::FILLED);
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

  auto fit_status_text = [](std::string text) {
    static constexpr std::size_t kMaxStatusTextLength = 58U;

    if (text.size() > kMaxStatusTextLength) {
      text = text.substr(0U, kMaxStatusTextLength - 3U) + "...";
    }

    return text;
  };

  auto draw_visual_code_line = [&](
      const CodeStabilityState &state,
      const char *category,
      int y) {
    if (state.stable_code.empty()) {
      return y;
    }

    const bool is_qr_code =
        state.stable_symbol_type.find("QR") != std::string::npos;
    const char *kind = is_qr_code ? "QR" : "Barcode";
    const std::string line = fit_status_text(cv::format(
        "%s %s: %s",
        kind,
        category,
        state.stable_code.c_str()));

    cv::putText(
        display,
        line,
        cv::Point(24, y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        is_qr_code ? cv::Scalar(120, 220, 255) : cv::Scalar(120, 255, 160),
        1);

    return y + 22;
  };

  int visual_code_y = 106;
  visual_code_y = draw_visual_code_line(shelf_code_state_, "shelf", visual_code_y);
  visual_code_y = draw_visual_code_line(sku_code_state_, "sku", visual_code_y);
  (void)draw_visual_code_line(pkg_code_state_, "pkg", visual_code_y);

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
