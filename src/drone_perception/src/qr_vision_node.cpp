#include "drone_perception/qr_vision_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

QrVisionNode::QrVisionNode()
    : Node("qr_vision_node")
{
  declareParameters();

  try
  {
    detector_ = std::make_unique<RknnYoloDetector>(model_path_);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_logger(), "Failed to initialize RKNN detector: %s", e.what());
    throw;
  }

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
      "QR D435i base ready. color=%s depth=%s camera_info=%s",
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
  log_throttle_ms_ = this->declare_parameter<int>("log_throttle_ms", 2000);
  sample_radius_px_ = this->declare_parameter<int>("sample_radius_px", 3);

  // 测试同学运行时可以用下面的 ROS 参数覆盖默认模型路径：
  // --ros-args -p model_path:=$HOME/qr_ws/src/qr_vision_cpp/qr_yolo11n_rk3588_i8.rknn
  model_path_ = this->declare_parameter<std::string>(
      "model_path",
      "/home/orangepi/models/qr_yolo11n_rk3588_i8.rknn");

  if (log_throttle_ms_ <= 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "log_throttle_ms must be greater than 0, reset to 2000");
    log_throttle_ms_ = 2000;
  }

  if (sample_radius_px_ < 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "sample_radius_px must be greater than or equal to 0, reset to 3");
    sample_radius_px_ = 3;
  }
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
  latest_camera_info_ = *camera_info_msg;
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

  std::vector<Detection> detections;

  try
  {
    detections = detector_->infer(color_bridge->image);
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "RKNN inference failed: %s",
        ex.what());
    return;
  }

  for (Detection &detection : detections)
  {
    DepthSampleResult depth_result = depth_processor_.sampleAt(
        depth_bridge->image,
        detection.center.x,
        detection.center.y,
        sample_radius_px_);

    if (depth_result.has_valid_depth)
    {
      detection.has_depth = true;
      detection.depth_m = depth_result.depth_m;

      if (has_camera_info_)
      {
        detection.point_3d = depth_processor_.projectTo3D(
            detection.center.x,
            detection.center.y,
            detection.depth_m,
            latest_camera_info_);
      }
    }
  }

  if (!detections.empty())
  {
    const Detection &best_detection = detections.front();

    if (best_detection.has_depth)
    {
      RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          log_throttle_ms_,
          "qr detection: class=%d score=%.2f center=(%d,%d) depth=%.3fm fps=%.1f",
          best_detection.class_id,
          best_detection.score,
          best_detection.center.x,
          best_detection.center.y,
          best_detection.depth_m,
          smoothed_fps_);
    }
    else
    {
      RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          log_throttle_ms_,
          "qr detection: class=%d score=%.2f center=(%d,%d) depth=invalid fps=%.1f",
          best_detection.class_id,
          best_detection.score,
          best_detection.center.x,
          best_detection.center.y,
          smoothed_fps_);
    }
  }
  else
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "no qr detection fps=%.1f",
        smoothed_fps_);
  }

  if (debug_view_)
  {
    displayDebugFrame(color_bridge->image, detections);
  }
}

void QrVisionNode::displayDebugFrame(
    const cv::Mat &color_image,
    const std::vector<Detection> &detections)
{
  cv::Mat display = color_image.clone();

  cv::Mat overlay = display.clone();
  cv::rectangle(overlay, cv::Point(12, 12), cv::Point(250, 76), cv::Scalar(30, 30, 30), cv::FILLED);
  cv::addWeighted(overlay, 0.45, display, 0.55, 0.0, display);

  for (const Detection &detection : detections)
  {
    const cv::Scalar color(0, 255, 0);
    cv::rectangle(display, detection.box, color, 2);
    cv::circle(display, detection.center, 4, cv::Scalar(0, 0, 255), cv::FILLED);

    std::string label;
    if (detection.has_depth)
    {
      label = cv::format(
          "cls:%d %.2f %.2fm",
          detection.class_id,
          detection.score,
          detection.depth_m);
    }
    else
    {
      label = cv::format(
          "cls:%d %.2f",
          detection.class_id,
          detection.score);
    }

    const int text_y = std::max(20, detection.box.y - 8);
    cv::putText(
        display,
        label,
        cv::Point(detection.box.x, text_y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        1);
  }

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

  cv::imshow(window_name_, display);
  cv::waitKey(1);
}
