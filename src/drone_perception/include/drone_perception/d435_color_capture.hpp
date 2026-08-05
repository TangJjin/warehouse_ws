#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <librealsense2/rs.hpp>

namespace drone_perception
{

// Latest captured D435i color frame (YUYV). The SDK frame is copied into this
// self-contained buffer on the capture thread so no rs2::video_frame is ever
// held across threads.
struct D435ColorFrame
{
  std::uint64_t sequence{0};
  double device_timestamp_ms{0.0};
  std::chrono::steady_clock::time_point received_at{};
  int width{0};
  int height{0};
  int stride_bytes{0};
  std::vector<std::uint8_t> yuyv;
};

// Direct color capture from the D435i via librealsense, bypassing the ROS
// realsense2_camera_node. Runs its own capture thread and keeps a single
// latest-frame slot: a new frame overwrites any unread frame, so consumers
// always see the newest frame and never accumulate backlog.
class D435ColorCapture
{
public:
  struct Config
  {
    std::string serial;
    int width{640};
    int height{480};
    int fps{30};
    int wait_timeout_ms{2000};
    int reconnect_delay_ms{2000};
  };

  explicit D435ColorCapture(Config config);
  ~D435ColorCapture();

  D435ColorCapture(const D435ColorCapture &) = delete;
  D435ColorCapture &operator=(const D435ColorCapture &) = delete;

  // Resolves the D435i serial when Config::serial is empty. Picks the single
  // device only when exactly one RealSense device is present; otherwise returns
  // an empty string so the caller can refuse to auto-select.
  static std::string detectSingleDeviceSerial();

  bool start();
  void stop();
  bool waitForLatest(D435ColorFrame &frame, std::chrono::milliseconds timeout);
  bool running() const;
  std::string lastError() const;

  std::uint64_t capturedCount() const;
  std::uint64_t droppedCount() const;
  std::uint64_t timeoutCount() const;
  std::uint64_t reconnectCount() const;

private:
  void captureLoop();
  bool openPipeline();
  void closePipeline();
  void setError(const std::string &message);

  Config config_;

  std::atomic_bool running_{false};
  std::thread capture_thread_;

  mutable std::mutex mutex_;
  std::condition_variable frame_cv_;
  D435ColorFrame latest_frame_;
  bool frame_pending_{false};
  std::string last_error_;

  std::atomic<std::uint64_t> captured_count_{0};
  std::atomic<std::uint64_t> dropped_count_{0};
  std::atomic<std::uint64_t> timeout_count_{0};
  std::atomic<std::uint64_t> reconnect_count_{0};
  std::uint64_t next_sequence_{1};

  std::unique_ptr<rs2::pipeline> pipeline_;
};

}  // namespace drone_perception
