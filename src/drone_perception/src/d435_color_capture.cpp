#include "drone_perception/d435_color_capture.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <utility>

namespace drone_perception
{

D435ColorCapture::D435ColorCapture(Config config)
    : config_(std::move(config))
{
}

D435ColorCapture::~D435ColorCapture()
{
  stop();
}

std::string D435ColorCapture::detectSingleDeviceSerial()
{
  try {
    rs2::context context;
    rs2::device_list devices = context.query_devices();

    if (devices.size() != 1U) {
      return {};
    }

    const char *serial =
        devices[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    return serial == nullptr ? std::string{} : std::string(serial);
  } catch (const rs2::error &) {
    return {};
  } catch (const std::exception &) {
    return {};
  }
}

bool D435ColorCapture::start()
{
  if (running_.exchange(true)) {
    return true;
  }

  setError({});

  if (!openPipeline()) {
    running_ = false;
    return false;
  }

  capture_thread_ = std::thread(&D435ColorCapture::captureLoop, this);
  return true;
}

void D435ColorCapture::stop()
{
  if (!running_.exchange(false)) {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    return;
  }

  frame_cv_.notify_all();

  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  closePipeline();
}

bool D435ColorCapture::running() const
{
  return running_.load();
}

std::string D435ColorCapture::lastError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

std::uint64_t D435ColorCapture::capturedCount() const
{
  return captured_count_.load();
}

std::uint64_t D435ColorCapture::droppedCount() const
{
  return dropped_count_.load();
}

std::uint64_t D435ColorCapture::timeoutCount() const
{
  return timeout_count_.load();
}

std::uint64_t D435ColorCapture::reconnectCount() const
{
  return reconnect_count_.load();
}

void D435ColorCapture::setError(const std::string &message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_error_ = message;
}

bool D435ColorCapture::openPipeline()
{
  try {
    // Fail fast on a missing device. pipeline::start() with enable_device()
    // blocks waiting for the requested device to appear, so verify presence
    // here to turn "no camera / wrong serial" into an immediate error instead
    // of an unbounded hang.
    {
      rs2::context context;
      const rs2::device_list devices = context.query_devices();

      if (config_.serial.empty()) {
        if (devices.size() == 0U) {
          setError("open pipeline: no RealSense device present");
          return false;
        }
      } else {
        bool found = false;

        for (std::size_t i = 0U; i < devices.size(); ++i) {
          const char *serial =
              devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

          if (serial != nullptr && config_.serial == serial) {
            found = true;
            break;
          }
        }

        if (!found) {
          setError(
              "open pipeline: device with serial " + config_.serial +
              " is not present");
          return false;
        }
      }
    }

    rs2::config config;
    config.disable_all_streams();

    if (!config_.serial.empty()) {
      config.enable_device(config_.serial);
    }

    config.enable_stream(
        RS2_STREAM_COLOR,
        config_.width,
        config_.height,
        RS2_FORMAT_YUYV,
        config_.fps);

    pipeline_ = std::make_unique<rs2::pipeline>();
    pipeline_->start(config);

    setError({});
    return true;
  } catch (const rs2::error &e) {
    setError(std::string("open pipeline: ") + e.what());
    pipeline_.reset();
    return false;
  } catch (const std::exception &e) {
    setError(std::string("open pipeline: ") + e.what());
    pipeline_.reset();
    return false;
  }
}

void D435ColorCapture::closePipeline()
{
  try {
    if (pipeline_) {
      pipeline_->stop();
    }
  } catch (const rs2::error &) {
    // Best-effort stop; the device may already be gone.
  }
  pipeline_.reset();
}

void D435ColorCapture::captureLoop()
{
  while (running_.load()) {
    if (!pipeline_) {
      ++reconnect_count_;

      std::this_thread::sleep_for(
          std::chrono::milliseconds(config_.reconnect_delay_ms));

      if (!running_.load()) {
        break;
      }

      if (!openPipeline()) {
        continue;
      }
    }

    rs2::frameset frames;

    try {
      frames = pipeline_->wait_for_frames(
          static_cast<unsigned int>(config_.wait_timeout_ms));
    } catch (const rs2::error &e) {
      const std::string message(e.what());
      const bool is_timeout =
          message.find("timeout") != std::string::npos ||
          message.find("Timeout") != std::string::npos;

      if (is_timeout) {
        ++timeout_count_;
        continue;
      }

      setError(std::string("wait_for_frames: ") + message);
      closePipeline();
      continue;
    } catch (const std::exception &e) {
      setError(std::string("wait_for_frames: ") + e.what());
      closePipeline();
      continue;
    }

    if (!frames) {
      continue;
    }

    rs2::video_frame color = frames.first_or_default(RS2_STREAM_COLOR);

    if (!color) {
      continue;
    }

    const int width = color.get_width();
    const int height = color.get_height();
    const int stride = color.get_stride_in_bytes();
    const int row_bytes = std::min<int>(stride, width * 2);

    if (row_bytes <= 0 || height <= 0) {
      continue;
    }

    const std::uint8_t *data =
        static_cast<const std::uint8_t *>(color.get_data());

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (frame_pending_) {
        ++dropped_count_;
      }

      D435ColorFrame frame;
      frame.sequence = next_sequence_++;
      frame.device_timestamp_ms = color.get_timestamp();
      frame.received_at = std::chrono::steady_clock::now();
      frame.width = width;
      frame.height = height;
      frame.stride_bytes = row_bytes;
      frame.yuyv.resize(
          static_cast<std::size_t>(row_bytes) * static_cast<std::size_t>(height));

      for (int row = 0; row < height; ++row) {
        std::memcpy(
            frame.yuyv.data() + static_cast<std::size_t>(row) * static_cast<std::size_t>(row_bytes),
            data + static_cast<std::size_t>(row) * static_cast<std::size_t>(stride),
            static_cast<std::size_t>(row_bytes));
      }

      latest_frame_ = std::move(frame);
      frame_pending_ = true;
      ++captured_count_;
    }

    frame_cv_.notify_all();
  }

  closePipeline();
}

bool D435ColorCapture::waitForLatest(
    D435ColorFrame &frame,
    std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const bool signaled = frame_cv_.wait_for(lock, timeout, [this]() {
    return frame_pending_ || !running_.load();
  });

  if (!signaled || !frame_pending_) {
    return false;
  }

  frame = latest_frame_;
  frame_pending_ = false;
  return true;
}

}  // namespace drone_perception
