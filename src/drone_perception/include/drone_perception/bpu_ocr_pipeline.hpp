#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include <dnn/hb_dnn.h>

struct OcrTextRegion
{
  std::array<cv::Point2f, 4> box{};
  std::string text;
  float score{0.0F};
};

struct BpuOcrConfig
{
  std::string det_model_path;
  std::string rec_model_path;
  float det_threshold{0.5F};
  int det_min_area_px{100};
  int rec_input_height_px{48};
  int rec_input_width_px{320};
};

class BpuOcrPipeline
{
public:
  explicit BpuOcrPipeline(const BpuOcrConfig &config);
  ~BpuOcrPipeline();

  BpuOcrPipeline(const BpuOcrPipeline &) = delete;
  BpuOcrPipeline &operator=(const BpuOcrPipeline &) = delete;

  std::vector<OcrTextRegion> infer(const cv::Mat &bgr_image);
  void printModelInfo() const;

private:
  class BpuDnnModel;

  struct DetectionBox
  {
    std::array<cv::Point2f, 4> points{};
  };

  struct RecognitionResult
  {
    std::string text;
    float score{0.0F};
  };

  std::vector<uint8_t> prepareDetectionInput(const cv::Mat &bgr_image) const;
  std::vector<DetectionBox> detectTextBoxes(
      const std::vector<std::vector<float>> &outputs,
      const cv::Size &image_size) const;
  RecognitionResult recognizeText(const cv::Mat &crop_bgr) const;
  cv::Mat cropAndRectify(const cv::Mat &image, const DetectionBox &box) const;
  static std::string decodeCtcGreedy(
      const float *logits,
      int time_steps,
      int class_count,
      float *mean_score);
  static std::string normalizeShelfCode(const std::string &text);
  static bool isValidShelfCode(const std::string &text);
  static std::string defaultDetectionModelPath();
  static std::string defaultRecognitionModelPath();

  BpuOcrConfig _config;
  std::unique_ptr<BpuDnnModel> _det_model;
  std::unique_ptr<BpuDnnModel> _rec_model;
};
