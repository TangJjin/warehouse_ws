#pragma once

#include <dnn/hb_dnn.h>

#include <cstdint>
#include <string>

class BpuYoloDetector
{
public:
  explicit BpuYoloDetector(const std::string &model_path);
  ~BpuYoloDetector();

  BpuYoloDetector(const BpuYoloDetector &) = delete;
  BpuYoloDetector &operator=(const BpuYoloDetector &) = delete;

  void printModelInfo() const;

private:
  void loadModel(const std::string &model_path);
  void releaseModel();

  static void checkRet(int ret, const std::string &step);
  static void printShape(const hbDNNTensorShape &shape);

  hbPackedDNNHandle_t _packed_dnn_handle{nullptr};
  hbDNNHandle_t _dnn_handle{nullptr};

  std::string _model_name;
  int32_t _input_count{0};
  int32_t _output_count{0};
};
