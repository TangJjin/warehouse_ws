#pragma once

#include <dnn/hb_dnn.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct BpuYoloDetection
{
  float x_min_px{0.0F};
  float y_min_px{0.0F};
  float x_max_px{0.0F};
  float y_max_px{0.0F};
  float score{0.0F};
  int32_t class_id{0};
};

class BpuYoloDetector
{
public:
  explicit BpuYoloDetector(const std::string &model_path);
  ~BpuYoloDetector();

  BpuYoloDetector(const BpuYoloDetector &) = delete;
  BpuYoloDetector &operator=(const BpuYoloDetector &) = delete;

  void printModelInfo() const;
  std::vector<BpuYoloDetection> inferNv12(
      const uint8_t *nv12_data,
      std::size_t nv12_size);

private:
  void loadModel(const std::string &model_path);
  void releaseModel();
  void queryTensorProperties();
  void validateOutputTensors() const;
  void allocateTensors();
  void releaseTensors();

  static void checkRet(int ret, const std::string &step);
  static void printShape(const hbDNNTensorShape &shape);

  hbPackedDNNHandle_t _packed_dnn_handle{nullptr};
  hbDNNHandle_t _dnn_handle{nullptr};

  std::string _model_name;
  int32_t _input_count{0};
  int32_t _output_count{0};

  hbDNNTensorProperties _input_properties{};
  std::vector<hbDNNTensorProperties> _output_properties;

  hbDNNTensor _input_tensor{};
  std::vector<hbDNNTensor> _output_tensors;

  bool _input_tensor_allocated{false};
  std::vector<uint8_t> _output_tensor_allocated;
};
