#pragma once

#include <dnn/hb_dnn.h>

#include <cstdint>
#include <string>
#include <vector>

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
  void queryTensorProperties();
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
