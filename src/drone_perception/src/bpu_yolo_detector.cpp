#include "drone_perception/bpu_yolo_detector.hpp"

#include <iostream>
#include <stdexcept>

namespace
{

static constexpr int32_t kSingleModelFileCount = 1;
static constexpr int32_t kMinModelCount = 1;

}  // namespace

BpuYoloDetector::BpuYoloDetector(const std::string &model_path)
{
  loadModel(model_path);
}

BpuYoloDetector::~BpuYoloDetector()
{
  releaseModel();
}

void BpuYoloDetector::releaseModel()
{
  if (_packed_dnn_handle == nullptr) {
    return;
  }

  const int ret = hbDNNRelease(_packed_dnn_handle);

  if (ret != 0) {
    std::cerr << "hbDNNRelease failed, ret=" << ret << std::endl;
  }

  _packed_dnn_handle = nullptr;
  _dnn_handle = nullptr;
  _input_count = 0;
  _output_count = 0;
  _model_name.clear();
}

void BpuYoloDetector::checkRet(int ret, const std::string &step)
{
  if (ret == 0) {
    return;
  }

  throw std::runtime_error(step + " failed, ret=" + std::to_string(ret));
}

void BpuYoloDetector::printShape(const hbDNNTensorShape &shape)
{
  std::cout << "[";

  for (int32_t i = 0; i < shape.numDimensions; ++i) {
    std::cout << shape.dimensionSize[i];

    if (i + 1 < shape.numDimensions) {
      std::cout << ", ";
    }
  }

  std::cout << "]";
}

void BpuYoloDetector::loadModel(const std::string &model_path)
{
  releaseModel();

  try {
    const char *model_files[] = {model_path.c_str()};

    checkRet(
        hbDNNInitializeFromFiles(&_packed_dnn_handle, model_files, kSingleModelFileCount),
        "hbDNNInitializeFromFiles");

    const char **model_name_list = nullptr;
    int32_t model_name_count = 0;

    checkRet(
        hbDNNGetModelNameList(&model_name_list, &model_name_count, _packed_dnn_handle),
        "hbDNNGetModelNameList");

    if (model_name_count < kMinModelCount) {
      throw std::runtime_error("no model found in bin file");
    }

    _model_name = model_name_list[0];

    checkRet(
        hbDNNGetModelHandle(&_dnn_handle, _packed_dnn_handle, _model_name.c_str()),
        "hbDNNGetModelHandle");

    checkRet(
        hbDNNGetInputCount(&_input_count, _dnn_handle),
        "hbDNNGetInputCount");

    checkRet(
        hbDNNGetOutputCount(&_output_count, _dnn_handle),
        "hbDNNGetOutputCount");
  } catch (...) {
    releaseModel();
    throw;
  }
}

void BpuYoloDetector::printModelInfo() const
{
  if (_dnn_handle == nullptr) {
    throw std::runtime_error("BPU model is not loaded");
  }

  std::cout << "model name: " << _model_name << std::endl;
  std::cout << "input count: " << _input_count << std::endl;
  std::cout << "output count: " << _output_count << std::endl;

  for (int32_t i = 0; i < _input_count; ++i) {
    hbDNNTensorProperties properties{};

    checkRet(
        hbDNNGetInputTensorProperties(&properties, _dnn_handle, i),
        "hbDNNGetInputTensorProperties");

    std::cout << "input[" << i << "] valid_shape=";
    printShape(properties.validShape);
    std::cout << " aligned_shape=";
    printShape(properties.alignedShape);
    std::cout << std::endl;
  }

  for (int32_t i = 0; i < _output_count; ++i) {
    hbDNNTensorProperties properties{};

    checkRet(
        hbDNNGetOutputTensorProperties(&properties, _dnn_handle, i),
        "hbDNNGetOutputTensorProperties");

    std::cout << "output[" << i << "] valid_shape=";
    printShape(properties.validShape);
    std::cout << " aligned_shape=";
    printShape(properties.alignedShape);
    std::cout << std::endl;
  }
}
