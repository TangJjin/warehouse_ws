#include "drone_perception/bpu_yolo_detector.hpp"

#include <dnn/hb_sys.h>

#include <cstddef>
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
  releaseTensors();

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
  _input_properties = {};
  _output_properties.clear();
}

void BpuYoloDetector::releaseTensors()
{
  if (_input_tensor_allocated) {
    const int ret = hbSysFreeMem(&_input_tensor.sysMem[0]);

    if (ret != 0) {
      std::cerr << "hbSysFreeMem input failed, ret=" << ret << std::endl;
    }

    _input_tensor = {};
    _input_tensor_allocated = false;
  }

  for (std::size_t i = 0; i < _output_tensors.size(); ++i) {
    if (i >= _output_tensor_allocated.size() || _output_tensor_allocated[i] == 0U) {
      continue;
    }

    const int ret = hbSysFreeMem(&_output_tensors[i].sysMem[0]);

    if (ret != 0) {
      std::cerr << "hbSysFreeMem output[" << i << "] failed, ret=" << ret << std::endl;
    }
  }

  _output_tensors.clear();
  _output_tensor_allocated.clear();
}

void BpuYoloDetector::allocateTensors()
{
  releaseTensors();

  try {
    _input_tensor.properties = _input_properties;

    checkRet(
        hbSysAllocCachedMem(&_input_tensor.sysMem[0], _input_properties.alignedByteSize),
        "hbSysAllocCachedMem input");

    _input_tensor_allocated = true;

    _output_tensors.clear();
    _output_tensor_allocated.clear();

    _output_tensors.resize(static_cast<std::size_t>(_output_count));
    _output_tensor_allocated.resize(static_cast<std::size_t>(_output_count), 0U);

    for (int32_t i = 0; i < _output_count; ++i) {
      const std::size_t index = static_cast<std::size_t>(i);
      hbDNNTensor &tensor = _output_tensors[index];
      const hbDNNTensorProperties &properties = _output_properties[index];

      tensor.properties = properties;

      checkRet(
          hbSysAllocCachedMem(&tensor.sysMem[0], properties.alignedByteSize),
          "hbSysAllocCachedMem output[" + std::to_string(i) + "]");

      _output_tensor_allocated[index] = 1U;
    }
  } catch (...) {
    releaseTensors();
    throw;
  }
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

    queryTensorProperties();
    allocateTensors();
  } catch (...) {
    releaseModel();
    throw;
  }
}

void BpuYoloDetector::queryTensorProperties()
{
  if (_dnn_handle == nullptr) {
    throw std::runtime_error("BPU model is not loaded");
  }

  if (_input_count != 1) {
    throw std::runtime_error("unsupported input count: " + std::to_string(_input_count));
  }

  checkRet(
      hbDNNGetInputTensorProperties(&_input_properties, _dnn_handle, 0),
      "hbDNNGetInputTensorProperties");

  _output_properties.clear();
  _output_properties.resize(static_cast<std::size_t>(_output_count));

  for (int32_t i = 0; i < _output_count; ++i) {
    hbDNNTensorProperties &properties = _output_properties[static_cast<std::size_t>(i)];

    checkRet(
        hbDNNGetOutputTensorProperties(&properties, _dnn_handle, i),
        "hbDNNGetOutputTensorProperties");
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

  std::cout << "input[0] valid_shape=";
  printShape(_input_properties.validShape);
  std::cout << " aligned_shape=";
  printShape(_input_properties.alignedShape);
  std::cout << std::endl;

  for (int32_t i = 0; i < _output_count; ++i) {
    const hbDNNTensorProperties &properties =
        _output_properties[static_cast<std::size_t>(i)];

    std::cout << "output[" << i << "] valid_shape=";
    printShape(properties.validShape);
    std::cout << " aligned_shape=";
    printShape(properties.alignedShape);
    std::cout << std::endl;
  }
}
