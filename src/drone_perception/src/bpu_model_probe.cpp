#include <dnn/hb_dnn.h>

#include <cstdint>
#include <iostream>
#include <string>

namespace
{

bool checkRet(int ret, const std::string &step)
{
  if (ret != 0) {
    std::cerr << step << " failed, ret=" << ret << std::endl;
    return false;
  }

  std::cout << step << " ok" << std::endl;
  return true;
}

void printShape(const hbDNNTensorShape &shape)
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

void printTensorProperties(
  const std::string &name,
  int32_t index,
  const hbDNNTensorProperties &properties)
{
  std::cout << name << "[" << index << "]"
            << " valid_shape=";
  printShape(properties.validShape);

  std::cout << " aligned_shape=";
  printShape(properties.alignedShape);
  std::cout << std::endl;
}

}  // namespace

int main()
{
  const char *model_path =
    "/home/gjl/drone_ws/src/rdk_best_bayese_640x640_nv12.bin";

  hbPackedDNNHandle_t packed_dnn_handle = nullptr;
  const char *model_files[] = {model_path};

  int ret = hbDNNInitializeFromFiles(&packed_dnn_handle, model_files, 1);

  if (!checkRet(ret, "hbDNNInitializeFromFiles")) {
    return 1;
  }

  const char **model_name_list = nullptr;
  int32_t model_name_count = 0;

  ret = hbDNNGetModelNameList(
    &model_name_list,
    &model_name_count,
    packed_dnn_handle);

  if (!checkRet(ret, "hbDNNGetModelNameList")) {
    hbDNNRelease(packed_dnn_handle);
    return 1;
  }

  std::cout << "model count: " << model_name_count << std::endl;

  if (model_name_count <= 0) {
    std::cerr << "no model found in bin file" << std::endl;
    hbDNNRelease(packed_dnn_handle);
    return 1;
  }

  const char *model_name = model_name_list[0];
  std::cout << "model name: " << model_name << std::endl;

  hbDNNHandle_t dnn_handle = nullptr;

  ret = hbDNNGetModelHandle(
    &dnn_handle,
    packed_dnn_handle,
    model_name);

  if (!checkRet(ret, "hbDNNGetModelHandle")) {
    hbDNNRelease(packed_dnn_handle);
    return 1;
  }

  int32_t input_count = 0;
  int32_t output_count = 0;

  ret = hbDNNGetInputCount(&input_count, dnn_handle);

  if (!checkRet(ret, "hbDNNGetInputCount")) {
    hbDNNRelease(packed_dnn_handle);
    return 1;
  }

  ret = hbDNNGetOutputCount(&output_count, dnn_handle);

  if (!checkRet(ret, "hbDNNGetOutputCount")) {
    hbDNNRelease(packed_dnn_handle);
    return 1;
  }

  std::cout << "input count: " << input_count << std::endl;
  std::cout << "output count: " << output_count << std::endl;

  for (int32_t i = 0; i < input_count; ++i) {
    hbDNNTensorProperties properties;
    ret = hbDNNGetInputTensorProperties(&properties, dnn_handle, i);

    if (!checkRet(ret, "hbDNNGetInputTensorProperties")) {
      hbDNNRelease(packed_dnn_handle);
      return 1;
    }

    printTensorProperties("input", i, properties);
  }

  for (int32_t i = 0; i < output_count; ++i) {
    hbDNNTensorProperties properties;
    ret = hbDNNGetOutputTensorProperties(&properties, dnn_handle, i);

    if (!checkRet(ret, "hbDNNGetOutputTensorProperties")) {
      hbDNNRelease(packed_dnn_handle);
      return 1;
    }

    printTensorProperties("output", i, properties);
  }

  hbDNNRelease(packed_dnn_handle);
  return 0;
}
