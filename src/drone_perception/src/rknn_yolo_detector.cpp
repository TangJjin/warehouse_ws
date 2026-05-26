#include "drone_perception/rknn_yolo_detector.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/imgproc.hpp>

namespace
{
void printTensorAttr(const char *prefix, const rknn_tensor_attr &attr)
{
    std::cout << "[RKNN] " << prefix << "[" << attr.index << "]"
              << " name=" << attr.name
              << " n_dims=" << attr.n_dims
              << " dims=(";

    for (uint32_t i = 0; i < attr.n_dims; ++i)
    {
        std::cout << attr.dims[i];
        if (i + 1 < attr.n_dims)
        {
            std::cout << ",";
        }
    }

    std::cout << ")"
              << " type=" << attr.type
              << " fmt=" << attr.fmt
              << " qnt_type=" << attr.qnt_type
              << " zp=" << attr.zp
              << " scale=" << attr.scale
              << std::endl;
}
struct RknnOutputGuard
{
    rknn_context context;
    rknn_output *output;
    bool active = true;

    ~RknnOutputGuard()
    {
        if(active && output != nullptr)
        {
            rknn_outputs_release(context, 1, output);
        }
    }
    RknnOutputGuard(const RknnOutputGuard &) = delete;
    RknnOutputGuard &operator=(const RknnOutputGuard&) = delete;

};


}  // namespace

RknnYoloDetector::RknnYoloDetector(const std::string &model_path)
{
    loadModel(model_path);
}

RknnYoloDetector::~RknnYoloDetector()
{
    if (context_ != 0)
    {
        rknn_destroy(context_);
        context_ = 0;
    }
}

std::vector<unsigned char> RknnYoloDetector::readFile(const std::string &path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);

    if (!file)
    {
        throw std::runtime_error("failed to open model file: " + path);
    }

    const std::streamsize file_size = file.tellg();

    if (file_size <= 0)
    {
        throw std::runtime_error("model file is empty: " + path);
    }

    std::vector<unsigned char> data(static_cast<std::size_t>(file_size));
    file.seekg(0, std::ios::beg);

    if (!file.read(reinterpret_cast<char *>(data.data()), file_size))
    {
        throw std::runtime_error("failed to read model file: " + path);
    }

    return data;
}

RknnYoloDetector::LetterboxResult RknnYoloDetector::makeLetterbox(
    const cv::Mat &bgr_image) const
{
    if (bgr_image.empty())
    {
        throw std::runtime_error("input image is empty");
    }

    // 保持原图比例并补边到 640x640，避免二维码/条码被拉伸变形。
    const float scale = std::min(
        static_cast<float>(input_width_) / static_cast<float>(bgr_image.cols),
        static_cast<float>(input_height_) / static_cast<float>(bgr_image.rows));

    const int resized_width = static_cast<int>(static_cast<float>(bgr_image.cols) * scale);
    const int resized_height = static_cast<int>(static_cast<float>(bgr_image.rows) * scale);

    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(resized_width, resized_height));

    const int pad_x = (input_width_ - resized_width) / 2;
    const int pad_y = (input_height_ - resized_height) / 2;

    cv::Mat letterbox(
        input_height_,
        input_width_,
        bgr_image.type(),
        cv::Scalar(114, 114, 114));

    resized.copyTo(letterbox(cv::Rect(pad_x, pad_y, resized_width, resized_height)));

    LetterboxResult result;
    result.image = letterbox;
    result.scale = scale;
    result.pad_x = pad_x;
    result.pad_y = pad_y;

    return result;
}

void RknnYoloDetector::loadModel(const std::string &model_path)
{
    model_data_ = readFile(model_path);

    // rknn_init 会把 .rknn 模型加载进 RKNN Runtime，必须在 RK3588/OrangePi 上运行。
    const int ret = rknn_init(
        &context_,
        model_data_.data(),
        static_cast<unsigned int>(model_data_.size()),
        0,
        nullptr);

    if (ret != RKNN_SUCC)
    {
        throw std::runtime_error("rknn_init failed, ret=" + std::to_string(ret));
    }

    rknn_input_output_num io_num;
    std::memset(&io_num, 0, sizeof(io_num));

    int query_ret = rknn_query(
        context_,
        RKNN_QUERY_IN_OUT_NUM,
        &io_num,
        sizeof(io_num));

    if (query_ret != RKNN_SUCC)
    {
        throw std::runtime_error("RKNN_QUERY_IN_OUT_NUM failed, ret=" + std::to_string(query_ret));
    }

    std::cout << "[RKNN] input num: " << io_num.n_input
              << ", output num: " << io_num.n_output << std::endl;

    for (uint32_t i = 0; i < io_num.n_input; ++i)
    {
        rknn_tensor_attr attr;
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;

        query_ret = rknn_query(
            context_,
            RKNN_QUERY_INPUT_ATTR,
            &attr,
            sizeof(attr));

        if (query_ret != RKNN_SUCC)
        {
            throw std::runtime_error("RKNN_QUERY_INPUT_ATTR failed, ret=" + std::to_string(query_ret));
        }

        printTensorAttr("input", attr);
    }

    for (uint32_t i = 0; i < io_num.n_output; ++i)
    {
        rknn_tensor_attr attr;
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;

        query_ret = rknn_query(
            context_,
            RKNN_QUERY_OUTPUT_ATTR,
            &attr,
            sizeof(attr));

        if (query_ret != RKNN_SUCC)
        {
            throw std::runtime_error("RKNN_QUERY_OUTPUT_ATTR failed, ret=" + std::to_string(query_ret));
        }

        printTensorAttr("output", attr);
    }
}

std::vector<Detection> RknnYoloDetector::infer(const cv::Mat &bgr_image)
{
    const LetterboxResult letterbox = makeLetterbox(bgr_image);

    cv::Mat rgb_image;
    cv::cvtColor(letterbox.image, rgb_image, cv::COLOR_BGR2RGB);

    rknn_input input;
    std::memset(&input, 0, sizeof(input));

    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.size = static_cast<unsigned int>(input_width_ * input_height_ * 3);
    input.fmt = RKNN_TENSOR_NHWC;
    input.buf = rgb_image.data;

    int ret = rknn_inputs_set(context_, 1, &input);

    if (ret != RKNN_SUCC)
    {
        throw std::runtime_error("rknn_inputs_set failed, ret=" + std::to_string(ret));
    }

    ret = rknn_run(context_, nullptr);

    if (ret != RKNN_SUCC)
    {
        throw std::runtime_error("rknn_run failed, ret=" + std::to_string(ret));
    }

    rknn_output output;
    std::memset(&output, 0, sizeof(output));

    output.index = 0;
    // 第一版先让 Runtime 把输出反量化成 float，方便复用 YOLO 后处理。
    output.want_float = 1;

    ret = rknn_outputs_get(context_, 1, &output, nullptr);

    if (ret != RKNN_SUCC)
    {
        throw std::runtime_error("rknn_outputs_get failed, ret=" + std::to_string(ret));
    }

    RknnOutputGuard output_guard{context_, &output};

    const float *output_data = static_cast<const float *>(output.buf);

    std::vector<Detection> detections = postprocessor_.parseOutput(
        output_data,
        candidate_count_,
        bgr_image.size(),
        letterbox.scale,
        letterbox.pad_x,
        letterbox.pad_y);

    
    return detections;
}
