#pragma once

#include <opencv2/core/types.hpp>

#include <vector>
#include "drone_perception/detection.hpp"

class YoloPostprocessor
{
public:
    YoloPostprocessor(
        int class_count,
        float confidence_threshold,
        float nms_threshold);

    std::vector<Detection> parseOutput(
        const float *output_data,
        int candidate_count,
        const cv::Size &original_size,
        float scale,
        int pad_x,
        int pad_y
    ) const;

    std::vector<Detection> parseSplitOutput(
        const float *bbox_data,
        const float *class_data,
        int candidate_count,
        const cv::Size &original_size,
        float scale,
        int pad_x,
        int pad_y
    ) const;

private:
    std::vector<Detection> parseChannelData(
        const float *bbox_data,
        const float *class_data,
        int candidate_count,
        const cv::Size &original_size,
        float scale,
        int pad_x,
        int pad_y
    ) const;

    int class_count_ = 2;
    float confidence_threshold_ = 0.60F;
    float nms_threshold_ = 0.35F;

};
