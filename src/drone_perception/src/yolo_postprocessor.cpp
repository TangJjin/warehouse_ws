#include "drone_perception/yolo_postprocessor.hpp"

#include <opencv2/dnn.hpp>

YoloPostprocessor::YoloPostprocessor(
    int class_count,
    float confidence_threshold,
    float nms_threshold)
    : class_count_(class_count),
      confidence_threshold_(confidence_threshold),
      nms_threshold_(nms_threshold)
{
}

std::vector<Detection> YoloPostprocessor::parseOutput(
    const float *output_data,
    int candidate_count,
    const cv::Size &original_size,
    float scale,
    int pad_x,
    int pad_y) const
{
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    if (output_data == nullptr || candidate_count <= 0 || scale <= 0.0F ||
        original_size.width <= 0 || original_size.height <= 0)
    {
        return {};
    }

    for (int i = 0; i < candidate_count; ++i)
    {
        const float x_center = output_data[0 * candidate_count + i];
        const float y_center = output_data[1 * candidate_count + i];
        const float width = output_data[2 * candidate_count + i];
        const float height = output_data[3 * candidate_count + i];

        float best_score = 0.0F;
        int best_class = -1;

        for (int class_id = 0; class_id < class_count_; ++class_id)
        {
            const float score = output_data[(4 + class_id) * candidate_count + i];
            if (score > best_score)
            {
                best_score = score;
                best_class = class_id;
            }
        }

        if (best_score < confidence_threshold_ || best_class < 0)
        {
            continue;
        }

        const float left_640 = x_center - width * 0.5F;
        const float top_640 = y_center - height * 0.5F;

        const int left = static_cast<int>((left_640 - static_cast<float>(pad_x)) / scale);
        const int top = static_cast<int>((top_640 - static_cast<float>(pad_y)) / scale);
        const int box_width = static_cast<int>(width / scale);
        const int box_height = static_cast<int>(height / scale);

        cv::Rect box(left, top, box_width, box_height);
        box &= cv::Rect(0, 0, original_size.width, original_size.height);

        if (box.empty())
        {
            continue;
        }

        boxes.push_back(box);
        scores.push_back(best_score);
        class_ids.push_back(best_class);
    }

    std::vector<int> keep_indices;

    cv::dnn::NMSBoxes(
        boxes,
        scores,
        confidence_threshold_,
        nms_threshold_,
        keep_indices);

    std::vector<Detection> detections;
    detections.reserve(keep_indices.size());

    for (const int index : keep_indices)
    {
        Detection detection;
        detection.class_id = class_ids[index];
        detection.box = boxes[index];
        detection.center = cv::Point(detection.box.x + detection.box.width / 2,
                                     detection.box.y + detection.box.height / 2);
        detection.score = scores[index];

        detections.push_back(detection);
    }
    return detections;
}
