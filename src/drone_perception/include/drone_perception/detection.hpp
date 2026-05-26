#pragma once

#include <opencv2/core/types.hpp>


struct Detection
{
    int class_id = -1;
    float score = 0.0F;
    cv::Rect box;
    cv::Point center;
    bool has_depth = false;
    float depth_m = 0.0F;
    cv::Point3d point_3d;
};
