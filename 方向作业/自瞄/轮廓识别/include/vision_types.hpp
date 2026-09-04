#pragma once

#include <opencv2/core.hpp>

struct RedComponent {
  int label = 0;
  int area = 0;
  cv::Rect box;
  cv::Point2f center;
};
