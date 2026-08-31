#include "color_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

ColorDetector::ColorDetector(VisionConfig config) : config_(std::move(config)) {}

TargetObservation ColorDetector::detect(const ImageFrame& frame) const {
  TargetObservation observation;
  observation.sequence = frame.sequence;
  observation.timestamp_us = frame.timestamp_us;
  if (frame.image.empty()) {
    return observation;
  }

  cv::Mat hsv;
  cv::Mat mask;
  cv::cvtColor(frame.image, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(
      hsv,
      cv::Scalar(config_.h_min, config_.s_min, config_.v_min),
      cv::Scalar(config_.h_max, config_.s_max, config_.v_max),
      mask);

  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double best_score = -std::numeric_limits<double>::infinity();
  double best_area = 0.0;
  cv::Point2f best_center;
  float best_radius = 0.0F;
  for (const auto& contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < config_.min_area || area > config_.max_area) {
      continue;
    }

    const auto moments = cv::moments(contour);
    if (std::abs(moments.m00) <= std::numeric_limits<double>::epsilon()) {
      continue;
    }
    const cv::Point2f centroid(
        static_cast<float>(moments.m10 / moments.m00),
        static_cast<float>(moments.m01 / moments.m00));
    cv::Point2f circle_center;
    float radius = 0.0F;
    cv::minEnclosingCircle(contour, circle_center, radius);

    const double perimeter = cv::arcLength(contour, true);
    const double circularity = perimeter > 0.0
        ? std::clamp(4.0 * CV_PI * area / (perimeter * perimeter), 0.0, 1.0)
        : 0.0;
    const double score = area * (0.5 + 0.5 * circularity);
    if (score > best_score ||
        (score == best_score && area > best_area)) {
      best_score = score;
      best_area = area;
      best_center = centroid;
      best_radius = radius;
      observation.confidence = static_cast<float>(circularity);
    }
  }

  if (best_score == -std::numeric_limits<double>::infinity()) {
    return observation;
  }
  observation.detected = true;
  observation.center = best_center;
  observation.radius = best_radius;
  return observation;
}
