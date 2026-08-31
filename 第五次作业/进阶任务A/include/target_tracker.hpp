#pragma once

#include "vision_config.hpp"
#include "vision_types.hpp"

class TargetTracker {
 public:
  explicit TargetTracker(VisionConfig config);
  TrackingResult update(const TargetObservation& observation);

 private:
  VisionConfig config_;
  bool initialized_ = false;
  int missed_frames_ = 0;
  std::int64_t last_timestamp_us_ = 0;
  cv::Point2f position_{0.0F, 0.0F};
  cv::Point2f velocity_{0.0F, 0.0F};
};
