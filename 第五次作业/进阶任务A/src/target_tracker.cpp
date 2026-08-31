#include "target_tracker.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace {

float nan_value() {
  return std::numeric_limits<float>::quiet_NaN();
}

cv::Point2f nan_point() {
  return {nan_value(), nan_value()};
}

double distance_between(const cv::Point2f& left, const cv::Point2f& right) {
  return std::hypot(
      static_cast<double>(left.x - right.x),
      static_cast<double>(left.y - right.y));
}

}  // namespace

TargetTracker::TargetTracker(VisionConfig config) : config_(std::move(config)) {}

TrackingResult TargetTracker::update(const TargetObservation& observation) {
  TrackingResult result;
  result.sequence = observation.sequence;
  result.timestamp_us = observation.timestamp_us;
  result.measured_position = observation.detected ? observation.center : nan_point();
  result.filtered_position = nan_point();
  result.velocity = nan_point();

  if (!initialized_) {
    if (observation.detected) {
      initialized_ = true;
      missed_frames_ = 0;
      last_timestamp_us_ = observation.timestamp_us;
      position_ = observation.center;
      velocity_ = {0.0F, 0.0F};
      result.valid = true;
      result.filtered_position = position_;
      result.velocity = velocity_;
    }
    return result;
  }

  const std::int64_t elapsed_us = observation.timestamp_us - last_timestamp_us_;
  const double dt = elapsed_us > 0
      ? static_cast<double>(elapsed_us) / 1'000'000.0
      : 1e-6;
  const cv::Point2f predicted = position_ + velocity_ * static_cast<float>(dt);
  const bool accepted = observation.detected &&
      distance_between(observation.center, predicted) <= config_.max_match_distance;

  if (accepted) {
    const cv::Point2f residual = observation.center - predicted;
    position_ = predicted + residual * static_cast<float>(config_.alpha);
    velocity_ += residual * static_cast<float>(config_.beta / dt);
    missed_frames_ = 0;
    result.valid = true;
    result.predicted = false;
  } else {
    position_ = predicted;
    ++missed_frames_;
    result.valid = missed_frames_ <= config_.max_missed_frames;
    result.predicted = result.valid;
    if (!result.valid) {
      initialized_ = false;
    }
  }

  last_timestamp_us_ = observation.timestamp_us;
  if (result.valid) {
    result.filtered_position = position_;
    result.velocity = velocity_;
  }
  return result;
}
