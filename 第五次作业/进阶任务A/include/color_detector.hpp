#pragma once

#include "vision_config.hpp"
#include "vision_types.hpp"

class ColorDetector {
 public:
  explicit ColorDetector(VisionConfig config);
  TargetObservation detect(const ImageFrame& frame) const;

 private:
  VisionConfig config_;
};
