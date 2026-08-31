#pragma once

#include <cstddef>
#include <string>

struct VisionConfig {
  std::string input_video;
  std::string output_video;
  std::string output_csv;
  std::size_t queue_capacity = 8;
  int h_min = 90;
  int h_max = 140;
  int s_min = 80;
  int s_max = 255;
  int v_min = 50;
  int v_max = 255;
  double min_area = 100.0;
  double max_area = 10000.0;
  int max_missed_frames = 5;
  double max_match_distance = 80.0;
  double alpha = 0.75;
  double beta = 0.20;
};

VisionConfig load_vision_config(const std::string& path);
