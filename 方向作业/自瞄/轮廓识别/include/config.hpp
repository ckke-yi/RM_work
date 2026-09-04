#pragma once

#include <string>

struct EnergyConfig {
  std::string input_video;
  std::string output_video;
  std::string output_csv;

  int red_h_low = 12;
  int red_h_high = 170;
  int red_s_min = 80;
  int red_v_min = 60;
  int morphology_kernel = 3;

  int component_min_area = 100;
  int marker_min_area = 500;
  int marker_max_area = 6000;
  int marker_min_width = 25;
  int marker_max_width = 100;
  int marker_min_height = 25;
  int marker_max_height = 100;
  double marker_min_aspect = 0.45;
  double marker_max_aspect = 2.2;
  int marker_expected_area = 1900;

  int active_arm_min_area = 8000;
  int active_arm_max_area = 26000;
  int active_arm_expected_area = 20000;
  double active_arm_min_radius = 100.0;

  double prediction_seconds = 0.10;
  double output_scale = 0.50;
  bool show_preview = true;
};

EnergyConfig load_config(const std::string& path);
