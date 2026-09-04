#include "config.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace fs = std::filesystem;

namespace {

std::string trim(std::string value) {
  const auto first = std::find_if_not(value.begin(), value.end(),
                                      [](unsigned char ch) { return std::isspace(ch) != 0; });
  value.erase(value.begin(), first);
  const auto last = std::find_if_not(value.rbegin(), value.rend(),
                                     [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
  value.erase(last, value.end());
  return value;
}

bool parse_bool(const std::string& value) {
  return value == "1" || value == "true" || value == "TRUE" || value == "yes";
}

}  // namespace

EnergyConfig load_config(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open config: " + path);
  }

  EnergyConfig config;
  const fs::path base = fs::absolute(fs::path(path)).parent_path();
  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.erase(comment);
    }
    line = trim(std::move(line));
    if (line.empty()) {
      continue;
    }
    const auto separator = line.find('=');
    if (separator == std::string::npos) {
      throw std::runtime_error("invalid config line " + std::to_string(line_number));
    }
    const std::string key = trim(line.substr(0, separator));
    const std::string value = trim(line.substr(separator + 1));
    try {
      if (key == "input_video") config.input_video = (base / value).lexically_normal().string();
      else if (key == "output_video") config.output_video = (base / value).lexically_normal().string();
      else if (key == "output_csv") config.output_csv = (base / value).lexically_normal().string();
      else if (key == "red_h_low") config.red_h_low = std::stoi(value);
      else if (key == "red_h_high") config.red_h_high = std::stoi(value);
      else if (key == "red_s_min") config.red_s_min = std::stoi(value);
      else if (key == "red_v_min") config.red_v_min = std::stoi(value);
      else if (key == "morphology_kernel") config.morphology_kernel = std::stoi(value);
      else if (key == "component_min_area") config.component_min_area = std::stoi(value);
      else if (key == "marker_min_area") config.marker_min_area = std::stoi(value);
      else if (key == "marker_max_area") config.marker_max_area = std::stoi(value);
      else if (key == "marker_min_width") config.marker_min_width = std::stoi(value);
      else if (key == "marker_max_width") config.marker_max_width = std::stoi(value);
      else if (key == "marker_min_height") config.marker_min_height = std::stoi(value);
      else if (key == "marker_max_height") config.marker_max_height = std::stoi(value);
      else if (key == "marker_min_aspect") config.marker_min_aspect = std::stod(value);
      else if (key == "marker_max_aspect") config.marker_max_aspect = std::stod(value);
      else if (key == "marker_expected_area") config.marker_expected_area = std::stoi(value);
      else if (key == "active_arm_min_area") config.active_arm_min_area = std::stoi(value);
      else if (key == "active_arm_max_area") config.active_arm_max_area = std::stoi(value);
      else if (key == "active_arm_expected_area") config.active_arm_expected_area = std::stoi(value);
      else if (key == "active_arm_min_radius") config.active_arm_min_radius = std::stod(value);
      else if (key == "prediction_seconds") config.prediction_seconds = std::stod(value);
      else if (key == "output_scale") config.output_scale = std::stod(value);
      else if (key == "show_preview") config.show_preview = parse_bool(value);
      else throw std::runtime_error("unknown key: " + key);
    } catch (const std::exception& error) {
      throw std::runtime_error("config line " + std::to_string(line_number) + ": " + error.what());
    }
  }

  if (config.input_video.empty() || config.output_video.empty() || config.output_csv.empty()) {
    throw std::runtime_error("config must define input_video, output_video and output_csv");
  }
  if (config.morphology_kernel < 1 || config.morphology_kernel % 2 == 0) {
    throw std::runtime_error("morphology_kernel must be a positive odd number");
  }
  if (config.active_arm_min_area < 1 ||
      config.active_arm_max_area < config.active_arm_min_area ||
      config.active_arm_expected_area < config.active_arm_min_area ||
      config.active_arm_expected_area > config.active_arm_max_area ||
      config.active_arm_min_radius <= 0.0) {
    throw std::runtime_error("invalid active arm parameters");
  }
  if (!(config.output_scale > 0.0 && config.output_scale <= 1.0)) {
    throw std::runtime_error("output_scale must be in (0, 1]");
  }
  return config;
}
