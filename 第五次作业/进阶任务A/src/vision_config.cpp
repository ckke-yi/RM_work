#include "vision_config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {

std::string trim(std::string value) {
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

template <typename T, typename Parser>
T parse_number(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key,
    Parser parser) {
  const auto it = values.find(key);
  if (it == values.end()) {
    throw std::runtime_error("missing config key: " + key);
  }

  std::size_t consumed = 0;
  try {
    const T result = parser(it->second, &consumed);
    if (consumed != it->second.size()) {
      throw std::runtime_error("invalid config value for: " + key);
    }
    return result;
  } catch (const std::invalid_argument&) {
    throw std::runtime_error("invalid config value for: " + key);
  } catch (const std::out_of_range&) {
    throw std::runtime_error("config value out of range for: " + key);
  }
}

}  // namespace

VisionConfig load_vision_config(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open config file: " + path);
  }

  std::unordered_map<std::string, std::string> values;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    line = trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    const auto separator = line.find('=');
    if (separator == std::string::npos) {
      throw std::runtime_error(
          "invalid config line " + std::to_string(line_number) + ": " + line);
    }

    const std::string key = trim(line.substr(0, separator));
    const std::string value = trim(line.substr(separator + 1));
    if (key.empty() || value.empty() || !values.emplace(key, value).second) {
      throw std::runtime_error("invalid or duplicate config key: " + key);
    }
  }
  if (!input.eof()) {
    throw std::runtime_error("failed while reading config file: " + path);
  }

  const auto required = [&values](const std::string& key) -> const std::string& {
    const auto it = values.find(key);
    if (it == values.end()) {
      throw std::runtime_error("missing config key: " + key);
    }
    return it->second;
  };

  const std::filesystem::path base =
      std::filesystem::absolute(std::filesystem::path(path)).parent_path();
  const auto resolve = [&base](const std::string& value) {
    const std::filesystem::path candidate(value);
    return (candidate.is_absolute() ? candidate : base / candidate)
        .lexically_normal()
        .string();
  };

  VisionConfig config;
  config.input_video = resolve(required("input_video"));
  config.output_video = resolve(required("output_video"));
  config.output_csv = resolve(required("output_csv"));

  const long long queue_capacity = parse_number<long long>(
      values, "queue_capacity", [](const std::string& text, std::size_t* used) {
        return std::stoll(text, used);
      });
  config.queue_capacity = static_cast<std::size_t>(queue_capacity);
  config.h_min = parse_number<int>(values, "h_min", [](const auto& text, auto* used) {
    return std::stoi(text, used);
  });
  config.h_max = parse_number<int>(values, "h_max", [](const auto& text, auto* used) {
    return std::stoi(text, used);
  });
  config.s_min = parse_number<int>(values, "s_min", [](const auto& text, auto* used) {
    return std::stoi(text, used);
  });
  config.s_max = parse_number<int>(values, "s_max", [](const auto& text, auto* used) {
    return std::stoi(text, used);
  });
  config.v_min = parse_number<int>(values, "v_min", [](const auto& text, auto* used) {
    return std::stoi(text, used);
  });
  config.v_max = parse_number<int>(values, "v_max", [](const auto& text, auto* used) {
    return std::stoi(text, used);
  });
  config.min_area = parse_number<double>(
      values, "min_area", [](const auto& text, auto* used) {
        return std::stod(text, used);
      });
  config.max_area = parse_number<double>(
      values, "max_area", [](const auto& text, auto* used) {
        return std::stod(text, used);
      });
  config.max_missed_frames = parse_number<int>(
      values, "max_missed_frames", [](const auto& text, auto* used) {
        return std::stoi(text, used);
      });
  config.max_match_distance = parse_number<double>(
      values, "max_match_distance", [](const auto& text, auto* used) {
        return std::stod(text, used);
      });
  config.alpha = parse_number<double>(values, "alpha", [](const auto& text, auto* used) {
    return std::stod(text, used);
  });
  config.beta = parse_number<double>(values, "beta", [](const auto& text, auto* used) {
    return std::stod(text, used);
  });

  if (queue_capacity <= 0 || queue_capacity > 65536) {
    throw std::runtime_error("queue_capacity must be in [1, 65536]");
  }
  if (config.h_min < 0 || config.h_max > 179 || config.h_min > config.h_max ||
      config.s_min < 0 || config.s_max > 255 || config.s_min > config.s_max ||
      config.v_min < 0 || config.v_max > 255 || config.v_min > config.v_max) {
    throw std::runtime_error("invalid HSV range");
  }
  if (!std::isfinite(config.min_area) || !std::isfinite(config.max_area) ||
      !std::isfinite(config.max_match_distance) || !std::isfinite(config.alpha) ||
      !std::isfinite(config.beta) || config.min_area <= 0.0 ||
      config.max_area < config.min_area ||
      config.max_missed_frames < 0 || config.max_match_distance <= 0.0 ||
      config.alpha <= 0.0 || config.alpha > 1.0 || config.beta < 0.0 ||
      config.beta > 1.0) {
    throw std::runtime_error("invalid detector or tracker configuration");
  }
  return config;
}
