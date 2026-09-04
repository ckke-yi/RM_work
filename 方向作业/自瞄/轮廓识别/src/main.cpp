#include "config.hpp"
#include "vision_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

bool finite_point(const cv::Point2f& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double point_distance(const cv::Point2f& left, const cv::Point2f& right) {
  return std::hypot(static_cast<double>(left.x - right.x),
                   static_cast<double>(left.y - right.y));
}

void draw_cross(cv::Mat& image, const cv::Point2f& point, const cv::Scalar& color) {
  if (finite_point(point)) {
    cv::drawMarker(image, cv::Point(cvRound(point.x), cvRound(point.y)), color,
                   cv::MARKER_CROSS, 46, 5, cv::LINE_AA);
  }
}

void write_point(std::ostream& output, const cv::Point2f& point, bool valid) {
  if (!valid || !finite_point(point)) {
    output << "nan,nan";
  } else {
    output << point.x << ',' << point.y;
  }
}

cv::Mat red_mask(const cv::Mat& hsv, const EnergyConfig& config) {
  cv::Mat low_red;
  cv::Mat high_red;
  cv::inRange(
      hsv, cv::Scalar(0, config.red_s_min, config.red_v_min),
      cv::Scalar(config.red_h_low, 255, 255), low_red);
  cv::inRange(
      hsv, cv::Scalar(config.red_h_high, config.red_s_min, config.red_v_min),
      cv::Scalar(179, 255, 255), high_red);
  cv::Mat mask;
  cv::bitwise_or(low_red, high_red, mask);
  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(config.morphology_kernel, config.morphology_kernel));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  return mask;
}

std::vector<RedComponent> components_from_mask(
    const cv::Mat& mask, int minimum_area, cv::Mat& labels) {
  cv::Mat stats;
  cv::Mat centers;
  const int count = cv::connectedComponentsWithStats(
      mask, labels, stats, centers, 8, CV_32S);
  std::vector<RedComponent> components;
  components.reserve(static_cast<std::size_t>(count));
  for (int label = 1; label < count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area < minimum_area) {
      continue;
    }
    const cv::Rect box(
        stats.at<int>(label, cv::CC_STAT_LEFT),
        stats.at<int>(label, cv::CC_STAT_TOP),
        stats.at<int>(label, cv::CC_STAT_WIDTH),
        stats.at<int>(label, cv::CC_STAT_HEIGHT));
    components.push_back({
        label,
        area,
        box,
        cv::Point2f(
            static_cast<float>(centers.at<double>(label, 0)),
            static_cast<float>(centers.at<double>(label, 1)))
    });
  }
  return components;
}

const RedComponent* choose_active_arm(
    const std::vector<RedComponent>& components,
    const RedComponent* marker,
    const EnergyConfig& config,
    bool has_previous_angle,
    double previous_angle) {
  if (marker == nullptr) {
    return nullptr;
  }

  const RedComponent* best = nullptr;
  double best_score = std::numeric_limits<double>::infinity();
  for (const RedComponent& component : components) {
    if (component.label == marker->label ||
        component.area < config.active_arm_min_area ||
        component.area > config.active_arm_max_area) {
      continue;
    }
    const double radius = point_distance(component.center, marker->center);
    if (radius < config.active_arm_min_radius) {
      continue;
    }
    const double angle = std::atan2(
        static_cast<double>(component.center.y - marker->center.y),
        static_cast<double>(component.center.x - marker->center.x));
    const double angle_error = has_previous_angle
        ? std::abs(wrap_angle(angle - previous_angle)) : 0.0;
    const double area_error =
        std::abs(component.area - config.active_arm_expected_area);
    const double score = area_error + angle_error * 5000.0;
    if (score < best_score) {
      best = &component;
      best_score = score;
    }
  }
  return best;
}

cv::Point2f estimate_target_center(
    const cv::Mat& labels,
    const RedComponent& arm,
    const cv::Point2f& marker_center) {
  double maximum_radius = 0.0;
  for (int y = arm.box.y; y < arm.box.y + arm.box.height; ++y) {
    const int* row = labels.ptr<int>(y);
    for (int x = arm.box.x; x < arm.box.x + arm.box.width; ++x) {
      if (row[x] == arm.label) {
        maximum_radius = std::max(
            maximum_radius,
            point_distance(cv::Point2f(static_cast<float>(x), static_cast<float>(y)),
                           marker_center));
      }
    }
  }

  // The outer 30% of the component is the rectangular target plate; the
  // inner pixels belong to the arrow-shaped light chain.
  const double minimum_target_radius = maximum_radius * 0.70;
  cv::Point2d sum(0.0, 0.0);
  int count = 0;
  for (int y = arm.box.y; y < arm.box.y + arm.box.height; ++y) {
    const int* row = labels.ptr<int>(y);
    for (int x = arm.box.x; x < arm.box.x + arm.box.width; ++x) {
      if (row[x] != arm.label) {
        continue;
      }
      const cv::Point2f point(static_cast<float>(x), static_cast<float>(y));
      if (point_distance(point, marker_center) >= minimum_target_radius) {
        sum.x += x;
        sum.y += y;
        ++count;
      }
    }
  }
  if (count == 0) {
    return arm.center;
  }
  return cv::Point2f(
      static_cast<float>(sum.x / count),
      static_cast<float>(sum.y / count));
}

bool is_marker_candidate(const RedComponent& component, const EnergyConfig& config) {
  const double aspect = component.box.width /
                        std::max(1.0, static_cast<double>(component.box.height));
  return component.area >= config.marker_min_area &&
         component.area <= config.marker_max_area &&
         component.box.width >= config.marker_min_width &&
         component.box.width <= config.marker_max_width &&
         component.box.height >= config.marker_min_height &&
         component.box.height <= config.marker_max_height &&
         aspect >= config.marker_min_aspect &&
         aspect <= config.marker_max_aspect;
}

const RedComponent* choose_marker(
    const std::vector<RedComponent>& components,
    const EnergyConfig& config,
    bool has_previous,
    const cv::Point2f& previous_center) {
  std::vector<const RedComponent*> candidates;
  for (const RedComponent& component : components) {
    if (is_marker_candidate(component, config)) {
      candidates.push_back(&component);
    }
  }
  if (candidates.empty()) {
    return nullptr;
  }

  cv::Point2f reference = previous_center;
  if (!has_previous) {
    cv::Rect union_box;
    bool initialized = false;
    for (const RedComponent& component : components) {
      if (!initialized) {
        union_box = component.box;
        initialized = true;
      } else {
        union_box |= component.box;
      }
    }
    reference = cv::Point2f(
        union_box.x + union_box.width * 0.5F,
        union_box.y + union_box.height * 0.5F);
  }

  const RedComponent* best = candidates.front();
  auto score = [&](const RedComponent* component) {
    const double distance = point_distance(component->center, reference);
    const double area_error = std::abs(component->area - config.marker_expected_area);
    return distance + 0.08 * area_error;
  };
  for (const RedComponent* candidate : candidates) {
    if (score(candidate) < score(best)) {
      best = candidate;
    }
  }
  return best;
}

cv::Rect union_box(const std::vector<RedComponent>& components) {
  cv::Rect result;
  bool initialized = false;
  for (const RedComponent& component : components) {
    if (!initialized) {
      result = component.box;
      initialized = true;
    } else {
      result |= component.box;
    }
  }
  return result;
}

void put_status(cv::Mat& image, const std::string& text, int row, const cv::Scalar& color) {
  const int baseline = 50 + row * 48;
  cv::rectangle(image, cv::Rect(20, baseline - 38, 760, 48),
                cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(image, text, cv::Point(36, baseline - 5),
              cv::FONT_HERSHEY_SIMPLEX, 1.05, color, 3, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) {
      std::cerr << "Usage: energy_contour_recognition <config-file>\n";
      return 2;
    }
    const EnergyConfig config = load_config(argv[1]);
    cv::VideoCapture capture(config.input_video);
    if (!capture.isOpened()) {
      throw std::runtime_error("cannot open input video: " + config.input_video);
    }

    const int width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = capture.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(fps) || fps <= 0.0) {
      fps = 30.0;
    }
    if (width <= 0 || height <= 0) {
      throw std::runtime_error("input video has invalid resolution");
    }

    const int output_width = std::max(1, cvRound(width * config.output_scale));
    const int output_height = std::max(1, cvRound(height * config.output_scale));
    fs::create_directories(fs::path(config.output_video).parent_path());
    fs::create_directories(fs::path(config.output_csv).parent_path());

    cv::VideoWriter writer(
        config.output_video,
        cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
        cv::Size(output_width, output_height), true);
    if (!writer.isOpened()) {
      throw std::runtime_error("cannot create output video: " + config.output_video);
    }
    std::ofstream csv(config.output_csv);
    if (!csv) {
      throw std::runtime_error("cannot create output csv: " + config.output_csv);
    }
    csv << "frame_id,timestamp_us,recognized,r_x,r_y,mechanism_x,mechanism_y,"
           "mechanism_w,mechanism_h,target_x,target_y,predicted_r_x,predicted_r_y,"
           "predicted_target_x,predicted_target_y\n";
    csv << std::fixed << std::setprecision(3);

    if (config.show_preview) {
      cv::namedWindow("Energy contour recognition", cv::WINDOW_NORMAL);
    }

    bool has_previous_marker = false;
    cv::Point2f previous_marker;
    cv::Point2f marker_velocity(0.0F, 0.0F);
    bool has_previous_target = false;
    double previous_angle = 0.0;
    double angular_velocity = 0.0;
    cv::Point2f predicted_marker;
    cv::Point2f predicted_target;
    int frame_id = 0;
    int recognized_frames = 0;
    int predicted_frames = 0;
    bool interrupted = false;

    cv::Mat frame;
    while (capture.read(frame)) {
      const double timestamp_seconds = frame_id / fps;
      const std::int64_t timestamp_us = static_cast<std::int64_t>(
          std::llround(timestamp_seconds * 1'000'000.0));
      cv::Mat hsv;
      cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
      const cv::Mat mask = red_mask(hsv, config);
      cv::Mat labels;
      const std::vector<RedComponent> components =
          components_from_mask(mask, config.component_min_area, labels);
      const RedComponent* marker = choose_marker(
          components, config, has_previous_marker, previous_marker);
      const cv::Rect mechanism = union_box(components);
      const bool recognized = marker != nullptr && components.size() >= 2;
      if (recognized) {
        ++recognized_frames;
      }

      cv::Mat annotated = frame.clone();
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      for (const auto& contour : contours) {
        if (cv::contourArea(contour) >= config.component_min_area) {
          cv::drawContours(annotated, std::vector<std::vector<cv::Point>>{contour},
                           0, cv::Scalar(0, 255, 0), 4, cv::LINE_AA);
        }
      }
      if (!mechanism.empty()) {
        cv::rectangle(annotated, mechanism, cv::Scalar(0, 255, 255), 5, cv::LINE_AA);
      }

      const RedComponent* active_arm = choose_active_arm(
          components, marker, config, has_previous_target, previous_angle);
      cv::Point2f target_center;
      float target_radius = 0.0F;
      const bool target_found = active_arm != nullptr && marker != nullptr;
      if (target_found) {
        target_center = estimate_target_center(labels, *active_arm, marker->center);
        target_radius = static_cast<float>(
            point_distance(target_center, marker->center));
      }

      bool marker_prediction_valid = false;
      bool target_prediction_valid = false;
      if (marker != nullptr) {
        if (has_previous_marker) {
          const double dt = 1.0 / fps;
          const cv::Point2f raw_velocity =
              (marker->center - previous_marker) / static_cast<float>(dt);
          marker_velocity = marker_velocity * 0.75F + raw_velocity * 0.25F;
        }
        predicted_marker = marker->center + marker_velocity *
            static_cast<float>(config.prediction_seconds);
        marker_prediction_valid = true;

        if (target_found && target_radius > 30.0F) {
          const double angle = std::atan2(
              static_cast<double>(target_center.y - marker->center.y),
              static_cast<double>(target_center.x - marker->center.x));
          if (has_previous_target) {
            const double delta = wrap_angle(angle - previous_angle);
            const double measured_velocity = delta * fps;
            if (std::abs(delta) < 0.8) {
              angular_velocity = angular_velocity * 0.75 + measured_velocity * 0.25;
            } else {
              angular_velocity = 0.0;
            }
          }
          const double predicted_angle =
              angle + angular_velocity * config.prediction_seconds;
          predicted_target = marker->center + cv::Point2f(
              target_radius * static_cast<float>(std::cos(predicted_angle)),
              target_radius * static_cast<float>(std::sin(predicted_angle)));
          target_prediction_valid = true;
          previous_angle = angle;
          has_previous_target = true;
        }
        previous_marker = marker->center;
        has_previous_marker = true;
      }
      if (target_prediction_valid) {
        ++predicted_frames;
      }

      if (marker != nullptr) {
        cv::rectangle(annotated, marker->box, cv::Scalar(255, 0, 255), 5, cv::LINE_AA);
        cv::circle(annotated, marker->center, 16, cv::Scalar(255, 0, 255), cv::FILLED);
      }
      if (target_found) {
        cv::circle(annotated, target_center, 14, cv::Scalar(0, 165, 255), cv::FILLED);
      }
      if (marker_prediction_valid) {
        draw_cross(annotated, predicted_marker, cv::Scalar(255, 255, 0));
      }
      if (target_prediction_valid) {
        draw_cross(annotated, predicted_target, cv::Scalar(0, 165, 255));
        cv::line(annotated, marker->center, predicted_target,
                 cv::Scalar(0, 165, 255), 4, cv::LINE_AA);
      }

      put_status(annotated, recognized ? "ENERGY MECHANISM: FOUND" :
                                     "ENERGY MECHANISM: NOT FOUND", 0,
                 recognized ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
      put_status(annotated, marker != nullptr ? "R MARKER: FOUND" :
                                     "R MARKER: NOT FOUND", 1,
                 marker != nullptr ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 255));
      put_status(annotated, target_prediction_valid ? "PREDICTION: TARGET +100 ms" :
                                     "PREDICTION: CENTER +100 ms", 2,
                 cv::Scalar(255, 255, 0));

      cv::Mat output_frame;
      cv::resize(annotated, output_frame, cv::Size(output_width, output_height),
                 0.0, 0.0, cv::INTER_AREA);
      writer.write(output_frame);
      csv << frame_id << ',' << timestamp_us << ',' << (recognized ? 1 : 0) << ',';
      write_point(csv, marker == nullptr ? cv::Point2f() : marker->center, marker != nullptr);
      csv << ',';
      if (mechanism.empty()) {
        csv << "nan,nan,nan,nan";
      } else {
        csv << mechanism.x + mechanism.width * 0.5 << ','
            << mechanism.y + mechanism.height * 0.5 << ','
            << mechanism.width << ',' << mechanism.height;
      }
      csv << ',';
      write_point(csv, target_center, target_found);
      csv << ',';
      write_point(csv, predicted_marker, marker_prediction_valid);
      csv << ',';
      write_point(csv, predicted_target, target_prediction_valid);
      csv << '\n';

      if (config.show_preview) {
        cv::imshow("Energy contour recognition", output_frame);
      }
      ++frame_id;
      // Required by the assignment: keep the preview visible for 100 ms per frame.
      const int key = cv::waitKey(100);
      if (key == 27) {
        interrupted = true;
        break;
      }
    }

    capture.release();
    writer.release();
    csv.close();
    if (config.show_preview) {
      cv::destroyAllWindows();
    }
    const double recognition_rate = frame_id == 0
        ? 0.0 : 100.0 * recognized_frames / static_cast<double>(frame_id);
    std::cout << std::fixed << std::setprecision(3)
              << "ENERGY_RESULT frames=" << frame_id
              << " recognized=" << recognized_frames
              << " recognition_rate=" << recognition_rate << "%"
              << " predicted_frames=" << predicted_frames
              << " resolution=" << width << "x" << height << '\n'
              << "saved_video=" << config.output_video << '\n'
              << "saved_csv=" << config.output_csv << '\n";
    if (interrupted) {
      std::cout << "Preview interrupted by ESC; rerun without pressing ESC for the full video.\n";
    }
    return recognized_frames == frame_id ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "energy_contour_recognition: " << error.what() << '\n';
    return 1;
  }
}
