#include "color_detector.hpp"
#include "target_tracker.hpp"
#include "thread_safe_queue.hpp"
#include "vision_config.hpp"
#include "vision_types.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

class ErrorState {
 public:
  void capture(std::exception_ptr error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!error_) {
      error_ = std::move(error);
    }
  }

  void rethrow_if_present() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (error_) {
      std::rethrow_exception(error_);
    }
  }

 private:
  mutable std::mutex mutex_;
  std::exception_ptr error_;
};

void close_all(
    ThreadSafeQueue<ImageFrame>& frame_queue,
    ThreadSafeQueue<DetectionPacket>& observation_queue,
    ThreadSafeQueue<TrackingPacket>& tracking_queue) {
  frame_queue.close();
  observation_queue.close();
  tracking_queue.close();
}

std::int64_t frame_timestamp(std::uint64_t sequence, double fps) {
  const long double timestamp =
      static_cast<long double>(sequence) * 1'000'000.0L / fps;
  if (timestamp >= static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    throw std::runtime_error("video is too long for microsecond timestamps");
  }
  return static_cast<std::int64_t>(std::llround(timestamp));
}

void run_frame_source(
    const VisionConfig& config,
    ThreadSafeQueue<ImageFrame>& frame_queue,
    ThreadSafeQueue<DetectionPacket>& observation_queue,
    ThreadSafeQueue<TrackingPacket>& tracking_queue,
    ErrorState& errors) noexcept {
  try {
    cv::VideoCapture capture(config.input_video);
    if (!capture.isOpened()) {
      throw std::runtime_error("cannot open input video: " + config.input_video);
    }

    double fps = capture.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(fps) || fps <= 0.0) {
      fps = 30.0;
    }

    std::uint64_t sequence = 0;
    std::int64_t previous_timestamp = -1;
    cv::Mat image;
    while (capture.read(image)) {
      if (image.empty()) {
        throw std::runtime_error("input video returned an empty frame");
      }
      std::int64_t timestamp = frame_timestamp(sequence, fps);
      if (timestamp <= previous_timestamp) {
        timestamp = previous_timestamp + 1;
      }
      previous_timestamp = timestamp;

      ImageFrame frame{sequence, timestamp, image.clone()};
      if (!frame_queue.push(std::move(frame))) {
        return;
      }
      ++sequence;
    }

    frame_queue.close();
  } catch (...) {
    errors.capture(std::current_exception());
    close_all(frame_queue, observation_queue, tracking_queue);
  }
}

void run_color_detector(
    const VisionConfig& config,
    ThreadSafeQueue<ImageFrame>& frame_queue,
    ThreadSafeQueue<DetectionPacket>& observation_queue,
    ThreadSafeQueue<TrackingPacket>& tracking_queue,
    ErrorState& errors) noexcept {
  try {
    ColorDetector detector(config);
    ImageFrame frame;
    while (frame_queue.pop(frame)) {
      TargetObservation observation = detector.detect(frame);
      DetectionPacket packet{std::move(frame), observation};
      if (!observation_queue.push(std::move(packet))) {
        return;
      }
    }
    observation_queue.close();
  } catch (...) {
    errors.capture(std::current_exception());
    close_all(frame_queue, observation_queue, tracking_queue);
  }
}

void run_tracker(
    const VisionConfig& config,
    ThreadSafeQueue<ImageFrame>& frame_queue,
    ThreadSafeQueue<DetectionPacket>& observation_queue,
    ThreadSafeQueue<TrackingPacket>& tracking_queue,
    ErrorState& errors) noexcept {
  try {
    TargetTracker tracker(config);
    DetectionPacket packet;
    while (observation_queue.pop(packet)) {
      TrackingResult tracking = tracker.update(packet.observation);
      TrackingPacket result{
          std::move(packet.frame), packet.observation, std::move(tracking)};
      if (!tracking_queue.push(std::move(result))) {
        return;
      }
    }
    tracking_queue.close();
  } catch (...) {
    errors.capture(std::current_exception());
    close_all(frame_queue, observation_queue, tracking_queue);
  }
}

void write_point(std::ostream& output, const cv::Point2f& point, bool valid) {
  if (!valid || !std::isfinite(point.x) || !std::isfinite(point.y)) {
    output << "nan,nan";
    return;
  }
  output << point.x << ',' << point.y;
}

cv::Point to_pixel(const cv::Point2f& point) {
  return {
      static_cast<int>(std::lround(point.x)),
      static_cast<int>(std::lround(point.y))};
}

void draw_cross(cv::Mat& image, const cv::Point2f& point, const cv::Scalar& color) {
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return;
  }
  cv::drawMarker(image, to_pixel(point), color, cv::MARKER_CROSS, 16, 2);
}

void run_result_writer(
    const VisionConfig& config,
    ThreadSafeQueue<ImageFrame>& frame_queue,
    ThreadSafeQueue<DetectionPacket>& observation_queue,
    ThreadSafeQueue<TrackingPacket>& tracking_queue,
    ErrorState& errors) noexcept {
  try {
    const std::filesystem::path csv_path(config.output_csv);
    const std::filesystem::path video_path(config.output_video);
    if (!csv_path.parent_path().empty()) {
      std::filesystem::create_directories(csv_path.parent_path());
    }
    if (!video_path.parent_path().empty()) {
      std::filesystem::create_directories(video_path.parent_path());
    }

    std::ofstream csv(csv_path);
    if (!csv) {
      throw std::runtime_error("cannot open output CSV: " + config.output_csv);
    }
    csv << "frame_id,timestamp_us,detected,predicted,measure_x,measure_y,"
           "track_x,track_y,vx,vy\n";
    csv << std::fixed << std::setprecision(3);

    cv::VideoWriter video;
    TrackingPacket packet;
    while (tracking_queue.pop(packet)) {
      const TargetObservation& observation = packet.observation;
      const TrackingResult& tracking = packet.tracking;
      csv << packet.frame.sequence << ',' << packet.frame.timestamp_us << ','
          << (observation.detected ? 1 : 0) << ','
          << (tracking.predicted ? 1 : 0) << ',';
      write_point(csv, observation.center, observation.detected);
      csv << ',';
      write_point(csv, tracking.filtered_position, tracking.valid);
      csv << ',';
      if (tracking.valid && std::isfinite(tracking.velocity.x) &&
          std::isfinite(tracking.velocity.y)) {
        csv << tracking.velocity.x << ',' << tracking.velocity.y;
      } else {
        csv << "nan,nan";
      }
      csv << '\n';

      cv::Mat annotated = packet.frame.image.clone();
      if (annotated.empty()) {
        throw std::runtime_error("cannot annotate an empty frame");
      }
      if (observation.detected && observation.radius > 0.0F) {
        cv::circle(
            annotated, to_pixel(observation.center),
            std::max(1, static_cast<int>(std::lround(observation.radius))),
            cv::Scalar(0, 255, 0), 2);
      }
      if (tracking.valid) {
        const cv::Scalar color = tracking.predicted
            ? cv::Scalar(0, 165, 255)      // orange prediction
            : cv::Scalar(0, 0, 255);      // red filtered position
        draw_cross(annotated, tracking.filtered_position, color);
      }

      if (!video.isOpened()) {
        video.open(
            video_path.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            30.0, annotated.size(), true);
        if (!video.isOpened()) {
          video.open(
              video_path.string(), cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
              30.0, annotated.size(), true);
        }
        if (!video.isOpened()) {
          throw std::runtime_error("cannot open output video: " + config.output_video);
        }
      }
      video.write(annotated);
    }

    if (!csv) {
      throw std::runtime_error("failed while writing output CSV: " + config.output_csv);
    }
  } catch (...) {
    errors.capture(std::current_exception());
    close_all(frame_queue, observation_queue, tracking_queue);
  }
}

int run(const std::string& config_path) {
  const VisionConfig config = load_vision_config(config_path);
  ThreadSafeQueue<ImageFrame> frame_queue(config.queue_capacity);
  ThreadSafeQueue<DetectionPacket> observation_queue(config.queue_capacity);
  ThreadSafeQueue<TrackingPacket> tracking_queue(config.queue_capacity);
  ErrorState errors;

  std::thread source;
  std::thread detector;
  std::thread tracker;
  std::thread writer;
  try {
    source = std::thread(
        run_frame_source, std::cref(config), std::ref(frame_queue),
        std::ref(observation_queue), std::ref(tracking_queue), std::ref(errors));
    detector = std::thread(
        run_color_detector, std::cref(config), std::ref(frame_queue),
        std::ref(observation_queue), std::ref(tracking_queue), std::ref(errors));
    tracker = std::thread(
        run_tracker, std::cref(config), std::ref(frame_queue),
        std::ref(observation_queue), std::ref(tracking_queue), std::ref(errors));
    writer = std::thread(
        run_result_writer, std::cref(config), std::ref(frame_queue),
        std::ref(observation_queue), std::ref(tracking_queue), std::ref(errors));
  } catch (...) {
    close_all(frame_queue, observation_queue, tracking_queue);
    if (source.joinable()) source.join();
    if (detector.joinable()) detector.join();
    if (tracker.joinable()) tracker.join();
    if (writer.joinable()) writer.join();
    throw;
  }

  source.join();
  detector.join();
  tracker.join();
  writer.join();
  errors.rethrow_if_present();
  return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    if (argc > 2) {
      throw std::runtime_error("usage: rm_vision_sim [config_file]");
    }
    const std::string config_path = argc == 2 ? argv[1] : "config/vision.conf";
    return run(config_path);
  } catch (const std::exception& error) {
    std::cerr << "rm_vision_sim: " << error.what() << '\n';
    return 1;
  }
}
