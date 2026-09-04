#include "config.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

namespace {

void putLabel(cv::Mat frame, const std::string& label, const cv::Scalar& color) {
    cv::rectangle(frame, cv::Rect(0, 0, 360, 42), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(frame, label, cv::Point(12, 29), cv::FONT_HERSHEY_SIMPLEX, 0.72, color, 2, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: rm_undistort <config-file>\n";
        return 2;
    }

    try {
        const Config config = Config::load(argv[1]);
        const fs::path videoPath = config.getString("input_video");
        const fs::path outputDir = config.getString("output_dir");
        const fs::path calibrationPath = outputDir / "camera.yaml";
        const fs::path outputPath = outputDir / "undistorted_comparison.avi";

        cv::FileStorage storage(calibrationPath.string(), cv::FileStorage::READ);
        if (!storage.isOpened()) {
            throw std::runtime_error("cannot open calibration file: " + calibrationPath.string());
        }
        cv::Mat cameraMatrix;
        cv::Mat distortion;
        int width = 0;
        int height = 0;
        storage["camera_matrix"] >> cameraMatrix;
        storage["distortion_coefficients"] >> distortion;
        storage["image_width"] >> width;
        storage["image_height"] >> height;
        storage.release();
        if (cameraMatrix.empty() || distortion.empty() || width <= 0 || height <= 0) {
            throw std::runtime_error("camera.yaml is missing required calibration fields");
        }

        cv::VideoCapture capture(videoPath.string());
        if (!capture.isOpened()) {
            throw std::runtime_error("cannot open input video: " + videoPath.string());
        }
        const int actualWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
        const int actualHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
        const double fps = capture.get(cv::CAP_PROP_FPS) > 1.0 ? capture.get(cv::CAP_PROP_FPS) : 30.0;
        if (actualWidth != width || actualHeight != height) {
            throw std::runtime_error(
                "video resolution does not match calibration: video=" +
                std::to_string(actualWidth) + "x" + std::to_string(actualHeight) +
                ", calibration=" + std::to_string(width) + "x" + std::to_string(height));
        }

        fs::create_directories(outputDir);
        cv::VideoWriter writer(
            outputPath.string(),
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            fps,
            cv::Size(width * 2, height));
        if (!writer.isOpened()) {
            throw std::runtime_error("cannot create output video: " + outputPath.string());
        }

        cv::Mat mapX;
        cv::Mat mapY;
        cv::initUndistortRectifyMap(
            cameraMatrix,
            distortion,
            cv::Mat(),
            cameraMatrix,
            cv::Size(width, height),
            CV_32FC1,
            mapX,
            mapY);

        cv::Mat frame;
        cv::Mat corrected;
        cv::Mat comparison;
        int frameCount = 0;
        while (capture.read(frame)) {
            cv::remap(frame, corrected, mapX, mapY, cv::INTER_LINEAR);
            comparison.create(height, width * 2, CV_8UC3);
            frame.copyTo(comparison(cv::Rect(0, 0, width, height)));
            corrected.copyTo(comparison(cv::Rect(width, 0, width, height)));
            putLabel(comparison(cv::Rect(0, 0, width, height)), "ORIGINAL", cv::Scalar(255, 255, 255));
            putLabel(comparison(cv::Rect(width, 0, width, height)), "UNDISTORTED", cv::Scalar(0, 255, 0));
            writer.write(comparison);
            ++frameCount;
        }
        capture.release();
        writer.release();

        std::cout << "UNDISTORT_RESULT frames=" << frameCount
                  << " resolution=" << width << "x" << height
                  << " output=" << outputPath.string() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "undistort error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
