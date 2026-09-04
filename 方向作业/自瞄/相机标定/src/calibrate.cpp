#include "config.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <cmath>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

bool findCorners(const cv::Mat& frame, const cv::Size& boardSize, std::vector<cv::Point2f>& corners) {
    const cv::Mat gray = [&]() {
        cv::Mat converted;
        cv::cvtColor(frame, converted, cv::COLOR_BGR2GRAY);
        return converted;
    }();

    bool found = false;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 5)
    found = cv::findChessboardCornersSB(
        gray,
        boardSize,
        corners,
        cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
#endif
    if (!found) {
        found = cv::findChessboardCorners(
            gray,
            boardSize,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cv::cornerSubPix(
                gray,
                corners,
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001));
        }
    }
    return found;
}

double meanReprojectionError(
    const std::vector<std::vector<cv::Point3f>>& objectPoints,
    const std::vector<std::vector<cv::Point2f>>& imagePoints,
    const std::vector<cv::Mat>& rotations,
    const std::vector<cv::Mat>& translations,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distortion,
    double& maxViewError) {
    double totalError = 0.0;
    std::size_t totalPoints = 0;
    maxViewError = 0.0;
    for (std::size_t i = 0; i < objectPoints.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            objectPoints[i], rotations[i], translations[i], cameraMatrix, distortion, projected);
        double viewError = 0.0;
        for (std::size_t j = 0; j < projected.size(); ++j) {
            viewError += cv::norm(imagePoints[i][j] - projected[j]);
        }
        viewError /= static_cast<double>(projected.size());
        maxViewError = std::max(maxViewError, viewError);
        totalError += viewError * static_cast<double>(projected.size());
        totalPoints += projected.size();
    }
    return totalError / static_cast<double>(totalPoints);
}

void putLabel(cv::Mat& frame, const std::string& label, const cv::Scalar& color) {
    cv::rectangle(frame, cv::Rect(0, 0, 430, 42), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(frame, label, cv::Point(12, 29), cv::FONT_HERSHEY_SIMPLEX, 0.72, color, 2, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: rm_camera_calibrate <config-file>\n";
        return 2;
    }

    try {
        const Config config = Config::load(argv[1]);
        const fs::path videoPath = config.getString("input_video");
        const fs::path outputDir = config.getString("output_dir");
        const int boardCols = config.getInt("board_cols");
        const int boardRows = config.getInt("board_rows");
        const double squareSize = config.getDouble("square_size_mm");
        const int sampleStep = config.getInt("sample_step_frames");
        const int maxSamples = config.getInt("max_samples");

        if (!fs::exists(videoPath)) {
            throw std::runtime_error("input video does not exist: " + videoPath.string());
        }
        fs::create_directories(outputDir);

        cv::VideoCapture capture(videoPath.string());
        if (!capture.isOpened()) {
            throw std::runtime_error("cannot open input video: " + videoPath.string());
        }
        const int width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
        const int height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
        const double fps = capture.get(cv::CAP_PROP_FPS) > 1.0 ? capture.get(cv::CAP_PROP_FPS) : 30.0;
        const cv::Size imageSize(width, height);
        const cv::Size boardSize(boardCols, boardRows);

        const fs::path annotatedPath = outputDir / "calibration_points.avi";
        cv::VideoWriter annotated(
            annotatedPath.string(),
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            fps,
            imageSize);
        if (!annotated.isOpened()) {
            throw std::runtime_error("cannot create calibration video: " + annotatedPath.string());
        }

        std::vector<std::vector<cv::Point3f>> objectPoints;
        std::vector<std::vector<cv::Point2f>> imagePoints;
        std::vector<cv::Point3f> objectTemplate;
        objectTemplate.reserve(static_cast<std::size_t>(boardCols * boardRows));
        for (int row = 0; row < boardRows; ++row) {
            for (int col = 0; col < boardCols; ++col) {
                objectTemplate.emplace_back(
                    static_cast<float>(col * squareSize),
                    static_cast<float>(row * squareSize),
                    0.0F);
            }
        }

        cv::Mat frame;
        int frameId = 0;
        int detected = 0;
        while (capture.read(frame)) {
            std::vector<cv::Point2f> corners;
            const bool sampled = frameId % sampleStep == 0 && detected < maxSamples;
            const bool found = sampled && findCorners(frame, boardSize, corners);
            if (found) {
                objectPoints.push_back(objectTemplate);
                imagePoints.push_back(corners);
                ++detected;
                cv::drawChessboardCorners(frame, boardSize, corners, true);
                putLabel(frame, "CALIBRATION CORNERS FOUND", cv::Scalar(0, 255, 0));
            } else if (sampled) {
                putLabel(frame, "NO CHESSBOARD DETECTED", cv::Scalar(0, 0, 255));
            }
            annotated.write(frame);
            ++frameId;
        }
        capture.release();
        annotated.release();

        if (imagePoints.size() < 8) {
            throw std::runtime_error("too few valid calibration views: " + std::to_string(imagePoints.size()));
        }

        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat distortion = cv::Mat::zeros(8, 1, CV_64F);
        std::vector<cv::Mat> rotations;
        std::vector<cv::Mat> translations;
        const double rms = cv::calibrateCamera(
            objectPoints,
            imagePoints,
            imageSize,
            cameraMatrix,
            distortion,
            rotations,
            translations,
            cv::CALIB_RATIONAL_MODEL);
        double maxViewError = 0.0;
        const double meanError = meanReprojectionError(
            objectPoints,
            imagePoints,
            rotations,
            translations,
            cameraMatrix,
            distortion,
            maxViewError);

        const fs::path calibrationPath = outputDir / "camera.yaml";
        cv::FileStorage storage(calibrationPath.string(), cv::FileStorage::WRITE);
        storage << "image_width" << width;
        storage << "image_height" << height;
        storage << "board_cols" << boardCols;
        storage << "board_rows" << boardRows;
        storage << "square_size_mm" << squareSize;
        storage << "camera_matrix" << cameraMatrix;
        storage << "distortion_coefficients" << distortion;
        storage << "rms_reprojection_error" << rms;
        storage << "mean_reprojection_error" << meanError;
        storage << "max_view_error" << maxViewError;
        storage << "calibration_views" << static_cast<int>(imagePoints.size());
        storage.release();

        std::cout << std::fixed << std::setprecision(6)
                  << "CALIBRATION_RESULT views=" << imagePoints.size()
                  << " image=" << width << "x" << height
                  << " rms_reprojection_error=" << rms
                  << " mean_reprojection_error=" << meanError
                  << " max_view_error=" << maxViewError << "\n"
                  << "camera_matrix=" << cameraMatrix << "\n"
                  << "distortion_coefficients=" << distortion.t() << "\n"
                  << "saved=" << calibrationPath.string() << "\n"
                  << "annotated_video=" << annotatedPath.string() << "\n";
        if (meanError >= 0.5) {
            std::cerr << "WARNING: mean reprojection error is not below 0.5 pixels; capture more varied views.\n";
        }
    } catch (const std::exception& error) {
        std::cerr << "calibration error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
