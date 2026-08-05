#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor() : Node("img_processor") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&ImageProcessor::topic_callback, this, std::placeholders::_1));
    }

private:
    void process_color(const cv::Mat& mask, cv::Mat& result_frame, const cv::Scalar& color, const std::string& label) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            // 计算轮廓面积
            double area = cv::contourArea(contour);

            // 过滤面积过小的噪声
            if (area > 800) {
                cv::Rect box = cv::boundingRect(contour);

                // 计算色块质心坐标 (cx, cy)
                cv::Moments M = cv::moments(contour);
                if (M.m00 == 0) continue;
                int cx = static_cast<int>(M.m10 / M.m00);
                int cy = static_cast<int>(M.m01 / M.m00);

                // 1. 绘制指定颜色的外接矩形框 (红色或蓝色)
                cv::rectangle(result_frame, box, color, 2);

                // 2. 绘制中心点
                cv::circle(result_frame, cv::Point(cx, cy), 3, color, -1);

                // 3. 构造文本信息：中心坐标和面积
                std::string text_coord = label + " (" + std::to_string(cx) + ", " + std::to_string(cy) + ")";
                std::string text_area = "Area: " + std::to_string(static_cast<int>(area));

                // 4. 将坐标与面积在矩形框上方分两行绘制
                cv::putText(result_frame, text_coord, cv::Point(box.x, box.y - 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                cv::putText(result_frame, text_area, cv::Point(box.x, box.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }
        }
    }

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;

        // --- 方法 1：HSV 空间阈值分割 (识别蓝色) ---
        cv::Mat hsv, blue_mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(100, 150, 50), cv::Scalar(140, 255, 255), blue_mask);

        // --- 方法 2：BGR 通道差值阈值法 (识别红色) ---
        std::vector<cv::Mat> channels;
        cv::split(frame, channels);
        cv::Mat r = channels[2], g = channels[1], b = channels[0];

        cv::Mat red_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        for (int row = 0; row < frame.rows; ++row) {
            for (int col = 0; col < frame.cols; ++col) {
                uchar r_val = r.at<uchar>(row, col);
                uchar g_val = g.at<uchar>(row, col);
                uchar b_val = b.at<uchar>(row, col);
                if (r_val > 120 && (r_val - b_val > 40) && (r_val - g_val > 40)) {
                    red_mask.at<uchar>(row, col) = 255;
                }
            }
        }

        // 合并生成二值化效果图 (Threshold 窗口)
        cv::Mat combined_threshold;
        cv::bitwise_or(blue_mask, red_mask, combined_threshold);

        cv::Mat result_frame = frame.clone();

        // 分别绘制：蓝色框标注 Blue，红色框标注 Red
        process_color(blue_mask, result_frame, cv::Scalar(255, 0, 0), "Blue");
        process_color(red_mask, result_frame, cv::Scalar(0, 0, 255), "Red");

        // 图像显示
        cv::imshow("Original", result_frame);
        cv::imshow("Threshold", combined_threshold);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
