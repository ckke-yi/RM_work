#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("img_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        // 打开摄像头 0
        cap_.open(0, cv::CAP_V4L2);

        // 根据 ffplay 输出强制指定 YUYV 像素格式与分辨率
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开摄像头!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "摄像头成功打开并开始发布 Topic: camera/image_raw");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        if (!cap_.isOpened()) return;

        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        std_msgs::msg::Header header;
        header.stamp = this->now();
        cv_bridge::CvImage cv_img(header, "bgr8", frame);

        publisher_->publish(*cv_img.toImageMsg());
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
