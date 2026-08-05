#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SyncSensorNode : public rclcpp::Node {
public:
    SyncSensorNode() : Node("sync_sensor_node") {
        // 定义时间同步策略 (近似时间同步, 队列大小为 10)
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Imu> SyncPolicy;

        // 订阅图像和 IMU 话题
        image_sub_.subscribe(this, "camera/image_raw");
        imu_sub_.subscribe(this, "imu/data");

        // 初始化同步器
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, imu_sub_);

        // 绑定回调函数
        sync_->registerCallback(
            std::bind(&SyncSensorNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "并发传感器数据处理与时间同步节点已启动！");
    }

private:
    void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
    {
        // 计算两组数据之间的时间差 (单位：毫秒)
        rclcpp::Time img_stamp = img_msg->header.stamp;
        rclcpp::Time imu_stamp = imu_msg->header.stamp;
        double diff_ms = std::abs((img_stamp - imu_stamp).seconds() * 1000.0);

        RCLCPP_INFO(this->get_logger(),
            " [同步成功] 图像时间戳: %.9f | IMU 时间戳: %.9f | 时间差: %.2f ms",
            img_stamp.seconds(),
            imu_stamp.seconds(),
            diff_ms);
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Imu> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncSensorNode>());
    rclcpp::shutdown();
    return 0;
}
