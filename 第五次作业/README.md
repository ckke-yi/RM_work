# 第五次作业

本作业实现 ROS 2 并发传感器数据处理与时间同步：

- `img_publisher` 节点调用摄像头，以约 30 FPS 发布 `camera/image_raw`。
- `sync_node` 节点并发订阅 `camera/image_raw` 和 `imu/data`。
- 使用 `message_filters::ApproximateTime` 对图像与 IMU 消息进行近似时间同步。
- 同步成功后输出两组消息的时间戳和时间差。

工程中同时保留了第四次作业的 `img_processor` 节点。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- OpenCV
- `cv_bridge`
- `message_filters`

## 编译

将 `color_detector_cpp` 放入 ROS 2 工作空间的 `src` 目录后执行：

```bash
cd ~/ros2_ws
colcon build --packages-select color_detector_cpp
source install/setup.bash
```

## 运行

终端 1，发布摄像头图像：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run color_detector_cpp img_publisher
```

终端 2，启动实际或模拟的 IMU 发布节点，使其向 `imu/data` 发布带有当前 `header.stamp` 的 `sensor_msgs/msg/Imu` 消息。

终端 3，启动同步节点：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run color_detector_cpp sync_node
```

同步节点会持续输出图像时间戳、IMU 时间戳及两者的时间差。
