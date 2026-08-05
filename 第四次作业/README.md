# 第四次作业

本作业使用 ROS 2 和 OpenCV 创建两个节点：

- `img_publisher`：调用摄像头并发布 `camera/image_raw` 图像话题。
- `img_processor`：订阅图像话题，分别使用 HSV 阈值法识别蓝色色块、BGR 通道差值法识别红色色块，并显示轮廓、中心坐标和面积。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- OpenCV
- `cv_bridge`

## 编译

将 `color_detector_cpp` 放到 ROS 2 工作空间的 `src` 目录后执行：

```bash
cd ~/ros2_ws
colcon build --packages-select color_detector_cpp
source install/setup.bash
```

## 运行

终端 1：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run color_detector_cpp img_publisher
```

终端 2：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run color_detector_cpp img_processor
```

程序会打开 `Original` 和 `Threshold` 两个窗口，分别显示识别标注结果和二值化结果。
