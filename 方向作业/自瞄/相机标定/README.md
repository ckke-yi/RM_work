# 相机标定与畸变校正

本工程对应自瞄方向的相机标定作业，使用 C++17 和 OpenCV 完成：

1. 从标定视频中按时间抽取棋盘格画面；
2. 检测 `9 x 6` 个内角点并进行亚像素优化；
3. 使用 `calibrateCamera` 计算相机内参和畸变系数；
4. 输出重投影误差和 `camera.yaml`；
5. 使用 `initUndistortRectifyMap` 与 `remap` 生成原始/去畸变左右对比视频。

## 标定板参数

在线生成器设置为 `10 x 7` 个方格，OpenCV 的内角点参数因此是 `9 x 6`。实测每格边长为 `16 mm`，代码使用：

```text
board_cols = 9
board_rows = 6
square_size_mm = 16.0
```

标定和校正必须使用相同的相机分辨率。本次录像为 `1280 x 720`，约 45 秒，包含不同距离和倾斜姿态。

## Ubuntu 运行

安装依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev
```

将相机原始录像复制到 `input/`，文件名保持为：

```text
input/WIN_20260904_02_30_18_Pro.mp4
```

然后在本目录执行：

```bash
chmod +x run_all.sh
bash run_all.sh
```

成功时终端会输出类似：

```text
CALIBRATION_RESULT views=... image=1280x720 rms_reprojection_error=... mean_reprojection_error=...
UNDISTORT_RESULT frames=... resolution=1280x720 output=output/undistorted_comparison.avi
Camera calibration and undistortion completed.
```

作业要求的 `< 0.5` 误差应看 `mean_reprojection_error`。程序同时输出 RMS 和最大单视图误差，便于复核。

## 结果文件

```text
output/camera.yaml                    # 相机矩阵、畸变系数和误差
output/calibration_points.avi         # 棋盘格角点检测效果
output/undistorted_comparison.avi     # 左原图、右去畸变图
```

播放对比视频：

```bash
xdg-open output/undistorted_comparison.avi
```

提交时保留终端中重投影误差、左右对比视频画面，以及 GitHub 中的源码和 README。个人原始录像不放入公开仓库。
