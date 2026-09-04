# 自瞄作业二：能量机关轮廓识别

本工程使用 C++17 和 OpenCV 完成能量机关视频的轮廓识别，识别对象包括：

- 待激活能量机关的红色发光轮廓；
- 机关中心的红色 `R` 标志；
- 待激活扇叶末端装甲板的位置，并预测其未来 100 ms 的位置。

程序逐帧处理视频，使用红色 HSV 双区间分割，避免把暗背景当成目标。连通域面积、宽高和长宽比用于筛选中心 `R` 标志；面积范围和上一帧角度连续性用于区分待激活扇叶与已经点亮的扇叶。程序从待激活扇叶最外侧轮廓估算装甲板中心，并用圆周运动模型预测未来位置。

## 工程结构

```text
.
├── CMakeLists.txt
├── config/energy.conf
├── include/config.hpp
├── include/vision_types.hpp
├── input/energy_mechanism.mp4       # 本地视频，不提交到 Git
├── src/config.cpp
├── src/main.cpp
└── run.sh
```

## Ubuntu 编译运行

先安装依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev
```

把视频复制到 `input/energy_mechanism.mp4`，然后在本目录运行：

```bash
chmod +x run.sh
bash run.sh
```

程序会按作业要求在每帧调用 `cv::waitKey(100)`，并生成：

```text
output/energy_result.mp4   # 标注了轮廓、R 标志和预测位置的效果视频
output/energy_result.csv   # 每帧识别与预测坐标
```

效果视频中的标记含义：绿色轮廓是识别到的机关，紫色框是 `R` 标志，青色十字是中心预测位置，橙色圆点/十字表示待激活装甲板及其预测位置。

终端会输出总帧数、识别帧数和识别率。完整运行时应看到 `recognition_rate=100.000%`：

```text
ENERGY_RESULT frames=... recognized=... recognition_rate=100.000% predicted_frames=...
```

打开效果视频：

```bash
xdg-open output/energy_result.mp4
```

提交截图时保留终端的识别率、`cv::waitKey(100)` 对应的运行效果和视频中的轮廓/R 标志标注。
