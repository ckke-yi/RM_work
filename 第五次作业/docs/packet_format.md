# 数据包格式

相机和 IMU 输入文件采用相同的文本格式，每行表示一个数据包：

```text
<sequence> <timestamp_us> [payload]
```

- `sequence`：非负整数序号。
- `timestamp_us`：有符号 64 位微秒时间戳。
- `payload`：可选文本；同步程序只保存它，不参与匹配。
- 空行和以 `#` 开头的行会被忽略。

示例：

```text
0 1000000 camera_frame_0
1 1020000 camera_frame_1
```

输出文件每个相机包占一行。匹配成功时：

```text
CAMERA <camera_sequence> IMU <imu_sequence> DELTA <absolute_delta_us>
```

在 `max_delta_us` 范围内没有 IMU 包时：

```text
CAMERA <camera_sequence> UNMATCHED
```
