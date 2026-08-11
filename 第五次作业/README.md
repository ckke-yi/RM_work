# 第五次作业：并发传感器数据处理与时间同步

`sensor_sync` 是一个独立的 C++17 程序。它用两个生产者线程分别读取相机和 IMU 文本数据，将数据写入同一个有界线程安全队列；消费者线程排空队列后，为每个相机包寻找时间戳最近的 IMU 包，并将结果写入指定文件。

本工程不依赖 ROS 2，核心并发实现明确使用：

- `std::thread`
- `std::mutex`
- `std::condition_variable`

## 工程结构

```text
.
├── config/sensor_sync.conf
├── data/
│   ├── camera.txt
│   ├── imu.txt
│   └── expected_result.txt
├── docs/packet_format.md
├── include/
│   ├── sensor_sync.hpp
│   └── thread_safe_queue.hpp
├── src/main.cpp
├── CMakeLists.txt
└── verify_sample.sh
```

## 编译与运行

```bash
cmake -S . -B build
cmake --build build --parallel
./build/sensor_sync config/sensor_sync.conf
```

程序会同时向标准输出和 `data/result.txt` 写入结果。样例输出为：

```text
CAMERA 0 IMU 0 DELTA 200
CAMERA 1 IMU 2 DELTA 100
CAMERA 2 IMU 3 DELTA 100
CAMERA 3 UNMATCHED
```

一键验证样例：

```bash
chmod +x verify_sample.sh
./verify_sample.sh
```

输入格式和输出格式详见 `docs/packet_format.md`。配置文件中的相对路径以配置文件所在目录为基准。

## 实现说明

两个生产者以不同延迟读取文本文件，用于模拟相机 100 Hz、IMU 400 Hz 的不同生产频率。共享队列有固定容量，生产过快时会阻塞，不会无限占用内存。消费者在队列关闭后仍继续取出已有数据，直到队列为空。

消费者将相机和 IMU 数据分别保存。生产和消费结束后，先按时间戳及序号对 IMU 数据排序，再通过二分查找检查每个相机时间戳左右两侧的候选 IMU。只有绝对时间差不超过 `max_delta_us` 才算匹配成功。

## 问题回答

### 1. 队列已满或为空时，线程如何进入等待状态？

`push` 在队列已满时通过 `not_full.wait(lock, predicate)` 阻塞生产者；`pop` 在队列为空且尚未关闭时通过 `not_empty.wait(lock, predicate)` 阻塞消费者。条件变量会释放互斥锁并休眠，收到通知后重新加锁并检查谓词，因此没有忙等，也能抵抗虚假唤醒。

### 2. 如何判断最后一个生产者已经结束？

`ProducerTracker` 使用原子计数器记录剩余生产者数量。每个生产者通过 RAII 的 `CompletionGuard` 在退出时将计数减一；使计数从 1 变为 0 的最后一个生产者负责关闭队列。无论正常结束、提前退出还是发生异常，完成通知都不会遗漏。

### 3. 为什么关闭队列后仍然需要允许消费者读取剩余数据？

关闭只表示以后不会再产生新数据，不代表队列里已经没有数据。如果关闭后立即禁止读取，已经成功入队但尚未消费的数据会丢失。本工程的 `pop` 会在“队列已关闭并且为空”时才返回 `false`。

### 4. 时间戳匹配算法的具体流程和复杂度是什么？

先将 IMU 数据按“时间戳、序号”排序，复杂度为 `O(I log I)`。对每个相机包使用 `lower_bound` 找到第一个不小于相机时间戳的 IMU，只比较该位置与前一个位置，复杂度为 `O(log I)`。总时间复杂度是 `O(I log I + C log I)`，额外存储复杂度是 `O(C + I + Q)`，其中 `Q` 是有界队列容量。

### 5. 如何处理时间差相同、时间戳相同等边界情况？

候选选择规则固定为：先选绝对时间差较小者；时间差相同时选时间戳较早者；时间戳仍相同时选序号较小者。等于 `max_delta_us` 的候选允许匹配，超过该阈值则输出 `UNMATCHED`。IMU 数据为空时，所有相机包均输出 `UNMATCHED`。

### 6. 如果某个生产者读取文件失败，如何保证其他线程能够正常退出？

生产者捕获异常后保存首个 `exception_ptr` 并立即关闭队列。关闭操作会同时唤醒等待入队的生产者和等待出队的消费者；其他生产者的 `push` 返回 `false` 后结束，消费者继续排空现有数据后结束。主线程 `join` 三个线程，再重新抛出错误并以非零状态退出，不会遗留后台线程或发生死锁。
