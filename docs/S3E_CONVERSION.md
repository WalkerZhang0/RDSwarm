# S3E Library_1 转 ROS1 bag

日期：2026-10-24。这里只转换数据，不启动 VIO、模型推理、回环或后端。

## 原始输入

下载目录：/media/walker/My PSSD/S3E。
应同时包含 S3E_Library_1.db3 和 metadata.yaml。
本机原始 db3 为 17,527,721,984 B，元数据记录约 454.36 秒。

默认按此前双机选择：Alpha＋Carol，每机仅左目与 IMU。
不复制右目、LiDAR、GPS；需要其他机器时显式修改 --agents。

## 推荐转换命令

在 RDSwarm 根目录运行：

~~~bash
source /opt/ros/noetic/setup.bash
python3 scripts/convert_s3e_ros1.py \
  --source "/media/walker/My PSSD/S3E" \
  --output "$PWD/results/s3e_library_1_ros1" \
  --agents Alpha Carol \
  --image-mode mono8 \
  --compression lz4
~~~

转换器读取 ROS2 CDR，使用 ROS1 Noetic 原生 writer。
当前旧 rosbags 0.9.23 的 LZ4 writer 在试转换中无法被原生 rosbag 解压；
因此未采用它写最终文件。失败试输出已标为 invalid，不能用来运行。

输出：
- alpha_mono8.bag
- carol_mono8.bag
- conversion.json：源/目标话题、数量、时间范围、重复/回退时间统计和可读性核验。

默认不覆盖已有输出。中途失败的文件保留 .partial；
成功关闭并完整通过 ROS1 读取后才改名为 .bag。ssh

## 本机已完成结果

本轮实际全量输出位于 RDSwarm/results/s3e_library_1_ros1/：

| 文件 | 图像数 | IMU 数 | 文件字节 | 时间跨度 |
|---|---|---|---|---|
| alpha_mono8.bag | 4,446 | 44,501 | 4,658,700,550 | 445.12 秒 |
| carol_mono8.bag | 4,544 | 45,415 | 4,477,223,297 | 454.35 秒 |

共 98,906 条消息，原生 ROS1 全量读取通过；源四个话题均无重复或回退采样时间。
原记录时间与 Header 时间在该输入中一致，转换逐个保留。
全量转换及读取校验约 147.4 秒；这是本机一次数据准备耗时，不是算法实时性能。

## 话题与像素处理

| 源 ROS2 话题 | 输出 ROS1 话题 | 类型 |
|---|---|---|
| /Alpha/left_camera/compressed | /Alpha/left_camera/image_raw | sensor_msgs/Image |
| /Alpha/imu/data | /Alpha/imu/data | sensor_msgs/Imu |
| /Carol/left_camera/compressed | /Carol/left_camera/image_raw | sensor_msgs/Image |
| /Carol/imu/data | /Carol/imu/data | sensor_msgs/Imu |

图像 JPEG 解码为 1224×1024 mono8，不缩放，不修改像素坐标。
这是灰度输入转换，不是保留彩色图像；后续若需要彩色应另选转换策略。
LZ4 是 bag 存储压缩，读出后仍是原生 Image，不是 CompressedImage。

保留原记录时间、Header 采样时间和 frame_id，使用整数纳秒，不将每机分别归零。
ROS2 Header 没有 seq；新增 ROS1 seq=0。
IMU 数值、单位与 covariance 原样保留，不做重采样或坐标变换。
统一时基不意味着两台设备同时开始录制，原始起止差异保留。

## 只验证短片段

换一个独立输出目录：

~~~bash
python3 scripts/convert_s3e_ros1.py \
  --source "/media/walker/My PSSD/S3E" \
  --output "$PWD/results/s3e_library_1_ros1_smoke_native" \
  --agents Alpha Carol --image-mode mono8 --compression lz4 \
  --start-offset 10 --duration 2
~~~

offset/duration 只筛选记录区间，不改变输出时间戳。
该 2 秒窗口已经逐消息与原始 CDR 比较：40 张图像像素及 400 条 IMU，
记录/采样时间、frame_id、数值和协方差一致。

如果只需要格式转换而不解压 JPEG：

~~~bash
python3 scripts/convert_s3e_ros1.py \
  --source "/media/walker/My PSSD/S3E" \
  --output "$PWD/results/s3e_library_1_ros1_compressed" \
  --agents Alpha Carol --image-mode compressed --compression lz4
~~~

这种模式保留 /left_camera/compressed，不能直接传给当前 VINS 的 Image 订阅。

## 检查与播放

~~~bash
rosbag info results/s3e_library_1_ros1/alpha_mono8.bag
rosbag info results/s3e_library_1_ros1/carol_mono8.bag
~~~

同一 ROS master 下回放双机时，将两个包交给同一个播放器合并时间线：

~~~bash
rosparam set /use_sim_time true
rosbag play --clock --pause \
  results/s3e_library_1_ros1/alpha_mono8.bag \
  results/s3e_library_1_ros1/carol_mono8.bag
~~~

不要分别归零，也不要让两个播放器同时给同一 master 发布 /clock。
按空格开始播放前，应先启动需要的订阅节点。

## 当前仍缺的输入

本机下载目录尚未包含官方 Calibration/alpha.yaml、Calibration/carol.yaml。
转换不依赖标定，但运行 VINS 必须用它们生成真实单目配置，不能用 EuRoC 参数。
真值 txt 保持原文件，尚未转换成在线观测，也未因此验证真值姿态/坐标定义。

数据输出放在 results/，被 Git 忽略；不会上传 db3 或生成的 bag。
转换器可由小型合成 ROS2 数据测试，不需下载整个数据集：

~~~bash
python3 tests/test_s3e_conversion.py
~~~
