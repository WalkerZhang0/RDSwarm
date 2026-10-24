# 2026-10-24_Convert_S3E_ROS1

## 1. 修改文件清单

- scripts/convert_s3e_ros1.py：选择机器/话题、CDR 读取、JPEG 灰度解码、原生 ROS1 写入及完整可读性校验。
- tests/test_s3e_conversion.py：合成 ROS2 输入，核验原生 LZ4、整数时间、IMU、图像和拒绝覆盖。
- docs/S3E_CONVERSION.md：分步转换、检查、播放与标定缺项说明。
- README.md：增加数据准备入口。
- results/s3e_library_1_ros1/：生成 Alpha＋Carol 两机 bag 和报告；该目录不进入版本库。

## 2. 功能实现说明

保留原始 db3、metadata 和真值文件，按默认双机选择只导出左目和 IMU。
图像转换为原始大小 mono8，供当前 VINS 的 sensor_msgs/Image 输入使用。
IMU 字段不改值、不换坐标；Header 与记录时间各自按整数纳秒保留。
也提供仅保留压缩图像的格式转换选项，命令中明确选择。

输出采用 .partial 防止误用失败结果，文件关闭且完整通过原生 ROS1 读取后才作为最终 bag。
每个输入消息对应一个输出消息；报告保留数量和时间统计。

## 3. 问题与解决办法

- 数据 ROS2 Header 与 ROS1 Header 不同：显式补 seq=0，保留原 stamp 和 frame_id。
- 旧 rosbags LZ4 写出试包无法被 Noetic 原生 rosbag 解压。
  2 秒试验证及时发现，改用 Noetic 原生 writer；失败试文件隔离标记 invalid。
- CompressedImage 不是当前 VINS 接收的 Image。
  在转换时 JPEG 解码为 mono8，避免再次复制完整多传感器原包。
- 不同设备起始时间不同。
  保留共同绝对时间，不分别归零，播放说明使用一个播放器合并时间线。
- 检查试窗口所有 440 条消息，包括 40 张图像和 400 条 IMU，与源数据逐项比较通过。
- 全量 Alpha/Carol 共 98,906 条消息完成转换，输出原生 ROS1 完整读取通过。
- 合成 ROS2 回归覆盖 mono8/压缩图像两种模式、原生 LZ4、记录/采样时间不同及拒绝覆盖，测试通过。

## 4. 注意事项 (Precautions)

- 本工具只准备数据，不表示 VIO、回环检索或后端已在 Library_1 验证。
- 当前目录缺少 Alpha/Carol 官方标定，VINS 启动仍需补齐配置。
- 转换不包含右目、LiDAR、GPS，不改写真值。
- .bag、.partial、实际数据与报告均放 results/，不提交 Git。
- 运行转换需要现有 ROS1 Noetic Python 包、rosbags、OpenCV 和 NumPy，不需要 ROS2 节点。
- 仅格式转换模式仍为 CompressedImage，不能冒充原生 Image。
- 原生读取验证是兼容性和格式检查，不替代相机/IMU 的标定与时间误差验证。
