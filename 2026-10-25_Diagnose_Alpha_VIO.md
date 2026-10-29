# 2026-10-25_Diagnose_Alpha_VIO

## 1. 修改文件清单

- `src/vins_fusion/vins_estimator/src/estimator/parameters.h`、`parameters.cpp`：新增 `frontend_frame_stride` 参数；诊断开关和输出路径支持私有 ROS 参数覆盖。
- `src/vins_fusion/vins_estimator/src/estimator/estimator.h`、`estimator.cpp`：显式控制估计帧抽样；读取输入缓冲时加锁；等待日志节流；记录速度、IMU 偏置、时间偏移、等待时长及积分步长。
- `src/vins_fusion/vins_estimator/src/featureTracker/feature_tracker.cpp`：诊断时间戳使用足够精度，避免 Unix 时间只剩六位有效数字。
- `results/alpha_vins_debug.yaml`：新增单参数噪声诊断配置，原标定和原基线文件保留。
- `.gitignore`：允许诊断配置参与版本管理，数据和测试结果继续忽略。
- `docs/ALPHA_VIO_DIAGNOSIS.md`：记录对照结论、边界和手动复现实验方法。
- `docs/S3E_DUAL_VINS.md`：添加漂移诊断入口。

## 2. 功能实现说明

确认问题发生在 `s3e_alpha.launch` 的 Alpha 单路。新增诊断保持原始标定、单目设置、噪声和求解选项不变，通过 `frontend_frame_stride=2/1` 比较原有 5 Hz 和完整 10 Hz 视觉估计；跟踪始终消费全部输入图像。

默认抽样策略与原版保持一致：多线程未指定参数时为 2，单线程为 1。两种模式都允许显式设置正整数步长。

等待日志由每 5 ms 的裸打印改为节流 ROS 提示，包含图像时间、IMU 覆盖差和墙钟等待时长。现有 CSV 增加 IMU/估计器诊断列；时间使用 17 位有效数字。

## 3. 问题与解决办法

原有 `freq: 10` 未控制 `inputImage` 中写死的隔帧处理。将该行为改为显式参数后，同数据前 90 秒对照显示：完整 10 Hz 仍发散且更早，因此不能把抽帧认定为主要原因。

原参数的 5 Hz 对照累计路程约 1655.6 m、最大速度约 70.9 m/s；10 Hz 对照约 17225.6 m、最大速度约 242.5 m/s。两者非线性阶段均没有累计 IMU 等待，最大积分步长约 15.3 ms，未出现与发散相伴的长时间 IMU 断流。

随后仅将 `acc_n` 从约 0.01092 改为 0.1，检验过强的加速度约束是否参与了发散。同一前 90 秒、同一 5 Hz 策略下，最大速度降为约 1.69 m/s、累计路程降为约 128.1 m，说明改善不能归因于提高视觉更新频率。

完整段二倍回放通过：4446 张图像全部跟踪、2223 帧进入估计器；初始化后 2202 条里程计覆盖约 440.2 秒，最大速度约 1.80 m/s、末帧加速度偏置范数约 0.041 m/s²，未复现原来的爆炸式发散。只初始化一次，轨迹有效字段均有限。全段最长逐帧 IMU 等待约 5.1 ms，99% 分位数为 0。该值是诊断用权重，不是已经验证的物理噪声标定；未完成 GT ATE/RPE 评测。

CPU 构建已通过，实际 replay 同时验证了步长 1/2、诊断 ROS 参数覆盖及 CSV 字段输出。比较汇总写入 `results/alpha_drift_audit/comparison.json`。测试节点和独立 master 已退出。

编译时遇到 ROS 日志宏将模板中的逗号识别为额外参数，改为先计算时长变量后传给宏。VINS 原有 CSV 末尾包含空字段，数值核验只读取 11 个真实数据列，未把末尾空列当作估计器 NaN。

## 4. 注意事项 (Precautions)

- 不据此宣称 UTC 时间一致就等于相机与 IMU 已物理同步。
- 基线 `results/alpha_vins.yaml` 和用户已有 EuRoC 配置修改不覆盖。
- 现有 `failureDetection()` 开头直接返回 false；本次对照保留该行为，避免自动重启掩盖不同配置的发散。
- 诊断日志列有新增，依赖固定列位置的解析程序需更新；按字段名读取不受影响。
- `frontend_frame_stride` 在启动时读取；每轮实验重启 VINS，并更换 `session_id/output_path`。
- 参数移植需要核对噪声定义和预积分离散化。ORB-SLAM3 的同名字段会按 IMU 频率转换，不能只凭字段名就认定与 VINS 语义相同。
- 对照在独立本机 ROS master 的 11476 端口执行，未向真实设备发送消息。
