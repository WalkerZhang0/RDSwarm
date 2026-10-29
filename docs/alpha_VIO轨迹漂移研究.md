# Alpha 单路 VIO 漂移诊断

问题来自 `s3e_alpha.launch` 单路运行，日志已确认实际加载 `results/alpha_vins.yaml`。原始轨迹文件中的位移也在发散，因此不是 RViz 或两机 TF 混用造成的显示问题。

## 已完成的前 90 秒对照

使用相同 Alpha 图像、IMU、相机标定、时间偏移和求解选项，仅改变视觉估计抽样步长。两个估计器同时订阅同一个播放器，rate 为 1.0；均为 Shi-Tomasi＋KLT。

| 设置 | 里程计输出数 | 累计路程 | 最大速度 | 最大加速度偏置范数 |
| --- | ---: | ---: | ---: | ---: |
| 原始参数，步长 2，约 5 Hz | 426 | 1655.6 m | 70.9 m/s | 1.52 m/s² |
| 原始参数，步长 1，约 10 Hz | 853 | 17225.6 m | 242.5 m/s | 4.81 m/s² |
| 仅改 `acc_n=0.1`，步长 2 | 426 | 128.1 m | 1.69 m/s | 1.09 m/s² |

这段测试内，非线性阶段 `imu_wait_ms` 均为 0，最大 IMU 积分步长约 15.3 ms。10 Hz 没有解决发散，不能认定原有 5 Hz 抽帧是主因。

输出数包含初始化时刻差异，不能直接拿两组首末位移做精度比较。以上速度、累计路程用于识别失稳，不是 ATE/RPE 评测。

## 为什么会打印等待 IMU

估计器等待 `最新 IMU 时间 >= 图像时间 + td`。图像先于下一条 IMU 到达、bag 暂停或播放结束且 IMU 覆盖不足，都可能进入等待。原代码每 5 ms 打印一次，刷屏不等于长期断流。

当前日志改为节流输出 `image_t/lag_ms/wait_ms`；CSV 中记录每帧的实际等待和最大积分步长。Alpha 原数据约 100 Hz，唯一约 120 ms 的 IMU 间隙出现在首图之后约 217.25 秒，晚于原参数开始明显发散的时刻；不能用这个间隙解释初始发散。

## 单独调整加速度权重

保持步长 2，只把 `acc_n` 从原来的 `0.0109196...` 改为 `0.1`。它降低加速度预积分误差的权重；其余三个噪声字段、标定和时间偏移均保持原值。

这是定位权重敏感性的实验，不是把 `0.1` 当作重新标定结果。原始 YAML 使用 ORB 风格字段；本机 ORB-SLAM3 对这些字段还会按 `sqrt(IMU.Frequency)` 做转换，当前 VINS 实现则直接将配置值平方后参与离散预积分。两者同名字段不能自动等同。

完整 Alpha bag 也已验证：二倍回放全部约 445 秒数据，4446 张图像全部参与跟踪，2223 帧进入估计器；初始化后输出 2202 条里程计，覆盖约 440.2 秒，累计路程约 614.6 m。最大速度约 1.80 m/s，末帧加速度偏置范数约 0.041 m/s²，没有复现原先速度爆炸。全段仅初始化一次，轨迹数值有限，临时测试节点和 master 已停止。

完整段最大逐帧 IMU 等待约 5.1 ms，99% 分位数为 0；日志同时记录了包含原始 IMU 间隙的积分步长。以上并不证明时间同步、噪声统计或轨迹精度已经正确，尚未完成 GT 的 ATE/RPE 评估。

原始前 90 秒输出在 `results/alpha_drift_audit/stride2` 和 `stride1`，噪声短段对照在 `acc_noise_check`，完整段在 `acc_noise_full`。比较汇总见该目录的 `comparison.json`。数据支持原始加速度权重是关键诱因，但不能将经验权重改善直接当作正确标定的证明。

## 手动复现

代码已经重新构建到 `devel_cpu`。每个终端按实际 shell 加载 ROS 和本工作区，连接同一 ROS master；下例为 zsh：

```bash
cd "/media/walker/My PSSD/tmp/论文/RDSwarm"
source /opt/ros/noetic/setup.zsh
source devel_cpu/setup.zsh
export ROS_MASTER_URI=http://127.0.0.1:11311
```

终端 1：端口无 master 时启动。

```bash
roscore -p 11311
```

终端 2：启动诊断配置并保存到新目录。

```bash
rosparam set /use_sim_time true
rosparam set /agent0/vins/frontend_frame_stride 2
roslaunch rdswarm_bringup s3e_alpha.launch \
  config_file:="$PWD/results/alpha_vins_debug.yaml" \
  session_id:=alpha_debug01 \
  output_path:="$PWD/results/alpha_debug01"
```

终端 3：可单独启动 RViz。

```bash
roslaunch rdswarm_bringup s3e_rviz.launch agent_id:=0
```

终端 4：仍手动控制 bag，按空格开始/暂停。

```bash
rosbag play --clock --pause -r 0.5 \
  "$PWD/results/s3e_library_1_ros1/alpha_mono8.bag"
```

输出包括 `vio.csv`、`feature_tracker_diagnostics.csv` 和 `feature_tracker_diagnostics_estimator.csv`。后者按字段名读取，新增 `velocity_norm/acc_bias_norm/gyro_bias_norm/td_s/imu_wait_ms/imu_max_step_ms`。

若需要单独复现抽帧对照，在启动前用私有参数覆盖，例如：

```bash
rosparam set /agent0/vins/frontend_frame_stride 1
```

该值优先于 YAML，下次测试恢复步长 2 时要显式改回 2，或删除该私有参数。每轮重新启动 VINS 和缓存，使用独立输出目录，不在同一估计器内倒放时间。

`failureDetection()` 的原始禁用行为本次保留；能持续输出并不证明估计可靠。正式开展协同回环实验前仍需用 GT 检查轨迹误差，并核实噪声/时间偏移的物理定义。
