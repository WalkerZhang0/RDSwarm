# 2026-10-25_Add_S3E_Dual_VINS_Launch

## 1. 修改文件清单

- `.gitignore`：仅放行四份 S3E 派生 YAML，bag、轨迹和测试结果继续忽略。
- `src/rdswarm_bringup/launch/mono_agent.launch`：隔离原版全局重启、传感器切换及外部特征入口。
- `src/rdswarm_bringup/launch/s3e_alpha.launch`：新增 S3E Alpha 单目 VINS 启动入口，绑定 `alpha_vins.yaml`、Alpha 图像/IMU 话题和独立输出目录。
- `src/rdswarm_bringup/launch/s3e_carol.launch`：新增 S3E Carol 单目 VINS 启动入口，绑定 `carol_vins.yaml`、Carol 图像/IMU 话题和独立输出目录。
- `results/alpha_cam0.yaml`、`results/carol_cam0.yaml`：将 S3E 左目内参转换为 VINS-Fusion 的 PINHOLE 相机模型格式。
- `results/alpha_vins.yaml`、`results/carol_vins.yaml`：将 S3E 原始标定转换为 VINS-Fusion 单目＋IMU 配置格式。
- `src/rdswarm_bringup/launch/s3e_dual.launch`：新增同一 ROS master 下的双路 VINS 启动入口。
- `docs/S3E_DUAL_VINS.md`：记录分终端启动、bag 播放、VIO 输出检查和跨 agent ROS 服务检查方法。

## 2. 功能实现说明

新增的 launch 文件不自动播放 bag，允许分别启动、暂停和检查 Alpha/Carol 两路 VINS。默认输出写入 `results/vio_alpha` 和 `results/vio_carol`，并保留 SuperPoint 参数覆盖入口。

通信验证分为两层：先检查各路 VINS 是否收到输入并发布 `odometry/states/keyframes`，再在同一 ROS master 下检查 loop 节点的跨 agent topic/service 可见性。

VINS 配置使用 S3E 左目原始内参、畸变及相机到 IMU 的 `Tic`。IMU 噪声原值映射，单目配置没有使用双目基线或对端相机标定。四份派生配置仍存放在用户指定的 `results` 中，并允许进入版本管理。

## 3. 问题与解决办法

原始 `alpha.yaml`/`carol.yaml` 是 S3E 标定格式，不包含 VINS-Fusion 所需的 `imu`、`body_T_cam0`、`cam0_calib` 等字段，直接启动会在参数读取阶段失败。保留原始文件作为来源，新增两份相机模型文件和两份 VINS 适配配置。原有通用 launch 的默认话题是 EuRoC/虚拟 drone 话题，不能直接用于 S3E；本次通过两个专用 launch 显式映射 S3E 的 Alpha/Carol 话题，避免修改通用入口的默认行为。

两路 VINS 原版均订阅全局 `/vins_restart` 等控制话题，单次重启可能影响另一台。本次 remap 到各自 `/agentX/vins/restart`、`imu_switch`、`cam_switch` 和 `feature_input`；已核对运行时实际订阅图。

验证结果：

- 五个通用/S3E launch 均可解析；两套内参、畸变、外参和噪声数值与原始标定逐项相等。
- 通过实际 `s3e_dual.launch`，在独立本机 master 的 11465 端口回放全局起点后的 35 秒、播放倍率 0.5。
- Alpha：收到 292 图像、2969 IMU，发布 125 条里程计/状态和 35 个关键帧。
- Carol：收到 350 图像、3491 IMU，发布 124 条里程计/状态和 54 个关键帧。
- 各流 Header 时间无回退，两台均完成初始化；请求返回的关键帧 ID 与原始整数采样时间一致。
- 两边 `get_keyframe` 返回 `success=true/status=ok`，所取 Alpha/Carol 关键帧分别含 74/93 个特征。
- `/agent0/peer_link_probe` 请求 `/agent1/get_keyframe` 成功，返回 Carol 的真实帧；这只验证同 master 下客户端到缓存服务的数据路径。
- 轨迹和完整计数写入 `results/s3e_dual_smoke/`。临时启动的测试节点和 master 已退出。

## 4. 注意事项 (Precautions)

- 当前两个 VINS 只通过 ROS 话题发布本机状态，尚不存在 VINS 到 VINS 的直接通信接口。
- 首轮链路测试应使用同一个 `roscore`；两个独立 ROS master 之间的 WiFi/跨 master 通信桥尚未实现。回环模块的对端描述子请求仍需在 SuperPoint 阶段验证。
- `rosbag play` 保持在 launch 外部手动执行；两个 bag 可用一条命令同时播放，避免自动流程掩盖时间和话题问题。
- launch 默认从源码包定位仓库 `results`；安装到其他目录后需覆盖 `results_dir` 或 `config_file/output_path`。配置中的原始输出路径也可通过 `output_path` 覆盖。
- 首轮固定 Shi-Tomasi＋KLT、`td=0`、原始 IMU 噪声值；本次没有验证轨迹 GT 精度或噪声统计一致性，不据此推断系统漂移界限。
- Shell 环境按实际 shell 使用 `setup.bash` 或 `setup.zsh`；仿真时钟需在启动节点前开启，两个 bag 用一个播放器发布 `/clock`。
