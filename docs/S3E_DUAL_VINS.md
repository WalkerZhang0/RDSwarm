# S3E Alpha/Carol 双路 VINS 测试

本轮只运行两个独立的单目＋IMU VINS 和各自的关键帧缓存。bag 播放由你单独控制；SuperPoint、PnP 和后端不随 launch 自动启动。

后续 Alpha 单路全段出现发散；此前短片段能启动的记录不能代表整段估计稳定。定位结果和单参数诊断配置见 [Alpha 漂移诊断](alpha_VIO轨迹漂移研究.md)。

## 先单独跑 Alpha 并打开 RViz

新增内容都是 launch/显示配置，无须重新编译已有 `devel_cpu`。每个终端先加载同一个工作区并连接同一个 master。以下按 zsh 给出；Bash 用户将两处 `setup.zsh` 换成 `setup.bash`。

```bash
cd "/media/walker/My PSSD/tmp/论文/RDSwarm"
source /opt/ros/noetic/setup.zsh
source devel_cpu/setup.zsh
export ROS_MASTER_URI=http://127.0.0.1:11311
```

终端 1：启动 master。如果该端口已有 master，直接复用。

```bash
roscore -p 11311
```

终端 2：启用仿真时钟，再启动 Alpha。

```bash
rosparam set /use_sim_time true
roslaunch rdswarm_bringup s3e_alpha.launch
```

终端 3：仅启动 Alpha 的 RViz。

```bash
roslaunch rdswarm_bringup s3e_rviz.launch agent_id:=0
```

终端 4：回放 Alpha 单个 bag，按空格开始/暂停。

```bash
rosbag play --clock --pause -r 0.5 \
  "$PWD/results/s3e_library_1_ros1/alpha_mono8.bag"
```

RViz 的 Fixed Frame 已设为 `world`，包含绿色轨迹、相机位姿、稀疏地图点和原始左目图像。图像在回放开始后更新，轨迹需等待 VINS 输出 `Initialization finish!`。绿色轨迹只表示当前 VIO，没有回环校正。

如需显示跟踪点，在 `src/rdswarm_bringup/config/s3e/alpha_vins.yaml` 中设置 `show_track: 1`，重启 VINS，并勾选 RViz 的 `Tracked image (optional)`。仅打开 RViz 的图像显示不能启用 VINS 内部跟踪图发布。

先测试 Carol 时，VINS 入口改成 `s3e_carol.launch`，RViz 参数改成 `agent_id:=1`，bag 改成 `carol_mono8.bag`。

## 加入第二路及 RViz 的坐标含义

要测双路，在开始回放前另开终端分别启动 Carol 和它的 RViz：

```bash
roslaunch rdswarm_bringup s3e_carol.launch
```

```bash
roslaunch rdswarm_bringup s3e_rviz.launch agent_id:=1
```

然后采用下文的一条双 bag 播放命令，替换单 bag 播放命令。两个 RViz 窗口分别观察本机 VIO。各自的 `world` 是独立初始化的本地坐标系，尚未建立两机坐标对齐，不能直接比较两窗口内的位置得到机间相对位姿。

原版 VINS 的 TF 帧名固定为 `world/body/camera`。`mono_agent.launch` 现在将广播分别 remap 到 `/agent0/tf`、`/agent1/tf`，RViz 入口选择对应的 TF，避免同 master 下互相覆盖。外部 TF 工具同样需要 remap，例如 `rosrun tf tf_echo world body /tf:=/agent0/tf`。这项修改需要重启先前运行的 VINS 实例才能生效。

显示配置文件是 `src/rdswarm_bringup/config/s3e_vins.rviz`；话题映射在 `src/rdswarm_bringup/launch/s3e_rviz.launch`，可以直接查看和修改。

## 1. 配置与输出

| 设备 | ROS 命名空间 | VINS 主配置 | 左目相机模型 | 结果 |
| --- | --- | --- | --- | --- |
| Alpha | `/agent0` | `src/rdswarm_bringup/config/s3e/alpha_vins.yaml` | `src/rdswarm_bringup/config/s3e/alpha_cam0.yaml` | `results/vio_alpha/vio.csv` |
| Carol | `/agent1` | `src/rdswarm_bringup/config/s3e/carol_vins.yaml` | `src/rdswarm_bringup/config/s3e/carol_cam0.yaml` | `results/vio_carol/vio.csv` |

`results/alpha.yaml`、`results/carol.yaml` 是原始 S3E 标定，不是可直接传给 VINS 的配置。派生配置使用左目原始内参和畸变，未缩放、未双目校正。原始 `Tic` 的注释定义为相机到 IMU，因此直接用作 `body_T_cam0`。

两路首先固定为 `use_superpoint:=0`，检验单目 VIO 基线；后续回环仍可在既有 VIO 特征处提取 SuperPoint 描述子。IMU 噪声暂按原始标定同名字段映射，尚未进行统计一致性验证。`td: 0.0`、`estimate_td: 0` 是当前起始设置；共享 UTC 不足以单独证明相机与 IMU 内部没有偏移。

## 2. 分终端运行

下面使用 Bash。每个终端先运行：

```bash
cd "/media/walker/My PSSD/tmp/论文/RDSwarm"
source /opt/ros/noetic/setup.bash
source devel_cpu/setup.bash
```

如果继续使用 zsh，两个 `setup.bash` 都替换成 `setup.zsh`。不要在 zsh 中 source Bash 的 setup 文件。

终端 A：同一个 ROS master。

```bash
roscore
```

终端 B：在启动 VINS 前启用 bag 仿真时钟，然后启动 Alpha。

```bash
rosparam set /use_sim_time true
roslaunch rdswarm_bringup s3e_alpha.launch
```

终端 C：启动 Carol。

```bash
roslaunch rdswarm_bringup s3e_carol.launch
```

两个入口默认启动各自的 `vins` 和 `loop_inputs`。后者缓存真实关键帧，提供 `/agentX/get_keyframe`，方便本轮验证传输。也可用下面一条 launch 同时启动它们；不要与上面两个入口同时运行：

```bash
roslaunch rdswarm_bringup s3e_dual.launch
```

可调参数示例：

```bash
roslaunch rdswarm_bringup s3e_alpha.launch \
  session_id:=library_test02 \
  output_path:="$PWD/results/library_test02/alpha"
```

每次启动会新建或清空对应 `vio.csv`；保留不同实验时更换 `output_path`。重放同一段数据时重启两路 VINS 及缓存，使用新的 `session_id`，避免混用旧帧。

终端 D：**用一个播放器同时播放两个 bag**，保持它们原始 UTC 时间关系。先暂停启动，准备好监测后按空格继续；首次双路测试用半速降低机器负载。

```bash
rosbag play --clock --pause -r 0.5 \
  "$PWD/results/s3e_library_1_ros1/alpha_mono8.bag" \
  "$PWD/results/s3e_library_1_ros1/carol_mono8.bag"
```

不要分别启动两个发布 `/clock` 的播放器。Carol 在记录中比 Alpha 早约 5.2 秒，这是原始数据的时间差。

## 3. 检查 VIO 数据链路

节点连接关系：

```bash
rosnode info /agent0/vins
rosnode info /agent1/vins
rostopic info /agent0/vins/keyframes
rostopic info /agent1/vins/keyframes
```

`/agent0/vins` 应订阅 `/Alpha/left_camera/image_raw`、`/Alpha/imu/data`，`/agent1/vins` 应订阅 Carol 对应话题。这些是实际输入话题，remap 不会再生成 `/agentX/vins/cam0/image_raw` 副本。

两个 `keyframes` 话题各自的发布者应是 `/agentX/vins`，订阅者应包含 `/agentX/loop_inputs`。原版全局重启、传感器切换和外部特征输入已隔离到各自 `/agentX/vins/` 下。

在其他终端按需观察，`Ctrl+C` 退出单项监测：

```bash
rostopic hz /Alpha/left_camera/image_raw
rostopic hz /Alpha/imu/data
rostopic hz /Carol/left_camera/image_raw
rostopic hz /Carol/imu/data
rostopic hz /agent0/vins/odometry
rostopic hz /agent1/vins/odometry
```

原始图像约 10 Hz、IMU 约 100 Hz。半速时墙钟接收频率约减半；评估采样频率时使用消息 Header 时间。VINS 需要运动、视差和 IMU 激励完成初始化，起播后不会立即出现里程计。

初始化后检查状态身份和关键帧：

```bash
rostopic echo -n 1 /agent0/vins/states/initialized
rostopic echo -n 1 /agent1/vins/states/initialized
rostopic echo -n 1 /agent0/vins/keyframes/key
rostopic echo -n 1 /agent1/vins/keyframes/key
```

通过条件：两路均持续发布有限的位姿、`initialized: true`，且产生对应 `agent_id` 的关键帧。这是运行和数据链路验证；轨迹精度还需另外与 GT 对齐评估。

## 4. 用真实响应验证通信

**两个 VINS 不会直接互发数据。** 当前第一条可验证的应用数据链是：

```text
Alpha 图像/IMU → /agent0/vins → /agent0/loop_inputs → /agent0/get_keyframe
Carol 图像/IMU → /agent1/vins → /agent1/loop_inputs → /agent1/get_keyframe
```

`rosnode list`、`rosservice list` 只能证明已注册。取回真实关键帧才能证明这条数据路径有效。

先在回放时用上一节命令记录 Carol 的 `agent_id/session_id/frame_id`，可以暂停 bag，再查看服务并请求该帧。下面把 `CAROL_FRAME_ID` 改成刚看到的整数，`session_id` 同样以消息为准：

```bash
rosservice info /agent1/get_keyframe
CAROL_FRAME_ID=替换为实际帧号
rosservice call /agent1/get_keyframe \
  "{key: {agent_id: 1, session_id: s3e_library_1.e0, frame_id: $CAROL_FRAME_ID}, include_image: false, include_descriptors: false}"
```

预期 `success: true`、`status: ok`，返回的帧键与请求一致，且包含真实特征。Alpha 可按同样方式请求。返回 `not_found` 只表明所请求的帧没有缓存；服务能应答不等于已取到有效数据。

这一步证明同 master 下客户端到关键帧服务的数据传输。下一阶段启动 SuperPoint 和 loop core 后，再由 `/agent0/fetch_peer_features` 请求 `/agent1/get_local_features`，验证回环模块实际的跨 agent 特征传输；随后才调用 PnP。不能跳过描述子提取直接用 `get_local_features` 验证特征载荷。

两个独立 `roscore` 之间的跨 master/WiFi 桥仍未实现。同机同 master 测试不代表真实无线链路、通信带宽或两机坐标已经对齐。

## 5. 本轮验证记录

已通过实际双路 launch，在独立本机 master 上回放起点后的 35 秒、倍率 0.5：

| 设备 | 图像 | IMU | 里程计/状态 | 关键帧 | 服务返回帧的特征数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Alpha | 292 | 2969 | 125 | 35 | 74 |
| Carol | 350 | 3491 | 124 | 54 | 93 |

两路均完成初始化，各消息流时间戳无回退；两个缓存请求返回 `success=true/status=ok`。另从 `/agent0/peer_link_probe` 成功请求 `/agent1/get_keyframe`，返回真实 Carol 帧。临时测试节点及 master 已停止。

完整记录在 `results/s3e_dual_smoke/validation.json`，轨迹为该目录 `alpha/vio.csv` 和 `carol/vio.csv`。这是一段运行验证，尚未与 GT 评估精度。本次变更记录见 [2026-10-25_Add_S3E_Dual_VINS_Launch.md](../2026-10-25_Add_S3E_Dual_VINS_Launch.md)。

2026-10-29 部署更新：启动默认标定与 VINS 配置位于包内 `config/s3e/`，可通过 `config_dir` 或 `config_file` 覆盖。Alpha 的 `acc_n=0.1` 保留当前诊断实验设置，不能解释为已校准噪声；历史 40 秒记录的实际配置应单独存档。新机器步骤见 [DEPLOYMENT.md](DEPLOYMENT.md)。
