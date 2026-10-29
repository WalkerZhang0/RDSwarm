# 2026-10-25_Add_Manual_RViz

## 1. 修改文件清单

- `src/rdswarm_bringup/launch/s3e_rviz.launch`：新增可独立启动的 Alpha/Carol RViz 入口，选择对应话题与 TF。
- `src/rdswarm_bringup/config/s3e_vins.rviz`：新增轨迹、相机标记、稀疏地图点、左目图像及可选跟踪图显示配置。
- `src/rdswarm_bringup/launch/mono_agent.launch`：按设备隔离 TF 话题。
- `src/rdswarm_bringup/package.xml`：声明 RViz 运行依赖。
- `docs/S3E_DUAL_VINS.md`：补充分终端单路运行、双路扩展和 RViz 操作说明。
- `README.md`：添加运行说明入口。

## 2. 功能实现说明

用户分别启动 master、VINS、RViz 和 bag 播放器。`s3e_rviz.launch agent_id:=0` 默认观察 Alpha，`agent_id:=1` 默认观察 Carol；两个实例可以同时运行。显示配置的占位话题通过 ROS remap 显式映射到实际话题，图像话题和配置文件均支持参数覆盖。

`world` 是 VINS 可视化消息的现有帧名。每台 TF 广播及相应 RViz 监听使用独立命名空间，保留现有估计状态与对外原生关键帧格式。

## 3. 问题与解决办法

原版 VINS 向同一个 `/tf` 发布同名 `body/camera` 帧，双路同时运行会覆盖对应关系。本次 remap `/tf` 与 `/tf_static` 到各 agent 的话题，并在 RViz 入口使用相同映射。

原始运行配置的 `show_track` 为 0，不能把没有跟踪图误判为 RViz 故障。默认显示原图，可选跟踪图需要显式改配置并重启 VINS。

已验证：RViz 配置 YAML 和包清单 XML 可解析，显示器及工具类型均存在于本机 RViz 插件目录。Alpha/Carol 的 launch 展开后，轨迹、点云、图像和 TF 分别指向对应 agent；自定义 agent ID、命名空间及图像话题覆盖也通过检查。双路 VINS launch 展开后的 TF 广播分别隔离到 `/agent0/tf` 和 `/agent1/tf`，`git diff --check` 通过。

当前只完成显示配置与话题映射检查，未实际打开 RViz 图形窗口；用户按文档手动启动并检查渲染。

## 4. 注意事项 (Precautions)

- 两路 `world` 分别属于各自 VIO 初始化坐标系；本次没有完成两机坐标对齐。
- 先启用 `/use_sim_time` 再启动 VINS 和 RViz；双 bag 使用一个播放器发布时钟。
- 本轮为 launch/配置修改，无须重新编译 VINS。安装部署仍需重新安装包资源。
- 自定义 agent ID 时需显式指定 `image_topic`，Alpha/Carol 默认映射仅覆盖 ID 0/1。
- 外部 TF 消费者必须选择对应的 `/agentX/tf` 和 `/agentX/tf_static`；之前运行的节点需重启应用新 remap。
