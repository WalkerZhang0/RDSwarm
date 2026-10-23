# 第一阶段范围与验收

日期：2026-10-23。

## 已冻结的实现边界

- 新仓库独立开发，不改 /home/walker/swarm/fusion_ws 和 /home/walker/swarm/full_ws。
- 导入用户 SuperPoint 版 VINS-Fusion 的 estimator 和 camera_models；不导入旧 loop_fusion/global_fusion。
- 单目＋IMU，SuperPoint 继续使用 GPU；后端和调度的接口面向 CPU。
- Pose3 后端数据类型，后续加入真实 iSAM2 更新；本阶段只交付输入、校验、查询协议和抽象接口。
- 启动文件隔离双机 namespace，两个节点的数据 ID、session、局部 VIO 参考系均独立。
- 先回环，后共视重投影；暂不启用 UWB、时间内插、ADMM、删帧或反馈 VIO。

## 实际可运行模块

1. vins/vins_node：原有单机 VIO，并新增私有话题 keyframes、states。
2. rdswarm_loop/rdswarm_loop_input_node：输入校验、缓存和 get_keyframe 服务。
3. rdswarm_backend_contract：Pose3 约束输入及六维噪声校验库，无优化节点。
4. rdswarm_msgs：9 个消息、2 个服务，供后端和回环后续接入。

后端服务 QueryRelativeState 目前仅为协议。没有有效优化结果时不返回假位姿或零协方差。
未实现全局检索、SuperPoint 局部描述子导出、特征匹配及 PnP。

## 验证边界

编译覆盖导入 VINS、TensorRT/CUDA 链接、原生消息导出、缓存与后端输入库。
合成接口夹具将测试原始纳秒时间、feature ID、相机系三维点转换、请求响应和 reset epoch。
该夹具不执行真实 VIO 初始化，也不执行 GPU 网络推理，不代表定位精度和实时性能测试。

实际结果见 VALIDATION.md，构建与测试日志位于被 Git 忽略的 logs/。
