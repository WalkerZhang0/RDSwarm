# 第一阶段验证记录

日期：2026-10-23。

## 构建环境

- ROS Noetic；CMake 3.16.3；C++17；catkin_make -j2 -l2。
- 前端：Ceres 1.14.0；系统 OpenCV 4.2.0，与系统 cv_bridge 一致。
- GPU 构建依赖：TensorRT 10.8、CUDA 12.8。编译/链接成功不等于执行过推理。
- 本机 GTSAM config.h 标记 4.3a1，CMake package 标记 4.3.0。
  本轮如实记录为开发版接口构建验证；未下载、安装或替换全局 GTSAM。
  正式稳定发布依赖仍需另外固定，不能把本轮结果称为稳定版部署验证。

## 已通过

1. 7 个 catkin 包完成配置及编译。
2. 9 个自定义消息、2 个服务的代码生成。
3. 两套 VINS 和回环输入缓存以 agent0/agent1 namespace 启动，
   全部四节点注册并正常响应 SIGINT 退出；测试不输入图像/IMU，不运行 GPU 推理。
4. 真实原生导出函数的合成夹具：
   - UTC 纳秒整数 1760000000123456789 保持不变。
   - feature_id、二维观测及当前相机系三维坐标对应正确。
   - 未知深度方差、局部描述子明确保持无效。
   - GetKeyframe 实际请求响应成功，并按请求移除图像。
   - VIO reset 后 epoch 改变，重复采样时间不会与原 session 冲突。
5. 后端输入库拒绝未知、非正定、非对称、非有限的协方差及相同端点。
6. 原工作副本导入文件的 82 项源 SHA256 校验，确认原工程未被修改。
7. package.xml 和 launch XML 语法检查。

## 重现

在仓库根目录：

~~~bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 scripts/smoke_interfaces.py
~~~

同时检查双机启动/退出：

~~~bash
python3 scripts/smoke_interfaces.py --engine /absolute/path/to/superpoint.engine
~~~

默认只使用 localhost:11531 的独立 ROS master；只清理本脚本启动的子进程。
若该端口占用，可以设置 --port。输出为 logs/interface_smoke/summary.json，
日志及结果文件不纳入 Git。受限沙箱会禁止本地 socket，需要在允许本地 ROS 的环境运行。

本轮最终 summary.json 中 passed=true，backend_contract 和 frontend_export
returncode 均为 0，dual_agent_startup_shutdown returncode 为 0。

## 尚未验证

- 真实单目数据上的 VIO 初始化、轨迹精度和长期运行。
- 本轮复制后的 SuperPoint engine 推理输出、局部/全局描述子。
- 实际跨机回环发现、匹配、PnP、空间初始化。
- iSAM2 因子图更新、相对协方差查询服务和调度算法。
- GPU 时间、实时性、真实网络带宽及 Jetson 部署。

这里记录的是第一阶段框架与接口验证，不是完整集群 SLAM 已运行。
