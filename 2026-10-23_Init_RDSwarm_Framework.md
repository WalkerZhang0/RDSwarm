# 2026-10-23_Init_RDSwarm_Framework

## 1. 修改文件清单

- src/vins_fusion/vins_estimator、camera_models：选择性导入已有工作副本源码，保留版权头及 LICENCE。
- src/vins_fusion/vins_estimator/CMakeLists.txt、package.xml：C++17、GPU 依赖查找、原生消息依赖、库/节点与测试目标。
- src/vins_fusion/vins_estimator/src/utility/rdswarm_export.h/.cpp：原始图像时间缓存、session epoch、Keyframe/VioState 导出。
- src/vins_fusion/vins_estimator/src/utility/visualization.cpp：保留旧输出，增加新消息发布钩子。
- src/vins_fusion/vins_estimator/src/rosNodeTest.cpp：输入图像缓存、同步线程退出及回收。
- src/vins_fusion/vins_estimator/src/estimator/estimator.h/.cpp：首次清理前指针初始化、reset 通知和停机线程退出。
- src/vins_fusion/vins_estimator/src/estimator/parameters.cpp：启动参数覆盖 engine 和输出路径、输出目录创建、缺失 engine 报错。
- src/vins_fusion/vins_estimator/src/featureTracker/feature_tracker.cpp：要求 SuperPoint 时禁止无记录回退至 Shi-Tomasi。
- src/vins_fusion/vins_estimator/test/export_contract_fixture.cpp：原生消息导出与服务的合成测试。
- src/rdswarm_msgs：9 个消息、2 个服务。
- src/rdswarm_core：与 ROS 无关的 FrameKey。
- src/rdswarm_backend：Pose3 后端抽象接口、约束校验库与测试。
- src/rdswarm_loop：回环抽象接口、输入校验、只增缓存和 GetKeyframe 服务。
- src/rdswarm_bringup：单机/双机 launch、EuRoC 单目示例和标定文件。
- scripts/smoke_interfaces.py：隔离 ROS master、接口及启动/退出验证。
- README.md、docs/*、LICENSE、.gitignore：使用说明、来源、设计快照、第一阶段范围和验证记录。

## 2. 功能实现说明

建立独立本地 Git 仓库，origin 指向用户提供的 WalkerZhang0/RDSwarm。
未修改 fusion_ws 或 full_ws，未推送远端。

新消息直接导出 VIO 关键帧中的机体位姿、速度、相机外参、原生特征 ID、
对应二维观测及当前相机系三维点。保留原始输入的整数采样时间，避免仅用
VIO 内部 double 时间重建后悄悄损失纳秒信息。VIO 重置产生新的 session epoch。

回环模块当前能够验证、保存并通过服务返回关键帧；检索、局部描述子、匹配和 PnP
尚待实现。后端提供 Pose3 输入/更新/查询接口及噪声验证，没有假装已实现 iSAM2 求解。
未知协方差和描述子均以有效性标记明确表示，不填入伪精确信息。

## 3. 问题与解决办法

- HTTPS 克隆先受网络限制，网络访问后远端仍要求认证。
  先在独立目录初始化本地 main 分支并关联 origin，未访问任何凭据或强制推送。
- 原 SuperPoint 接口只做新点检测，不输出回环所需描述子。
  保留其真实能力，提供描述子字段和无效标记，避免伪造已完成的回环功能。
- 原 VINS 构造器依赖全局零初始化，普通实例会在 clearState 中访问未初始化指针。
  只在新库副本初始化相关指针，合成导出夹具验证通过。
- 原线程在 ROS 退出时可能仍等待图像/IMU或触发 joinable 析构终止。
  加入停机条件和同步线程回收，双机启动/正常退出测试通过。
- ROS 首帧发布可能早于缓存订阅完成 TCP 握手。
  测试显式等待两个订阅连接，不靠任意延时或锁存消息掩盖问题。
- 路径包含空格时 launch 的配置文件参数会被拆开。
  为配置路径保留引号，并在本仓库真实路径下完成双节点启动验证。
- 系统 cv_bridge 链接 OpenCV 4.2。
  本轮构建显式选择对应的系统 OpenCV，避免混用机器上其他 OpenCV 安装。

## 4. 注意事项 (Precautions)

- 当前改动是第一阶段框架与接口，不包含完整回环、优化、UWB 或调度算法。
- 最终 catkin 构建和本地隔离 ROS smoke 均通过，详细结果见 docs/VALIDATION.md。
- 合成测试不代表真实数据精度，也未执行 GPU engine 推理。
- 模型、engine、数据包、build/devel、日志及结果不提交 Git。
- GTSAM 本机头文件为 4.3a1，不能宣称已使用稳定发布版；正式部署需固定版本。
- TensorRT 代码目前使用 TensorRT 10 API；未来 Jetson 的版本和 engine 需单独适配。
- 标定示例只适用于 EuRoC；自采数据必须替换成实际单目标定。
- package.xml 中新增包维护人邮箱是 example.invalid 占位符，正式发布前由维护者填写。
- 已保留 GPL v3 及来源记录；源项目和新库是不同工作目录，不嵌套导入旧 .git。
- GitHub 认证尚未完成，当前结果在独立本地工作树中，未创建远端提交。
