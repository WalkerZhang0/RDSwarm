# 2026-10-25_Extend_Peer_2D_Features

## 1. 修改文件清单

- `src/vins_fusion/vins_estimator/src/utility/rdswarm_export.cpp`：导出当前关键帧全部有效二维轨迹；成熟深度仅作为三维附加条件。
- `src/vins_fusion/vins_estimator/test/export_contract_fixture.cpp`：增加未三角化、失败/无效深度和当前帧轨迹的导出验收。
- `src/rdswarm_loop/include/rdswarm_loop/feature_enrichment.hpp`、`src/feature_enrichment.cpp`：原点保留、SP 新点去重追加、相机归一化及独立 ID。
- `src/rdswarm_loop/src/feature_node.cpp`：接入增补函数，读取显式参数并输出原点/三维/追加/去重计数。
- `src/rdswarm_loop/CMakeLists.txt`、`package.xml`：使用已导入的 camera_models；新增增补库、单元测试及离线探针。
- `src/rdswarm_loop/test/feature_enrichment_test.cpp`、`test/describe_probe.cpp`：无 ROS master 的增补逻辑测试和真实关键帧离线回归入口。
- `src/rdswarm_bringup/launch/describe_agent.launch`：暴露开关、标定、检测预算和去重半径，S3E 默认配置两台各自标定。
- `src/rdswarm_msgs/msg/TrackedFeature.msg`：仅补充帧内二维 ID 约定注释，未改消息字段。
- `tests/peer_2d_services_test.py`：真实记录帧的服务链回归；`tests/loop_services_test.py`：旧合成基线显式关闭追加。
- `docs/PEER_2D_FEATURES.md`、`INTERFACES.md`、`LOOP_WORKFLOW.md`、`S3E_INTER_AGENT_LOOP.md`、`TWO_IMAGE_DIAGNOSIS.md`、`README.md`：更新当前接口、独立运行方法和结果边界。

## 2. 功能实现说明

B 的二维观测不再以三角化成功为前提。VIO 原生输出中，二维坐标与身份有效者均保留，只有满足原有成熟条件且相机系深度有效的点才设置 `position_valid=true`。

描述子节点保留原特征顺序、ID、坐标及深度，仅为它们采样描述子，然后追加去重 SP 检测点。新点使用帧内最高位 ID，不具有三维或深度方差。归一化使用与 VIO 一致的相机模型，并校验图像尺寸及原有归一化观测。

本机 PnP 只用有效三维点，邻机可用全部二维候选。未修改匹配/PnP 门限、后端或机间重投影实现；服务仍需显式调用。

## 3. 问题与解决办法

原逻辑为三维点导出设定的筛选同时限制了 B 二维候选。将二维导出与三维有效性分开，并在描述阶段追加独立检测点，避免原先替换整个 B 点集。

候选追加需要相机归一化及身份隔离，不能将新点与旧深度按数组序号拼接。新增校验、像素去重及独立 ID 范围；使用与在线节点相同的增补函数离线回归，避免两个实现分叉。

验证结果：CPU 构建与三项相关 CTest 通过；原生导出 fixture 的二维/三维、时间、缓存及 epoch 验证通过。三组旧关键帧中 A 有效三维和原描述子逐字段保持一致；B 候选为 1080/1103/992，PnP 内点由 11/11/5 变为 7/7/15。两组被当前门限拒绝，一组改善，未将失败结果包装为精度提升。

真实记录帧的在线 describe/get/fetch/verify 链路通过：取回 992 个 B 二维特征、有效三维为 0、无原图，PnP 24 匹配/15 内点，与离线一致。测试初次出现 rosbag 动态消息类及空数组 tuple/list 的比较误报，规范化到当前消息类型并按原数组类型比较后通过；实际坐标和时间未发生变化。

旧合成服务链在显式关闭追加后通过：两端各 42 个描述子，42 个 PnP 内点，归一化重投影 RMS 约 1.71e-8。逐个复核前两组旧匹配，分别有 10/8 个被扩大候选集后的 ratio 检查淘汰，确认原点保留不等于原匹配保留。

## 4. 注意事项 (Precautions)

- 新增候选会改变 ratio 与互检对象，即使旧点保留也可能失去旧匹配；本轮没有改匹配策略来维持旧匹配。
- 高位 ID 仅对当前 FrameKey 有效，后续共视优化不能当作已有跨帧地标。
- 参数、标定和模型在启动时固定；修改后重启描述/回环节点并换 session 或重放。
- 基于旧 bag 的三组数值对照不包含新导出的未成熟二维轨迹；该导出分支由原生 fixture 单独验收。
- 在线测试基于记录关键帧与同 master，不代表无线通信、实时整段 VINS 或 GT 位姿精度已通过。
- 追加二维候选使示例载荷增加到约 1.09 MB，压缩与调度尚待处理。
- 模型、bag 和结果留在忽略目录，既有用户修改不覆盖；没有引入 CUDA/TensorRT 必需依赖。
