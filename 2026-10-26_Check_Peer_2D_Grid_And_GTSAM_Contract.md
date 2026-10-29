# 2026-10-26_Check_Peer_2D_Grid_And_GTSAM_Contract

## 1. 修改文件清单
- `src/rdswarm_loop/test/feature_enrichment_test.cpp`：修正增补后特征数量断言；增加网格预算下的跨网格覆盖检查。
- `src/rdswarm_backend/CMakeLists.txt`：将 GTSAM 后端契约测试注册到 CTest。
- `src/rdswarm_backend/package.xml`：声明 GTSAM 运行/构建依赖。
- `README.md`：补充特征增补测试入口。
- `docs/PEER_2D_FEATURES.md`：同步当前四项 CTest 和网格覆盖验证状态。
- `src/vins_fusion/config/euroc/cam0_pinhole.yaml`：清理尾随空格。

## 2. 功能实现说明
检查确认邻机二维特征链路为 `feature_node -> enrichKeyframe -> SuperPointCpu::detect`，网格参数实际进入检测器。网格模式按候选分数排序后逐格轮询，在预算内优先提供空间覆盖；原始 VIO 特征、三维有效性和高位图像点 ID 约定保持不变。

## 3. 问题与解决办法
重新配置 `build_cpu` 后，新增增补测试首次因旧数量断言将四个有效描述子误写成三个而失败，已修正。随后增加了两个网格候选的跨网格断言，避免测试只验证数量而不验证覆盖。后端契约测试此前未注册到 CTest，已补充注册。

## 4. 注意事项 (Precautions)
- 当前 GTSAM 后端仍是 Pose3 数据协议、约束校验和抽象接口；尚未实现 iSAM2 因子图、更新流程或相对协方差服务。
- VINS-Fusion 和 `camera_models` 仍依赖 Ceres；本轮只确认 RDSwarm 后端契约使用 GTSAM，不代表整个工作区已移除 Ceres。
- 真实记录帧的邻机二维服务结果可证明接口和一组 PnP 链路，但不能证明真实回环召回率、位姿精度、无线传输或后端优化。
- 当前 CPU 构建的四项 CTest 均通过；ROS master/网络相关回归在受限沙箱中未重新启动，已有记录结果仍按其原始测试范围解释。
