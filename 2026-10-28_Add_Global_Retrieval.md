# 2026-10-28_Add_Global_Retrieval

## 1. 修改文件清单

- `src/rdswarm_loop/python/rdswarm_runtime/`、`setup.py`：4096-D MobileNetVLAD CPU 适配、磁盘序列化、JSONL 日志和有界 worker。
- `src/rdswarm_loop/scripts/global_descriptor_node.py`：VIO 关键帧触发的独立全局描述节点，可选非关键帧超时 search-only。
- `src/rdswarm_loop/scripts/loop_baseline_node.py`：Top-K→按需描述→特征请求→BF/PnP 的自动验证，帧对去重、约束和共视预留包保存。
- `src/rdswarm_loop/include/rdswarm_loop/keyframe_store.hpp`、`src/keyframe_store.cpp`：可配置磁盘历史与有限 RAM 工作集。
- `src/rdswarm_loop/src/feature_node.cpp`、`src/loop_node.cpp`：使用历史缓存，loop 缓存去除原图，可配置邻机缓存上限。
- `src/rdswarm_loop/include/rdswarm_loop/covisibility_interface.hpp`：预留两端 packet、关联和来源的共视 bundle 接口，无重投影因子实现。
- `src/rdswarm_loop/test/describe_probe.cpp`：显式点预算与 SP 分段计时。
- `src/rdswarm_loop/test/keyframe_archive_test.cpp`：工作集移出、历史取回、重启、损坏和重复身份测试。
- `src/rdswarm_loop/CMakeLists.txt`、`package.xml`：Python 安装、依赖与归档 CTest。
- `src/rdswarm_bringup/launch/global_agent.launch`、`loop_baseline.launch`：独立运行入口。
- `src/rdswarm_bringup/launch/describe_agent.launch`、`loop_agent.launch`：磁盘缓存、摘要话题和 RAM 上限参数。
- `scripts/prepare_mobilenetvlad.py`、`evaluate_global_loop.py`、`check_loop_length_gt.py`：模型准备、因果序列评估及有限 GT 长度核对。
- `tests/global_descriptor_test.py`、`global_loop_services_test.py`：真实模型与 ROS 流程验证。
- `requirements-global.txt`、`docs/GLOBAL_LOOP_BASELINE.md`、`docs/INTERFACES.md`、`docs/LOOP_WORKFLOW.md`、`docs/THIRD_PARTY.md`、`README.md`：依赖、运行说明、来源及状态更新。

## 2. 功能实现说明

接入本地 D2SLAM 现有 MobileNetVLAD ONNX，CPU 输出 4096 维归一化图像描述子；模型 SHA 与 resize/像素范围进入身份。推理与 VIO 解耦，只由 VIO 正式关键帧触发；可选 fallback 使用精确同时间图像和 VIO 状态，只输出搜索候选。

自动 worker 使用现有全局索引和 BF/PnP 服务，保留协方差未知状态。长序列工作集支持磁盘回读，不因 RAM 淘汰失去历史图像。共视 bundle 保存原始匹配和 packet，明确其三维来自 VIO，尚无跨帧轨迹、标定指纹与联合协方差。

## 3. 问题与解决办法

此前只在 HF-Net 源码附近搜索，未发现可用权重；扩大到本地 D2SLAM 后找到真实 4096-D 模型，并核实其像素范围为 0–255。避免把 SP 的 0–1 预处理误用于此模型。没有修改 ROS 消息协议。

CPU 构建和 5 项 CTest 通过；真实模型与 D2SLAM 参考输入约定的描述子数值对照通过，验证重复推理与非法图像拒绝。

40 秒记录序列：112 关键帧（45/67），因果 Top-3 检索验证 318 个不同跨机帧对，95 对通过几何，95 对匹配不足，128 对内点不足；零处理异常。全局中位/P95 8.43/10.91 ms，SP 407.36/424.93 ms，顺序离线墙钟 79.29 s。95 对的 GT 位置参考间距与平移模长绝对差中位/P95 0.919/1.798 m；该指标不是完整平移误差或误回环率。

最终隔离 ROS 测试通过：16 真实记录关键帧、16 摘要、至少 8 次自动验证和几何接受，工作集限制为 1 时历史重载可用；fallback 一次推理产生 3 个候选，不进入关键帧索引。测试只覆盖运行中的一段自动流程，不宣称全部在线队列处理完毕。测试退出时在途服务可能因子进程关闭而在日志中记录连接终止，不计入长期序列结果。

## 4. 注意事项 (Precautions)

- 95 是几何接受的帧对数，可能有误回环和相邻重复事件，不能当成 95 个真回环事件或精确率。
- 当前默认 VLAD Top-K 无 cosine 截断，阈值 -1 只是完整候选基线，不是已校准的判定标准。
- 本轮只评估已有 40 秒关键帧记录，不是完整约 445 秒实时双 VIO＋loop 测试。独立节点支持后续完整回放，SP 仍是主要 CPU 开销。
- 模型、SP 参数和相机标定改变后使用新缓存目录；旧磁盘记录不可静默当新配置结果复用。
- 磁盘与全局摘要索引仍随历史增长；有界队列可能丢任务并记录，需要在完整回放中检查覆盖率。
- 同 master ROS1 服务尚无执行超时/跨网络恢复；WiFi 桥、可靠重试、持久索引恢复不在本次验收范围。
- fallback 只搜索；落到正式关键帧约束需要端点重验证。GTSAM 增量优化、共视重投影和协方差校准按后续研究路线实施。
- 关键预处理、历史缓存、异步描述确认及因果检索处已加中文注释并链接本记录。部署依赖 Python ONNX Runtime CPU，权重许可保持来源约定。
