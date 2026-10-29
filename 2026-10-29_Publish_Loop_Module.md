# 2026-10-29_Publish_Loop_Module

## 1. 修改文件清单

- `src/rdswarm_bringup/config/s3e/{alpha,carol}_{cam0,vins}.yaml`：将部署必需标定和 VIO 配置从本地 results 整理到包内配置；移除作者机器绝对输出路径。
- `src/rdswarm_bringup/launch/{s3e_alpha,s3e_carol,s3e_dual,describe_agent}.launch`：默认读取包内 S3E 配置，保持运行结果输出独立。
- `scripts/evaluate_global_loop.py`、`tests/global_loop_services_test.py`：标定默认路径随部署配置迁移。
- `.gitignore`：完整忽略 results；额外忽略 db3/mcap，继续排除模型、日志和构建产物。
- `docs/DEPLOYMENT.md`、`docs/S3E_DUAL_VINS.md`、`docs/LOOP_WORKFLOW.md`、`README.md`：CPU 换机依赖、模型校验和/准备、包内配置入口、跨主机边界。
- 本次 Git 快照同时纳入前几轮尚未提交的回环实现、VIO 适配、测试、诊断脚本及对应技术文档/变更记录，具体文件以提交清单为准。

## 2. 功能实现说明

将已验证的 VIO→MobileNetVLAD→按需 SP→BF/PnP→相对约束代码形成源码部署快照。模型单独准备，不依赖作者机器结果目录。保留 agent/session、原采样时间、局部地图及外参约定、来源元数据和后端/共视预留接口。提交说明使用用户指定的“回环模块补充完成”。

## 3. 问题与解决办法

- 启动默认配置曾依赖不会上传的 results。新增包内 config/s3e，并同步启动与验证默认路径；results 中原文件保留，不复制实验输出进仓库。
- 两个源 EuRoC 配置被本地临时改成 S3E 参数，与本次部署默认配置无关；不提交这两处修改，不改写用户本地文件。
- Alpha acc_n=0.1 是诊断实验值，新配置显式标注未校准，不能延续“全部沿用原标定噪声”的误导表述。
- 模型、数据集、ONNX Runtime 等不能通过只上传代码自动获取。部署文档列明依赖与 SHA256；VINS/camera_models 仍需 Ceres，GTSAM 求解器仍未实现。
- 验证：CPU 构建、五项 CTest、Python/launch 语法、真实 VLAD 预处理测试、包内标定与本地来源数值对照；提交清单审查排除结果/模型/编译产物。隔离端口 11631 的 ROS 集成通过（16 关键帧、16 摘要、8 次验证、1 接受、历史回取与 fallback 通过），记录在未上传的 `logs/publish_loop_20261029/summary.json`。从暂存区导出不含 results/models 的源码快照，验证包内配置解析和默认相机路径，无需作者本地配置。

## 4. 注意事项 (Precautions)

- 仅上传关键源码与技术资料，不上传实验图片/包/日志/模型；部分历史报告里的产物链接需在本地重新生成。
- 目标机自行准备一致模型、标定、数据与依赖，源码构建后复核。没有声称已验证 ARM 或跨主机网络部署。
- 同 master 多 agent 已验证，跨 master/WiFi 桥、网络恢复、邻机后端状态摘要、iSAM2 与校准协方差均未完成。
- 关键配置/路径处添加带本日志引用的中文注释；未为简单路径替换增加重复注释。
