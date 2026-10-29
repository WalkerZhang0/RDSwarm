# 两张图的 SuperPoint 匹配与 VIO 三维点诊断

后续已将“保留 VIO 原二维点＋追加 SP 二维点”接入描述子节点，当前实现与服务验证见 [邻机二维候选](PEER_2D_FEATURES.md)。下文保留当时用于定位问题的替换式对照结果。

## 本轮结论

已对前轮相同的三组 Alpha/Carol 原始图像做四种对照，未更换网络权重，未改默认匹配/PnP 门限。两张图直接检测能得到显著更多的局部匹配；目前链路只导出约 100 个有深度的 VIO 点，是匹配数量受限的重要原因。不能由此认定网络只能提取少量特征，或描述子采样实现损坏。

这里验证的是给定图像对的局部匹配与几何一致性。MobileNetVLAD 全局检索尚未接入，不能将这项结果称为从数据库自动检索到了回环。

## 四种对照

| 模式 | 本机点 | 邻机点 | 深度使用 |
| --- | --- | --- | --- |
| `vio_vio` | 既有 VIO 点＋采样 SuperPoint 描述子 | 既有 VIO 点＋采样描述子 | 本机 VIO 三维点用于 PnP；对端三维标记清除 |
| `vio_superpoint` | 同一批本机 VIO 点，保持三维与 ID 不变 | SuperPoint 独立检测点＋描述子 | 本机 VIO 三维点用于 PnP；对端新点只提供二维 |
| `superpoint_equal_budget` | SuperPoint 检测点，数量截到本机 VIO 预算 | 同理截到邻机 VIO 预算 | 只做二维几何，无已知深度 |
| `superpoint_superpoint` | 最多 1024 个 SuperPoint 检测点 | 最多 1024 个 SuperPoint 检测点 | 只做二维几何，无已知深度 |

固定参数：检测置信度 0.02，最多 1024 点，最小点距 8 像素，边界 8 像素；匹配 ratio 0.8、描述子 L2 距离不大于 0.9、双向互检。几何使用去畸变归一化坐标，阈值 0.004；二维以 F-RANSAC/H-RANSAC诊断，PnP 直接调用现有 C++ `verifyLoop` 默认实现。

`superpoint_equal_budget` 按检测器的分数排序取前 N 个点，没有重新做空间均匀化。它用于粗略区分点数和检测位置的作用，不是对不同检测器精度的完整比较。

## 三组实际结果

| 帧对 | 模式 | A/B 点数 | 局部匹配 | F 二维内点 | PnP 内点 |
| --- | --- | ---: | ---: | ---: | ---: |
| 0 | VIO/VIO | 96/101 | 14 | 8 | 11，通过 |
| 0 | VIO/SP | 96/1024 | 7 | — | 0，匹配不足 |
| 0 | SP，同预算 | 96/101 | 12 | 7 | 无深度 |
| 0 | SP/SP | 1024/1024 | 116 | 87 | 无深度 |
| 1 | VIO/VIO | 96/122 | 16 | 14 | 11，通过 |
| 1 | VIO/SP | 96/1024 | 15 | 11 | 8，通过 |
| 1 | SP，同预算 | 96/122 | 19 | 16 | 无深度 |
| 1 | SP/SP | 1024/1024 | 159 | 123 | 无深度 |
| 2 | VIO/VIO | 105/96 | 21 | 14 | 5，未通过 |
| 2 | VIO/SP | 105/928 | 19 | 16 | 13，通过 |
| 2 | SP，同预算 | 105/96 | 22 | 18 | 无深度 |
| 2 | SP/SP | 1024/928 | 174 | 149 | 无深度 |

F 内点与 PnP 内点来自不同模型和 RANSAC 检查，不能直接等同；例如两张图无深度时，123 个二维内点不代表 123 个 PnP 内点或已恢复米制相对位姿。

同预算 SP 匹配数量与 VIO 相近，明显增益主要出现在使用更大点集时。因此当前证据支持“参与回环的点集规模受限”，不足以单独归因于 KLT 像素处采样描述子不正确。VIO 点与 SP 检测点确实通常不重合：帧对 1 的 Alpha 96 点中仅 8 个距离 SP 检测点不超过 2 像素；不能把两组点的深度按数组顺序拼接。

只增加邻机点数的效果并不稳定：三组 PnP 内点分别由 11/11/5 变为 0/8/13。邻机新点的位置变了，ratio 的次近邻和互检对象也随之改变，因此特征更多不保证原有匹配保留。帧对 2 的新成功结果只说明通过几何门限，仍未完成独立位姿精度验证。

本机深度也限制平移质量。原帧对 1 的 11 个 PnP 内点全部位于约 27.47～39.96 米，中位数约 29.86 米；本机三维点对自身二维观测的重投影差中位数约 1.52 像素。远点相对缺少近处视差，平移可观测性较弱。深度的真实误差尚未测定，不能将点数问题视为全部原因。

下一项实现宜先扩充本机可提供可靠深度的特征轨迹，并保留同一 `feature_id` 的二维/三维/描述子关系，观察近处点和空间覆盖，再评价 PnP。邻机二维检测可独立扩大，但不能代替本机三维覆盖。

## 原有 PnP 模式是否已使用

是。实际输入为：

```text
Alpha VIO 的 feature_id、相机系三维点、同一像素位置的 SuperPoint 描述子
                         ↓ 局部匹配
Carol 的二维归一化观测、描述子
                         ↓
                 3D–2D PnP-RANSAC
```

原测试和本次 `vio_vio` 均使用这条链路，不用邻机三维点，也没有临时靠双目恢复尺度。本机三维尺度来自当前单目＋IMU VIO 的估计。

本轮验证了三组原 PnP 的匹配/内点数完全复现，成功位姿与前轮结果一致；本机序列化三维点包逐字段相等。新邻机包的 `position_valid` 全为 false，消息没有原图字段。二维归一化坐标与原标定重算一致。

## 手动输入两张图

不启动 ROS master、VINS 或回环服务。以下按 Bash；zsh 用户使用对应的 `setup.zsh`。编译已完成在 `devel_cpu`，两个探针均属于测试构建目标。

```bash
cd "/media/walker/My PSSD/tmp/论文/RDSwarm"
source /opt/ros/noetic/setup.bash
source devel_cpu/setup.bash
mkdir -p results/two_image_manual
```

第一步：对图 A 检测和描述。下面两个路径是已保存的原始关键帧图像，可以替换为自己的图像；此工具不缩放图像。

```bash
devel_cpu/lib/rdswarm_features/superpoint_image_probe \
  "$PWD/models/superpoint_dense.onnx" \
  "$PWD/results/s3e_loop_validation/offline/0_1660900402165000000.png" \
  "$PWD/results/two_image_manual/a.yml" \
  0.02 1024 8 8
```

第二步：对图 B 检测和描述。

```bash
devel_cpu/lib/rdswarm_features/superpoint_image_probe \
  "$PWD/models/superpoint_dense.onnx" \
  "$PWD/results/s3e_loop_validation/offline/1_1660900408127000000.png" \
  "$PWD/results/two_image_manual/b.yml" \
  0.02 1024 8 8
```

四个数依次是 `score_threshold/max_points/min_distance/border`，可以显式改变。模型权重和采样约定保持一致；变更参数时使用新输出目录保留对照。

第三步：匹配和二维几何验收，输出 JSON 和连线图。

```bash
/usr/bin/python3 scripts/inspect_image_pair.py \
  --features-a results/two_image_manual/a.yml \
  --features-b results/two_image_manual/b.yml \
  --calib-a results/alpha_cam0.yaml \
  --calib-b results/carol_cam0.yaml \
  --output results/two_image_manual/matching01
```

这三个命令只检验给定图像对，不封装其他系统启动。改变图像尺寸或设备时必须提供相应 PINHOLE 标定；脚本会拒绝图像尺寸与标定不一致。`report.json` 已存在时要求使用新目录，避免覆盖实验结果。

如果同时需要复现 VIO 对照，在第三步增加以下两个选项：

```text
--vio-a-packet results/s3e_loop_validation/offline/0_1660900402165000000_local.bin
--vio-b-packet results/s3e_loop_validation/offline/1_1660900408127000000_local.bin
```

包必须属于对应的两张原图，并使用同一模型。仅提供两个图像无法凭空生成可靠米制深度。脚本使用本机已保存的原始 VIO 包，不重新赋予新检测点旧三维坐标。

## 输出与验证

- `results/two_image_diagnosis/summary.json`：三组对照汇总和输入关系校验。
- `results/two_image_diagnosis/pair_1/diagnostic_v2/report.json`：示例帧对的各阶段淘汰数量、空间覆盖及深度统计。
- [两图 SP 匹配图](../results/two_image_diagnosis/pair_1/diagnostic_v2/superpoint_superpoint_2d_inliers.jpg)：159 个局部匹配中的 123 个二维几何内点。
- [原 PnP 内点图](../results/two_image_diagnosis/pair_1/diagnostic_v2/vio_vio_pnp_inliers.jpg)：原来的 11 个 PnP 内点。
- `results/two_image_diagnosis/controls/validation.json`：同图自匹配 1024 对、空白图零匹配、错误标定尺寸和非法点数被拒绝。

CPU 构建、现有 SuperPoint 采样与 PnP 核心测试通过。模型采样此前已有数值对照，本轮重点验证真实图像的点集与几何覆盖。所有结果为局部图像对诊断，未计算数据库检索召回率或新的 GT 位姿精度。
