# 可移植回环三步验证记录

日期：2026-10-24。只记录本轮实际检查，不把原阶段报告当作新功能验证。

## 已完成的检查

### CPU 构建和依赖

全新 build_cpu/devel_cpu，以 RDSWARM_WITH_TENSORRT=OFF 配置编译通过。
默认没有查找或链接 YOLO、Torch、CUDA、TensorRT。
ldd 检查 libvins_lib.so、librdswarm_features.so、librdswarm_loop_core.so，
均无 nvinfer、cudart、libcuda、torch、yolo 依赖。

### 描述子

1. 解析子像素采样测试通过：双线性、L2、越界/NaN、零向量。
2. 实际用户 ONNX 权重截取密集头后，由 OpenCV DNN CPU 推理通过。
3. 与原 ONNX 稀疏输出在 40 个相同点进行真实数值对照：
   - 维度 256；
   - 最大绝对误差 3.054738e-7；
   - 最小 cosine 0.99999994；
   - 平均 cosine 1.0。
4. VINS 公共 detector facade 选择 opencv 后端运行通过，测试图检测到 27 点。
   这不是原 TensorRT 检测后处理的逐位等价测试。

### CPU 几何

独立 C++ 测试通过：
- 全局向量归一化、Top-K 和无效向量拒绝；
- 双向互检/ratio 局部匹配；
- 非零旋转、平移、两端非零相机外参；
- 40 个匹配中的 8 个错误二维观测，由 RANSAC 拒绝；
- 验证 T_Ba_Bb 而非其逆或相机系变换；
- 模型不匹配、缺描述子、近共线几何拒绝；
- 默认未知协方差无效，配置名义噪声时来源明确。

### 实际 ROS 服务链

在独立 localhost:11541 master 上启动两台 feature 节点和两台 loop 节点，
使用两个已知同视图的合成 raw 关键帧：

- auto_describe=false 时未自动执行描述子工作；
- 手动 describe 后双方各有 42 个 256-D 描述子；
- 帧键、原采样时间和 feature_id 保持对应；
- 未请求对端数据前 verify 返回明确缺失状态；
- 非法服务名及非法运行参数明确拒绝，恢复参数后验证成功；
- 对端特征包没有 image 字段，有效三维点数为 0；
- 显式 fetch 后 PnP 验证得到 42 内点，归一化重投影 RMS 约 1.71e-8；
- 默认没有伪造有效协方差；
- VerifiedMatches 对应数据可用，未创建重投影残差；
- 合成全局向量注入后 Top-K 查询得到预期候选。

当前合成数据的局部包序列化为 46,190 B，包含 42 个 256-D 描述子和元数据。
这只是该消息/夹具的实测字节数，不是压缩收益或无线带宽评估。
include_3d=false 清除有效三维数据，但保留 schema 中三维占位开销。

### 原接口回归

以新 devel_cpu 执行既有 smoke_interfaces.py：
原始纳秒时间、相机系三维转换、重启 epoch、缓存请求、
后端输入校验和双节点启动/正常退出通过。

### 可选 GPU 配置

全新 build_gpu/devel_gpu，以 RDSWARM_WITH_TENSORRT=ON 编译独立 TensorRT 库通过。
ON 构建中的公共检测器选择 opencv 后端也运行通过，检测到同样的 27 点。
本轮未运行旧 TensorRT engine 推理；不能由成功编译推导 GPU 输出或耗时有效。

## 结果位置和重现

- logs/build_cpu.log、build_cpu_2.log、build_cpu_3.log：CPU 构建。
- logs/descriptor_parity/summary.json：原模型对照。
- logs/phase2_integration/summary.json：两机服务链。
- logs/interface_smoke_devel_cpu/summary.json：原接口回归。
- 模型准备、逐终端运行和逐项测试命令见 LOOP_WORKFLOW.md。
- tests/publish_loop_fixture.py 只负责发布夹具，不封装整个运行流程。
- tests/loop_services_test.py 是可选自动回归，支持 --model、--devel-space、--port；实际使用仍按手动工作流。
- 两个新增 C++ 测试已注册 CTest，当前 2/2 通过。

## 未完成及结论边界

- MobileNetVLAD 推理未接入；合成向量只检验索引，不代表回环检索质量。
- 未测试真实双机图像匹配、S3E 轨迹精度、召回率或误回环率。
- 未实现协同后端更新、调度、机间重投影、UWB。
- 未进行 GPU/CPU 性能公平比较、真实无线通信或 Jetson 部署验证。
- 测试通过表明当前模块和协议可用，不证明统计一致性或真实误差上界。
