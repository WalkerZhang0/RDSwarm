# 导入来源

src/vins_fusion 来源于本机 /home/walker/swarm/fusion_ws/src/VINS-Fusion，
其上游为 HKUST-Aerial-Robotics/VINS-Fusion。导入的是含用户 SuperPoint 修改的工作副本，
不是纯上游发布版。源提交、导入文件 SHA256 及排除范围见 provenance/vins_fusion_import.json。

保留原版权头及 GNU GPL v3 许可证，根 LICENSE 及 src/vins_fusion/LICENCE 为许可证副本。
RDSwarm 新增代码也使用 GPL-3.0-only；不要将组合工程误标为 MIT。
package.xml 的 example.invalid 维护人地址是显式占位符，发布前由仓库维护者填写。

未导入旧工程的 .git、loop_fusion、global_fusion、数据包、结果、模型权重和 TensorRT engine。
不自动下载或重新许可 SuperPoint 权重。engine 通过启动参数提供，平台部署时单独准备。

原工程只读，新代码和适配补丁只存在于 RDSwarm。
远端初始化时 GitHub 要求认证，本地 origin 已关联给定地址，未执行 push。
