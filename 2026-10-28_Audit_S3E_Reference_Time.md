# 2026-10-28_Audit_S3E_Reference_Time

## 1. 修改文件清单

- `scripts/audit_s3e_reference_time.py`：追溯原始图像 header/record 与缓存像素、GT 插值端点和 GNSS 投影/高度对应。
- `docs/S3E_REFERENCE_TIME_AUDIT.md`：记录精确的时间对应、距离计算、GNSS 质量和结论更正。
- `docs/LOOP_GEOMETRY_AUDIT.md`、`BIDIRECTIONAL_PNP_PAIR_00152.md`、`GLOBAL_LOOP_BASELINE.md`：在顶部标注参考数据更新，保留原始诊断数值。

## 2. 功能实现说明

只读读取 S3E SQLite 原始 bag，完整匹配两份文本 GT 与 GNSS fix，使用系统 PROJ 计算 WGS84 UTM 49N。逐行记录时间差、水平投影差、分段高度偏移，并对当前 pair_00152 的图像做完整像素一致性核验。

## 3. 问题与解决办法

原先仅验证短时间插值区间，没有验证参考文件的高精度来源。新审计发现 Alpha 400 行与 Carol 377 行水平坐标对应原始 fix，投影差最大约 0.022 mm；时间差最大 219/5204 ns。两张图像源 header、关键帧时间及像素完全一致。

Alpha 原始 fix 在相应区间的自报近似水平标准差为约 4.212/2.9 m；Carol 前 40 行文本高度比原始 fix 低 19.7355 m，其余行没有该偏移。由此撤回将 0.179 m 当作高精度 body 距离、据此确认算法数米误差的解释。原始 GNSS 存在 NaN 记录，投影时显式排除，保留原始序号再与有效文本对应。

## 4. 注意事项 (Precautions)

- 消息时间数值吻合不能证明真实曝光/GNSS 测量历元同步，物理时间偏移仍待核验。
- GNSS 自报 APPROXIMATED 协方差不是校准真值，不能简单当独立误差传播或断定真实误差大小。
- 脚本不修改数据、不改变既有 PnP、不调时间或高度去匹配估计结果。
- 全文数值为原输入的重现和来源核验；没有新增需要中文注释解释的复杂算法分支，未增加冗余注释。
