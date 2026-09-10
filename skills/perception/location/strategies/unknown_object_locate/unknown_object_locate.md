---
id: locate.unknown_object
name: unknown_object_locate
display_name: Unknown Object Locate
parent_skill: locate
version: 1.0.0

description: >
  使用 VLM 或开放词汇视觉模型定位自然语言描述的目标，并结合深度或点云生成三维操作目标位姿。

input_schema: strategies/unknown_object_locate/unknown_object_locate.schema.yaml
implementation: strategies/unknown_object_locate/unknown_object_locate.py
---

# Unknown Object Locate Strategy

## Name
Unknown Object Locate

## Description
使用 VLM 或开放词汇视觉模型根据目标描述定位二维目标点或区域，并结合深度图、点云和局部几何生成三维操作目标位姿。

## When To Use
适用于：
- 无目标模型和明确类别先验
- 目标由自然语言描述
- 需要开放词汇目标或区域定位
- 目标可能是物体局部或语义位置

不适用于：
- 已知高质量模型且需要高精度配准
- 缺少可用视觉观测
- 描述无法唯一对应场景目标

## Input Requirements
- `target.object`、`target.instruction` 或 `target_description`
- RGB 图像
- 三维定位时需要深度图或点云
- 相机内参和坐标变换

## Strategy Parameters
- `target_description`
- `vlm_model`
- `point_selection_mode`: `point | region | region_center`
- `depth_sampling_radius`
- `depth_aggregation`: `median | mean | nearest_valid`
- `minimum_valid_depth_points`
- `orientation_estimation`
- `operation_point_rule`

## Execution Logic
1. 获取同步视觉和深度观测。
2. 解析目标描述与操作点语义。
3. 使用 VLM 或开放词汇模型进行目标 grounding。
4. 选择二维点或目标区域。
5. 聚合深度并投影到三维空间。
6. 根据局部表面或任务规则估计姿态。
7. 输出候选位姿、grounding 置信度和证据。
8. 交由统一 Pose Validator 完成最终验证。

## Output
返回：
- target_pose
- confidence
- image_point / region
- source = unknown_object
- evidence

## Failure Handling
- `TARGET_DESCRIPTION_AMBIGUOUS`: 目标描述歧义
- `TARGET_NOT_FOUND`: VLM grounding 失败
- `DEPTH_UNAVAILABLE`: 目标区域无有效深度
- `INSUFFICIENT_POINTS`: 有效深度样本不足
- `PROJECTION_FAILED`: 二维点无法投影到三维
- `ORIENTATION_ESTIMATION_FAILED`: 无法获得所需姿态
- `LOW_CONFIDENCE`: 结果低于阈值

本文件只描述策略语义；VLM、开放词汇检测、深度采样和三维投影实现由用户接入。
