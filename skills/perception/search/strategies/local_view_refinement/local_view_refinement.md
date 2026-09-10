---
id: search.local_refinement
name: local_view_refinement
display_name: Local View Refinement
parent_skill: search
version: 1.0.0

description: >
  围绕当前相机位姿生成小范围平移和旋转候选，以改善目标居中、深度覆盖和位姿估计质量。

input_schema: strategies/local_view_refinement/local_view_refinement.schema.yaml
implementation: strategies/local_view_refinement/local_view_refinement.py
---

# Local View Refinement Strategy

## Name
Local View Refinement

## Description
围绕当前相机位姿生成小范围平移和旋转候选，以改善目标居中、深度覆盖和位姿估计质量。

## When To Use
适用于：
- 目标已经可见或部分可见
- 置信度低于任务要求
- 深度覆盖、清晰度或位姿质量不足
- 只需要小范围视角调整

不适用于：
- 目标完全不可见且没有目标位置先验
- 需要覆盖整个工作空间

## Input Requirements
- 当前相机位姿
- 当前目标二维区域或目标射线
- 至少一种不足的观测质量指标
- 相机内参和视场信息

## Strategy Parameters
- `translation_step`
- `rotation_step`
- `candidate_count`
- `optimize_for`
- `keep_target_in_view`
- `maximum_local_radius`

## Execution Logic
1. 读取当前目标区域和观测质量。
2. 围绕当前相机位姿生成小幅平移和旋转候选。
3. 为候选生成保持目标可见的 look-at 朝向。
4. 预测目标居中、深度覆盖和表面入射角改善。
5. 去除超过局部半径或历史重复的候选。
6. 输出质量增益和最小运动代价。
7. 交由统一 Viewpoint Validator 完成最终验证和排序。

## Output
返回：
- candidate_viewpoints
- expected_visibility_gain
- expected_information_gain
- source = local_view_refinement
- evidence
- search_progress

## Failure Handling
- `OBSERVATION_INSUFFICIENT`: 缺少当前目标区域或质量指标
- `VIEWPOINT_GENERATION_FAILED`: 局部扰动候选生成失败
- `NO_QUALITY_GAIN`: 候选无法改善观测质量
- `NO_VALID_VIEWPOINT`: 候选均未通过验证

本文件只描述策略语义；候选生成、射线投射、视场计算、几何分析和机器人验证实现由用户接入。
