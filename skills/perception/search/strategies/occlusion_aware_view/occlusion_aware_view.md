---
id: search.occlusion_aware
name: occlusion_aware_view
display_name: Occlusion-Aware View
parent_skill: search
version: 1.0.0

description: >
  根据目标粗位置、可见区域和遮挡关系，生成能够降低遮挡并保持目标在视野中的候选观察位姿。

input_schema: strategies/occlusion_aware_view/occlusion_aware_view.schema.yaml
implementation: strategies/occlusion_aware_view/occlusion_aware_view.py
---

# Occlusion-Aware View Strategy

## Name
Occlusion-Aware View

## Description
根据目标粗位置、可见区域和遮挡关系，生成能够降低遮挡并保持目标在视野中的候选观察位姿。

## When To Use
适用于：
- 目标部分可见
- 存在遮挡比例、遮挡物或遮挡方向证据
- 存在目标粗位姿、目标射线或目标先验
- 需要从侧向或上方绕开遮挡

不适用于：
- 目标完全不可见且没有任何目标先验
- 任务要求移动或移除遮挡物

## Input Requirements
- 目标粗位姿、目标射线或目标先验位姿之一
- 部分可见或遮挡证据
- 当前相机位姿与场景几何
- 相机内参和视场信息

## Strategy Parameters
- `orbit_radius`
- `azimuth_offsets`
- `elevation_offsets`
- `candidate_count`
- `keep_target_in_view`
- `minimum_expected_occlusion_reduction`
- `occluder_clearance`

## Execution Logic
1. 估计目标、当前相机和遮挡物的相对关系。
2. 从 VLM 或几何模块获得遮挡粗方向。
3. 围绕目标生成侧向、上方和环绕候选。
4. 通过射线或可见性模型估计遮挡降低。
5. 去除目标离开视场或与遮挡物过近的候选。
6. 输出预期可见性增益和遮挡证据。
7. 交由统一 Viewpoint Validator 完成最终验证和排序。

## Output
返回：
- candidate_viewpoints
- expected_visibility_gain
- expected_information_gain
- source = occlusion_aware_view
- evidence
- search_progress

## Failure Handling
- `OCCLUSION_EVIDENCE_INSUFFICIENT`: 遮挡信息不足
- `TARGET_PRIOR_INVALID`: 无法确定目标粗位置或射线
- `VIEWPOINT_GENERATION_FAILED`: 环绕候选生成失败
- `NO_VISIBILITY_GAIN`: 候选不能改善可见性
- `NO_VALID_VIEWPOINT`: 候选均未通过验证

本文件只描述策略语义；候选生成、射线投射、视场计算、几何分析和机器人验证实现由用户接入。
