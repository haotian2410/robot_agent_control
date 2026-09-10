---
id: search.prior_guided
name: prior_guided_search
display_name: Prior-Guided Search
parent_skill: search
version: 1.0.0

description: >
  根据目标位置、区域、跟踪预测、图像边缘或语义方向先验，在高概率区域生成下一观察视角。

input_schema: strategies/prior_guided_search/prior_guided_search.schema.yaml
implementation: strategies/prior_guided_search/prior_guided_search.py
---

# Prior-Guided Search Strategy

## Name
Prior-Guided Search

## Description
根据目标位置、区域、跟踪预测、图像边缘或语义方向先验，在高概率区域生成下一观察视角。

## When To Use
适用于：
- 目标当前不可见但存在位置或区域先验
- 目标刚从图像边缘离开
- 跟踪器能够预测目标运动
- VLM 或用户提供了粗方向提示

不适用于：
- 没有任何先验且需要全局覆盖搜索
- 目标已部分可见并具有可靠遮挡几何

## Input Requirements
- 至少一种位置、区域、方向、图像边缘或跟踪先验
- 当前相机位姿和相机模型
- 先验参考坐标系
- 搜索历史

## Strategy Parameters
- `prior_radius`
- `candidate_distribution`: `hemisphere | ring | grid | directional`
- `candidate_count`
- `direction_bias_weight`
- `look_at_mode`
- `uncertainty_scale`

## Execution Logic
1. 融合目标位置、区域、方向和跟踪先验。
2. 根据先验不确定性计算局部搜索范围。
3. 在先验周围生成候选视角。
4. 使用方向提示重新加权候选分布。
5. 去除历史重复或失败视角。
6. 输出候选、先验来源和候选概率。
7. 交由统一 Viewpoint Validator 完成最终验证和排序。

## Output
返回：
- candidate_viewpoints
- expected_visibility_gain
- expected_information_gain
- source = prior_guided_search
- evidence
- search_progress

## Failure Handling
- `TARGET_PRIOR_INVALID`: 目标先验缺失或不可解析
- `FRAME_TRANSFORM_FAILED`: 先验无法转换到输出坐标系
- `VIEWPOINT_GENERATION_FAILED`: 局部候选生成失败
- `NO_VALID_VIEWPOINT`: 候选均未通过验证

本文件只描述策略语义；候选生成、射线投射、视场计算、几何分析和机器人验证实现由用户接入。
