---
id: locate.approach_point
name: approach_point_locate
display_name: Approach Point Locate
parent_skill: locate
version: 1.0.0

description: >
  根据已知最终目标位姿、接近方向和安全约束，在目标附近生成可达且无碰撞的预靠近位姿。

input_schema: strategies/approach_point_locate/approach_point_locate.schema.yaml
implementation: strategies/approach_point_locate/approach_point_locate.py
---

# Approach Point Locate Strategy

## Name
Approach Point Locate

## Description
根据最终操作目标位姿、接近方向和距离生成初始靠近点；初始点无效时，可在有限邻域中生成并排序候选靠近位姿。

## When To Use
适用于：
- 最终操作目标位姿已经可用
- 需要为 Move、VLA、ACT 或复杂操作提供准备位姿
- 需要保持安全接近距离
- 需要检查候选点可达性和碰撞风险

不适用于：
- 最终目标位姿尚未定位
- 需要直接检测目标物体
- 接近方向和距离无法确定

## Input Requirements
- `reference.pose` 或 `strategy_params.target_pose`
- 有效 `approach_direction`
- 正数 `approach_distance`
- 方向坐标系可解析

## Strategy Parameters
- `target_pose`
- `approach_direction`: vector | semantic | surface_normal
- `approach_distance`
- `direction_frame`
- `orientation_mode`: `keep_target | align_with_direction | specified`
- `search_radius`
- `candidate_count`
- `minimum_clearance`

## Execution Logic
1. 获取并校验最终目标位姿。
2. 解析接近方向及其参考坐标系。
3. 按接近距离生成初始靠近位姿。
4. 初始点无效时，在受限邻域内生成候选点。
5. 计算候选点距离、间隙和任务方向一致性。
6. 输出排序后的候选靠近位姿。
7. 交由统一 Pose Validator 完成可达性与碰撞验证。

## Output
返回：
- target_pose = selected approach pose
- final_target_pose
- candidates
- confidence / validity score
- source = approach_point

## Failure Handling
- `REFERENCE_INVALID`: 最终目标位姿不可用
- `DIRECTION_UNRESOLVED`: 接近方向无法解析
- `INVALID_OFFSET`: 接近距离或方向参数非法
- `NO_VALID_CANDIDATE`: 未生成满足约束的靠近点
- `POSE_VALIDATION_FAILED`: 所有候选点不可达或发生碰撞

本文件只描述策略语义；方向解析、候选生成、可达性和碰撞实现由用户接入。
