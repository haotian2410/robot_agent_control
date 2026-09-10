---
id: locate.relative_point
name: relative_point_locate
display_name: Relative Point Locate
parent_skill: locate
version: 1.0.0

description: >
  根据参考位姿和语义或笛卡尔偏移生成目标位姿，适用于末端、物体或命名坐标系附近的自定义定位。

input_schema: strategies/relative_point_locate/relative_point_locate.schema.yaml
implementation: strategies/relative_point_locate/relative_point_locate.py
---

# Relative Point Locate Strategy

## Name
Relative Point Locate

## Description
根据末端、对象、显式 Pose 或命名坐标系的参考位姿，将语义方向或笛卡尔偏移转换为统一坐标系下的目标位姿。

## When To Use
适用于：
- 在机械臂末端左、右、上、下、前、后方生成目标点
- 在目标物体附近生成偏移点
- 根据明确 x、y、z 偏移生成目标位姿
- 为扫描、观察、接近或临时运动生成参考目标点

不适用于：
- 无有效参考位姿或坐标系
- 需要从视觉中检测未知目标
- 语义方向没有定义参考坐标系

## Input Requirements
- `reference.type`
- 可解析的 reference id、frame 或 pose
- `offset.mode`
- 语义方向和距离，或笛卡尔 x/y/z 偏移

## Strategy Parameters
- `reference_type`
- `reference_id`
- `reference_pose`
- `offset_mode`
- `offset`
- `default_offset`
- `semantic_direction_mapping`
- `orientation_mode`: `keep_reference | align_with_reference | face_reference | specified`
- `search_radius`

## Execution Logic
1. 获取参考位姿和参考坐标系。
2. 解析语义方向或笛卡尔偏移。
3. 将偏移转换到参考坐标系。
4. 组合参考位姿和偏移生成初始目标位姿。
5. 根据 orientation_mode 生成目标姿态。
6. 必要时在有限邻域内生成修正候选。
7. 交由统一 Pose Validator 完成最终验证。

## Output
返回：
- target_pose
- reference_pose
- resolved_offset
- confidence / validity score
- source = relative_point

## Failure Handling
- `REFERENCE_INVALID`: 参考位姿或坐标系不可用
- `SEMANTIC_DIRECTION_AMBIGUOUS`: 语义方向无法映射
- `INVALID_OFFSET`: 偏移缺失或数值非法
- `FRAME_TRANSFORM_FAILED`: 偏移或参考位姿转换失败
- `NO_VALID_CANDIDATE`: 初始点及修正候选均无效
- `POSE_VALIDATION_FAILED`: 目标点不可达或发生碰撞

本文件只描述策略语义；坐标变换、方向映射、候选修正和验证实现由用户接入。
