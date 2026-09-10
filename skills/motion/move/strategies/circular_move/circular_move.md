---
id: move.circular
name: circular_move
display_name: Circular Move
parent_skill: move
version: 1.0.0

description: >
  在笛卡尔空间中根据圆弧中间点、旋转中心、旋转轴、
  半径或旋转角度生成连续的末端圆弧运动路径。

input_schema: strategies/circular_move/circular_move.schema.yaml
implementation: strategies/circular_move/circular_move.py
---

# Circular Move Strategy

## Name
Circular Move

## Description
根据明确的圆弧几何定义生成末端笛卡尔圆弧路径。Circular Move 是工艺或机构约束路径，不是通用避障策略。

## When To Use
适用于：
- 用户明确要求圆弧运动
- 提供 `via_point`
- 提供 `arc_center + arc_axis + arc_angle`
- 开门、转动机构、沿圆形边缘运动
- 工艺过程要求已知圆弧

不得使用于：
- 仅因为环境中存在障碍物
- 缺少有效圆弧几何参数
- 可以任意改变路径的普通 transit 避障

## Arc Definition
支持两种互斥定义：

1. `via_point`
   - start pose + via point + goal pose 定义圆弧

2. `center_axis`
   - arc_center
   - arc_axis
   - arc_angle
   - 可选 radius 或由起点自动计算

## Strategy Parameters
- `arc_definition_mode`: `via_point | center_axis`
- `via_point`
- `arc_center`
- `arc_axis`
- `arc_angle`
- `cartesian_step`
- `orientation_mode`: `fixed | interpolate | tangent | radial`
- `constraints.keep_end_effector_orientation`: 默认 `false`。为 true 时，圆弧全程保持起始末端姿态；为 false 时使用目标姿态。
- `min_path_fraction`

## Execution Logic
1. 校验圆弧定义完整且几何上有效。
2. 将所有几何量转换到统一规划坐标系。
3. 计算圆心、半径、旋转平面和方向。
4. 采样圆弧 Pose，并为每个点求解连续 IK。
5. 检查轨迹连续性、奇异点、关节限制和碰撞。
6. hard 圆弧不可行时直接失败，不得静默改为其他路径。

当 soft 圆弧发生碰撞时，避障 fallback 仍会保持该起始姿态；无可行路径时失败而不会擅自旋转末端。

## Failure Handling
- `INVALID_ARC_DEFINITION`: 圆弧参数缺失或几何退化
- `FRAME_TRANSFORM_FAILED`: 几何量无法转换到规划坐标系
- `IK_FAILED`: 圆弧中间点不存在连续 IK
- `PATH_CONSTRAINT_INFEASIBLE`: hard 圆弧路径不可行
- `COLLISION_DETECTED`: 圆弧轨迹发生碰撞

本文件只描述策略语义；圆弧计算、IK、规划和控制实现由用户接入。
