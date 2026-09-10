---
id: move.linear
name: linear_move
display_name: Linear Move
parent_skill: move
version: 1.0.0

description: >
  在笛卡尔空间中生成末端直线路径，可保持或连续调整末端姿态。

input_schema: strategies/linear_move/linear_move.schema.yaml
implementation: strategies/linear_move/linear_move.py
---

# Linear Move

# Linear Move Strategy

## Name
Linear Move

## Description
在笛卡尔空间生成末端直线路径，用于保持接近方向、退出方向或接触运动的几何约束。

## When To Use
适用于：
- 用户明确要求直线运动
- `phase=approach | retreat | contact`
- 抓取前接近、抓取后抬升或撤离
- 放置下降
- 插入、拔出、按压
- 沿工具坐标系或其他参考坐标系进行相对位移

## Input Requirements
- Pose Target，或可转换为目标 Pose 的相对位移
- 有效参考坐标系
- 连续 IK 和中间路径可达

## Strategy Parameters
- `cartesian_step`: 笛卡尔离散步长
- `orientation_mode`: `fixed | interpolate | target_only`
- `constraints.keep_end_effector_orientation`: 默认 `false`。为 true 时，整条路径使用起始末端姿态；为 false 时使用目标姿态。
- `direction`: 可选的单位方向向量
- `distance`: 沿 direction 的运动距离
- `jump_threshold`: 关节跳变阈值
- `min_path_fraction`: 可接受的最小路径完成比例

## Path Constraint
- `soft`: 直接直线路径失败时，上层允许采用替代避障方案
- `hard`: 必须保持直线路径；失败时返回 `PATH_CONSTRAINT_INFEASIBLE`

注意：对于插入、按压等接触动作通常应使用 `hard`，不得静默绕行。

## Execution Logic
1. 将目标或相对位移转换到规划坐标系。
2. 生成连续笛卡尔直线采样点。
3. 为每个采样点求解连续 IK。
4. 检查路径比例、关节跳变、奇异点、关节限制和碰撞。
5. 根据 path_constraint 决定失败或合法 fallback。

默认的固定姿态约束同样会传递给 soft 路径的避障 fallback；若无固定姿态的可行避障路径，返回失败而不会擅自旋转末端。

## Failure Handling
- `FRAME_TRANSFORM_FAILED`: 坐标系转换失败
- `IK_FAILED`: 一个或多个中间点不存在连续 IK
- `PATH_CONSTRAINT_INFEASIBLE`: hard 直线路径不可行
- `COLLISION_DETECTED`: 直线路径存在碰撞风险
- `GOAL_TOLERANCE_EXCEEDED`: 执行后误差超过要求

本文件只描述策略语义；插值、IK、碰撞与控制实现由用户接入。
