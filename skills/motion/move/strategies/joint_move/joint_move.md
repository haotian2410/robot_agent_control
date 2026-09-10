---
id: move.joint
name: joint_move
display_name: Joint Move
parent_skill: move
version: 1.0.0

description: >
  在关节空间中生成机械臂运动路径，适用于关节目标、
  命名状态以及无严格末端几何路径要求的自由空间转移。

input_schema: strategies/joint_move/joint_move.schema.yaml
implementation: strategies/joint_move/joint_move.py
---

# Joint Move Strategy

## Name
Joint Move

## Description
在关节空间生成点到点运动。目标可以是 Joint Target、Named State，或先通过 IK 转换得到目标关节状态的 Pose Target。

## When To Use
适用于：
- 明确的关节目标
- Home、Ready、Park 等预定义状态
- 自由空间中的 transit 运动
- 无严格末端几何路径要求
- 优先考虑效率、可达性或较短规划时间

不适用于：
- 必须保持末端直线的接近、退出、插入或按压
- 必须经过指定圆弧的任务

## Input Requirements
- `target.type`: `joint`、`named_state` 或 `pose`
- Pose Target 必须能够获得至少一个有效 IK 解
- 机器人当前关节状态必须可用

## Strategy Parameters
- `joint_target`: 可选的显式关节目标
- `named_state`: 可选的预定义状态名称
- `ik_solution_policy`: `nearest | minimum_motion | avoid_limits | custom`
- `interpolation`: `linear | cubic | quintic`
- `preserve_current_configuration`: 是否偏好保持当前构型分支

## Execution Logic
1. 将 Named State 解析为关节目标，或为 Pose Target 求解 IK。
2. 按策略选择有效 IK 解。
3. 构造关节空间路径要求。
4. 交由 Direct Planner 生成候选轨迹。
5. 对候选轨迹执行关节限制、奇异点和碰撞检查。
6. 直接路径不可行且约束允许时，交由 Collision-Free Planner 搜索替代关节轨迹。

## Output
返回后端无关的 Joint Path Request，包括：
- start_joint_state
- goal_joint_state
- interpolation
- velocity / acceleration constraints
- path_constraint

## Failure Handling
- `IK_FAILED`: Pose Target 不存在有效 IK 解
- `JOINT_LIMIT_VIOLATION`: 目标或轨迹超过关节限制
- `COLLISION_DETECTED`: 候选轨迹发生碰撞
- `PLANNING_FAILED`: 未找到有效关节轨迹

本文件只描述策略语义；具体 IK、插值、规划与控制实现由用户接入。
