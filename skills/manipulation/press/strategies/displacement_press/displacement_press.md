--
id: press.displacement
name: displacement_press
display_name: Displacement Press
parent_skill: press
version: 1.0.0

description: >
  沿明确按压方向执行指定深度或行程的受限局部接触运动，并使用最大力和最大行程保护执行。

input_schema: strategies/displacement_press/displacement_press.schema.yaml
implementation: strategies/displacement_press/displacement_press.py
---

# Displacement Press Strategy

## Name
Displacement Press

## Description
沿解析后的按压方向执行指定按压深度或总行程。该策略以位置或位移为主要控制目标，并持续监测接触、力、行程、超时和局部安全状态。

## When To Use
适用于：
- 目标按钮或机构行程已知
- 预按压位姿重复精度较好
- 任务主要要求达到指定深度
- 力反馈仅作为安全限制或接触证据

不适用于：
- 目标位置误差较大或表面高度未知
- 需要稳定保持目标按压力
- 目标柔顺性很高
- 无法可靠设置最大力和最大行程

## Input Requirements
- 有效按压方向和参考坐标系；表面法向量必须朝外，执行方向取其反向
- `press_depth` 或 `travel_distance`
- 正数 `maximum_travel`
- 合法按压速度和最大力
- 接触控制器和局部运动接口可用

## Strategy Parameters
- `press_depth`: 从接触点开始的目标按压深度
- `travel_distance`: 从预按压位姿开始的总运动距离
- `stop_on_contact`: 检测到接触后是否立即停止
- `post_contact_depth`: 接触后继续按压的附加位移
- `motion_profile`: `constant_velocity | trapezoidal | smooth`

## Execution Logic
1. 合并请求参数和默认值。
2. 解析并单位化按压方向。
3. 校验目标位移不超过 `maximum_travel`。
4. 从预按压位姿沿按压方向低速前进。
5. 持续监测接触、法向力、行程和局部碰撞状态。
6. 根据 `stop_on_contact`、`press_depth` 或 `post_contact_depth` 决定停止位置。
7. 保持指定时间并采集验证证据。
8. 需要时沿原轴回撤至安全位姿。
9. 返回接触、位移、峰值力、目标状态和回撤结果。

## Output
返回：
- contact_detected
- target_depth_reached
- peak_force
- travel_distance
- target_actuated
- verified
- retracted
- evidence
- execution_time

## Failure Handling
- `INVALID_PRESS_DIRECTION`: 按压方向非法
- `CONTACT_NOT_DETECTED`: 允许行程内未检测到接触
- `FORCE_LIMIT_EXCEEDED`: 力超过安全上限
- `TRAVEL_LIMIT_EXCEEDED`: 位移超过最大行程
- `TARGET_NOT_ACTUATED`: 目标未触发
- `RETRACT_FAILED`: 回撤失败
- `PRESS_TIMEOUT`: 执行超时
- `PRESS_EXECUTION_ERROR`: 控制器或运动异常

MuJoCo 实现使用固定轴短步 IK、目标接触监测和原路回撤；真实机器人通过兼容的局部 Press Controller 接口接入。
