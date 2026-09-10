---
id: press.force_controlled
name: force_controlled_press
display_name: Force-Controlled Press
parent_skill: press
version: 1.0.0

description: >
  沿明确按压方向建立接触，并闭环调节法向力至目标按压力，在最大力和最大行程约束内完成保持与回撤。

input_schema: strategies/force_controlled_press/force_controlled_press.schema.yaml
implementation: strategies/force_controlled_press/force_controlled_press.py
---

# Force-Controlled Press Strategy

## Name
Force-Controlled Press

## Description
沿按压方向低速建立接触，然后闭环调节工具法向力至目标值。该策略以按压力为主要控制目标，并使用最大行程、最大力、稳定时间和控制稳定性约束保护执行。

## When To Use
适用于：
- 目标行程未知或位置存在一定误差
- 目标具有柔顺性
- 任务明确要求目标按压力
- 可用可靠力/力矩、关节力矩或触觉反馈

不适用于：
- 力反馈不可用或未校准
- 目标力超过工具或目标安全范围
- 按压方向不确定
- 接触控制器无法保证稳定性

## Input Requirements
- 有效按压方向和参考坐标系；表面法向量必须朝外，执行方向取其反向
- 正数 `target_force`
- 可用并已校准的力或等效触觉反馈
- `target_force <= maximum_force`
- 合法最大行程、接触阈值和控制参数

## Strategy Parameters
- `target_force`: 目标法向按压力
- `contact_force`: 建立接触的力阈值
- `force_ramp_rate`: 目标力爬升速率
- `force_control_gain`: 力控制增益或控制器配置引用
- `force_settle_time`: 目标力达到后的稳定等待时间
- `allow_force_overshoot`: 是否允许有限瞬态超调
- `overshoot_limit`: 最大允许超调量

## Execution Logic
1. 合并请求参数和默认值。
2. 解析并单位化按压方向。
3. 验证力传感器状态、零偏和接触控制器可用性。
4. 沿按压方向低速移动并检测初始接触。
5. 检测到接触后切换到力控制。
6. 按 `force_ramp_rate` 增加目标法向力。
7. 持续限制最大力、最大行程、横向力和超时。
8. 达到目标力后保持指定时间。
9. 采集目标状态和力曲线验证结果。
10. 退出力控制并沿原轴安全回撤。

## Output
返回：
- contact_detected
- target_force_reached
- peak_force
- final_force
- travel_distance
- target_actuated
- verified
- retracted
- evidence
- execution_time

## Failure Handling
- `SENSOR_DATA_UNAVAILABLE`: 力或触觉反馈不可用
- `FORCE_SENSOR_NOT_CALIBRATED`: 传感器未校准或零偏异常
- `CONTACT_NOT_DETECTED`: 最大行程内未检测到接触
- `TARGET_FORCE_NOT_REACHED`: 未达到目标按压力
- `FORCE_LIMIT_EXCEEDED`: 力超过安全上限
- `FORCE_CONTROL_UNSTABLE`: 力控制振荡或不稳定
- `TRAVEL_LIMIT_EXCEEDED`: 达到最大行程
- `TARGET_NOT_ACTUATED`: 目标未触发
- `RETRACT_FAILED`: 回撤失败
- `PRESS_TIMEOUT`: 执行超时

MuJoCo 实现根据接触约束力与穿透估算反馈逐步建立目标力；真实机器人必须通过兼容接口提供已标定的闭环力控制和目标状态证据。
