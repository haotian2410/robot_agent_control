---
name: press
description: >
  在机械臂已到达预按压位姿后，沿明确按压方向执行受限的接触建立、位移或力控制、
  保持、释放和结果验证，不负责目标检测、按压点定位或自由空间运动规划。
metadata:
  display_name: Press
  version: 2.0.0
  entrypoint: skill.py
  input_schema: schema.yaml
---

# Press Skill

## Name
Press

## Description
在机械臂已经到达目标附近的预按压位姿后，控制末端执行短距离、受约束的接触按压动作。Press Skill 负责接触检测、按压方向约束、位移或力控制、保持、释放、回撤和按压结果验证，不负责目标检测、按压点定位、全局路径规划或自由空间接近。

Press Skill 将按压执行拆分为三个阶段：
- Press Strategy：定义使用位移控制还是力控制完成按压
- Contact Safety Pipeline：检查方向、传感器、行程、力、控制器和局部运动安全条件
- Press Verification：根据目标状态、力/位移曲线、按钮反馈、触觉或视觉证据判断按压是否成功

## Purpose
用于：
- 按压按钮、开关、触摸区域或机械触发件
- 沿表面法向执行受限接触运动
- 按指定深度或位移完成已知行程按压
- 按指定目标力完成未知行程或柔顺目标按压
- 在接触后保持指定时间
- 按压完成后沿原方向安全回撤
- 验证按钮、开关或目标状态是否发生预期变化

不负责：
- 按钮或目标检测
- 按压点、表面法向或预按压位姿生成
- 机械臂从远处移动到目标附近
- 全局路径规划和绕障
- 移动或移除遮挡物
- 抓取、释放、插入或旋拧操作
- 高层任务逻辑规划

说明：
Press Skill 与 Move Skill 的边界为：Move Skill 负责自由空间到达预按压位姿以及按压完成后的远距离撤离；Press Skill 只负责从预按压位姿开始的短距离、局部、受限接触段及可选原路回撤。Press Skill 不应为了避障静默改变按压方向。

当前 MuJoCo 适配器通过固定轴短步 IK 执行局部运动，使用目标接触与穿透深度估算法向力，并在异常时按原关节路径回撤。真实硬件适配器必须提供等价的受限局部控制接口，并保留相同的力、行程、碰撞和回撤语义。

## When To Use
当按压点、按压方向和预按压位姿已经确定，机械臂已到达目标附近，需要执行实际接触按压时调用该技能。

典型场景：
- 按下已定位的实体按钮
- 触发带明确行程的机械开关
- 以指定力按压柔性表面
- 保持按压一段时间后释放
- 根据按钮状态或视觉反馈验证是否触发成功

不应调用的情况：
- 按压点或方向尚未确定
- 机械臂未到达预按压位姿
- 需要大范围移动或绕过障碍物
- 目标需要旋转、拨动、滑动或抓取
- 力控策略需要的传感器不可用
- 当前机器人或接触控制器处于不安全状态

## Press Strategy Architecture

完整按压执行方案由 Press Strategy、接触安全流程和结果验证组成：

```text
Press Result = Press Strategy + Contact Safety Pipeline + Press Verification
```

策略结构：

```text
Position-Based Contact
└── Displacement Press

Force-Based Contact
└── Force-Controlled Press
```

统一执行流程：

```text
Request Validation
→ Runtime State Acquisition
→ Strategy Selection
→ Precondition and Direction Validation
→ Establish Contact
→ Execute Displacement or Force Profile
→ Hold
→ Verify Target Actuation
→ Retract Along Press Axis (optional)
→ Return Structured Result
```

## Press Strategies

### Displacement Press

沿明确按压方向运动指定距离或到达指定按压深度，并使用最大力和最大行程约束保护执行过程。

适用于：
- 按钮或机构行程已知
- 目标刚性较高且位置重复性较好
- 主要成功条件是达到指定按压深度
- 可以使用力限制作为安全保护，但不依赖闭环目标力

特点：
- 执行逻辑确定且易于复现
- 必须限制最大行程和最大力
- 可在接触后继续指定附加位移
- 不适合目标位置和行程高度不确定的柔性场景

详细策略：
Displacement Press

### Force-Controlled Press

沿按压方向建立接触，并闭环调节末端法向力至目标按压力，在指定保持时间后释放或回撤。

适用于：
- 目标行程未知或存在位置误差
- 目标具有柔顺性
- 任务明确要求目标按压力
- 系统具有可靠的力/力矩或等效触觉反馈

特点：
- 对目标位置误差和表面柔顺性更鲁棒
- 必须设置最大力、最大行程和控制稳定性限制
- 接触检测与目标力建立是两个独立阶段
- 力传感器不可用或异常时不得执行

详细策略：
Force-Controlled Press

## Strategy Selection Policy

Press Skill 根据显式策略、按压控制模式、目标力、按压深度、传感器能力和运行时状态选择按压策略。

选择优先级：

1. 用户或上层任务显式指定的合法策略优先级最高。
2. `press.control_mode=force` 时选择 Force-Controlled Press。
3. `press.control_mode=displacement` 时选择 Displacement Press。
4. 自动模式下，提供 `target_force` 且运行时存在有效力反馈时选择 Force-Controlled Press。
5. 自动模式下，提供 `press_depth` 或 `travel_distance` 时选择 Displacement Press。
6. 同时提供目标力和按压深度时，目标力用于主控制、按压深度作为最大行程保护；优先选择 Force-Controlled Press。
7. 力控策略缺少有效力反馈时返回 `SENSOR_DATA_UNAVAILABLE`，不得静默降级为位移控制。
8. 位移策略缺少按压深度或合法行程时返回 `INVALID_REQUEST`。
9. 所有策略必须验证按压方向、最大力、最大行程和当前接触控制器状态。
10. 按压方向为 hard 任务约束，不得为避障或可达性静默改变。
11. 达到最大力、最大行程或超时时必须停止前进并按安全策略回撤。
12. 重试只能在目标状态明确未触发、错误可恢复且回撤成功后执行。

默认选择逻辑：

```text
press.control_mode = force
→ Force-Controlled Press

press.control_mode = displacement
→ Displacement Press

auto + target_force available + force feedback available
→ Force-Controlled Press

auto + press_depth available
→ Displacement Press

force requested + no valid force feedback
→ SENSOR_DATA_UNAVAILABLE

no target force or press depth
→ INVALID_REQUEST
```

职责边界：
- Agent：构造按压目标、方向、控制模式、约束和验证要求
- Location Skill：生成按压点、表面法向和预按压位姿
- Move Skill：到达预按压位姿，并执行非接触的自由空间移动
- Strategy Selector：根据确定性规则选择 Press Strategy
- Press Strategy：执行短距离接触、位移或力控制、保持和局部回撤
- Press Validator：检查方向、状态、力、行程和按压结果证据
- Safety System：可以立即停止并否决任何不安全的接触动作

## Input

### Target
按压目标及其几何信息。

格式：

```yaml
target:
  type: press_pose | surface_point | button
  object_id: optional string
  press_pose: optional pose
  press_point: optional point
  press_direction: optional vector
  surface_normal: optional vector
  expected_state_change: optional object
```

`press_direction` 直接表示实际运动方向。`surface_normal` 统一表示表面向外法向量；当 `direction_source=surface_normal` 时，Press Skill 使用 `-surface_normal` 作为实际按压方向。Press Skill 不负责通过图像估计该方向。

### Press
按压策略和执行意图。

支持：
- strategy: auto | displacement_press | force_controlled_press
- control_mode: auto | displacement | force
- operation: press | verify_only
- direction_source: target | surface_normal | specified
- retract_after_press
- verification_mode: auto | force_profile | position | digital_io | visual | combined

格式：

```yaml
press:
  strategy: auto
  control_mode: auto
  operation: press
  direction_source: target
  retract_after_press: true
  verification_mode: auto
```

### Constraints
接触执行和安全约束。

支持：
- maximum_force
- maximum_travel
- contact_threshold
- approach_speed
- press_speed
- retract_speed
- hold_time
- verify_press
- retry_count
- timeout
- force_tolerance
- position_tolerance
- require_contact
- emergency_retract

格式：

```yaml
constraints:
  maximum_force: 30.0
  maximum_travel: 0.03
  contact_threshold: 2.0
  press_speed: 0.01
  hold_time: 0.5
  verify_press: true
  retry_count: 0
  timeout: 10.0
  emergency_retract: true
```

### Strategy Parameters
由所选 Strategy 独立解释：
- Displacement Press：`press_depth`、`travel_distance`、`stop_on_contact`、`post_contact_depth`
- Force-Controlled Press：`target_force`、`force_ramp_rate`、`force_control_gain`、`contact_force`

### Runtime Context
由机器人系统内部提供：
- Current End-Effector Pose
- Current Joint State
- Force / Torque State
- Tactile State
- Contact Controller State
- Robot Model
- Planning Scene / Local Collision State
- Tool Geometry and Compliance
- Target State Sensor / Digital IO
- Camera Observation for Verification
- Safety State

## Output

### Success
执行是否成功。

### Status
状态：
- completed
- failed
- timeout
- stopped

### Selection
实际选择结果：
- press_strategy
- selection_reason
- retry_used

### Press Result
执行结果：
- contact_detected
- target_force_reached
- target_depth_reached
- peak_force
- final_force
- travel_distance
- hold_time
- target_actuated
- verified
- retracted
- final_pose
- attempts
- execution_time

### Error
失败信息：
- error_code
- error_message
- failed_stage
- recoverable
- recommended_action

## Failure Handling

### Invalid Request

Description:
按压点、方向、控制模式或策略参数缺失或非法。

Possible Causes:
- 缺少 press direction 或 surface normal
- 位移策略缺少 press depth
- 力控策略缺少 target force
- 最大力、最大行程或速度参数非法
- 显式策略与 control mode 冲突

Recommended Action:
- 根据主 schema 和策略 schema 修正输入
- 请求 Location Skill 补充按压点和表面法向
- 明确指定控制模式及其目标参数
- 检查安全限制是否合理

### Press Controller Not Ready

Description:
接触控制器、机器人或安全系统当前不允许执行按压。

Possible Causes:
- 控制器未使能
- 力传感器未归零
- 当前末端状态未知
- 机器人处于急停或保护停止
- 工具模型与实际工具不一致

Recommended Action:
- 停止执行并检查控制器和安全状态
- 重新初始化接触控制器和传感器
- 更新工具模型
- 仅在状态安全时重新执行

### Direction Invalid

Description:
按压方向无法解析、不是有效单位方向，或与当前目标几何关系不一致。

Possible Causes:
- 坐标系未定义
- 表面法向方向反向
- 方向向量接近零
- 目标位姿和方向时间戳不一致

Recommended Action:
- 重新获取按压点和表面法向
- 明确方向参考坐标系
- 验证方向符号和单位化结果
- 不得由 Press Skill 静默选择任意方向

### Contact Not Detected

Description:
在允许行程内未检测到目标接触。

Possible Causes:
- 预按压位姿距离目标过远
- 按压方向错误
- 目标位置发生变化
- 接触阈值设置不合理
- 传感器不可用

Recommended Action:
- 停止继续前进并回撤
- 请求 Location Skill 重新定位按压点
- 检查按压方向和接触阈值
- 由 Move Skill重新到达有效预按压位姿

### Force Limit Exceeded

Description:
按压过程中法向力或其他受监测力超过安全上限。

Possible Causes:
- 最大力设置不合理
- 目标机构卡住或不可按压
- 按压方向偏离表面法向
- 力控不稳定
- 传感器异常

Recommended Action:
- 立即停止前进并执行安全回撤
- 检查目标机构和工具状态
- 调整控制参数或按压方向
- 不得在未排除故障时自动重试

### Travel Limit Exceeded

Description:
在达到目标力或目标状态前已达到最大允许行程。

Possible Causes:
- 目标位置估计错误
- 目标行程大于预期
- 力传感器未检测到接触
- 目标并非可按压机构

Recommended Action:
- 停止并回撤到预按压位姿
- 检查目标类型和实际行程
- 请求重新定位或更新目标先验
- 不得继续扩大行程绕过安全限制

### Target Not Actuated

Description:
完成按压动作后，目标未产生预期状态变化。

Possible Causes:
- 按压深度或按压力不足
- 按压位置偏离有效区域
- 目标机构故障
- 验证信号延迟或不可用

Recommended Action:
- 在允许时调整目标力、深度或保持时间
- 请求重新定位按压点
- 检查按钮、数字输入或视觉验证信号
- 达到重试次数后返回失败

### Retract Failed

Description:
按压完成或异常停止后未能沿安全路径回撤。

Possible Causes:
- 控制器异常
- 工具被目标卡住
- 动态障碍进入局部区域
- 回撤方向或距离非法

Recommended Action:
- 停止其他机械臂动作
- 保持或降低接触力至安全水平
- 请求人工或上层恢复流程
- 更新现场状态后再规划恢复运动

### Execution Error

Description:
按压执行阶段发生控制器、传感器、通信或硬件异常。

Possible Causes:
- 接触控制器异常
- 力/力矩传感器故障
- 通信中断
- 执行超时
- 安全系统触发停止

Recommended Action:
- 保存当前机器人、接触和传感器状态
- 安全停止并检查控制器、传感器和通信
- 清除故障后重新初始化
- 仅在错误可恢复且已安全回撤时重新执行
