---
name: grasp
description: >
  控制机器人末端夹爪对已到达抓取位置的目标执行张开、闭合、夹持、保持和抓取验证，
  不负责目标检测、抓取姿态生成、机械臂移动或路径规划。
metadata:
  display_name: Grasp
  version: 1.0.0
  entrypoint: skill.py
  input_schema: schema.yaml
---

# Grasp Skill

## Name
Grasp

## Description
控制机器人末端夹爪抓取指定目标物体。Grasp Skill 仅负责夹爪动作执行、接触判断、夹持力建立、抓取保持和结果验证，不负责物体检测、目标位姿估计、抓取姿态生成或机械臂移动。

## Current Implementation

当前 Windows/MuJoCo 实现使用 `control/gripper_controller.py` 驱动 Robotiq 2F-85：

- `skill.py` 负责运行时上下文、策略调度、有限重试和结构化结果。
- `utils/validation.py` 负责请求归一化和参数范围检查。
- `validators/grasp_validator.py` 负责控制器状态、夹爪限制和抓取证据验证。
- `strategies/default_grasp/default_grasp.py` 执行张开、闭合、接触检测与保持。
- 有效接触限定为目标物体与左右指垫的接触，避免把掌部或环境碰撞误判为抓取。
- 当前仿真验证使用接触、夹爪宽度和目标存在证据；力值是控制命令语义值，不是标定后的真实力传感器测量。

测试入口位于 `tests/grasp_skill`。自动测试会先把机械臂置于抓取位姿；这属于测试准备，不改变 Grasp Skill“不负责移动机械臂”的职责边界。

Grasp Skill 将抓取执行拆分为三个阶段：
- Grasp Strategy：定义夹爪采用何种抓取执行流程
- Gripper Validation Pipeline：检查夹爪状态、参数范围、目标兼容性和控制器可用性
- Grasp Verification：根据位置、力、接触、目标在夹爪内状态或滑移信息判断是否抓取成功

## Purpose
用于：
- 打开夹爪至抓取前宽度
- 闭合夹爪并接触目标物体
- 对目标物体施加指定夹持力
- 抓取成功后保持夹持状态
- 验证目标是否被稳定抓取
- 在允许时执行有限次数的抓取重试
- 为后续 Move、Place、Insert 或其他操作技能提供已夹持状态

不负责：
- 物体检测和目标实例选择
- 目标位姿估计
- 抓取姿态搜索或抓取生成
- 机械臂接近、对齐、抬升和撤离
- 自由空间或接触路径规划
- 放置和释放目标物体
- 高层任务逻辑规划

说明：
Grasp Skill 默认假设机械臂已经到达有效抓取位姿。夹爪参数和抓取结果可以使用控制器、位置传感器、力传感器、触觉传感器或外部视觉进行验证，但这些验证不生成新的机械臂目标位姿。

## When To Use
当机械臂已经到达目标抓取位姿，需要控制夹爪完成实际抓取时调用该技能。

典型场景：
- 抓取桌面上的目标物体
- 搬运任务前夹持物体
- 装配任务前获取零件
- 获取工具或操作对象
- 抓取失败后在当前位置重新夹紧
- 检查夹爪是否已稳定持有目标

不应调用的情况：
- 目标抓取位姿尚未生成
- 机械臂尚未到达抓取位置
- 需要移动机械臂寻找目标或改变抓取姿态
- 只需要打开夹爪释放目标
- 当前夹爪或控制器处于故障状态

## Grasp Strategy Architecture

完整抓取执行方案由 Grasp Strategy、统一验证流程和结果验证组成：

```text
Grasp Result = Grasp Strategy + Gripper Validation Pipeline + Grasp Verification
```

当前策略结构：

```text
Standard Grasp
└── Default Grasp
```

执行流程：

```text
Request Validation
→ Runtime State Acquisition
→ Strategy Selection
→ Precondition Validation
→ Open Gripper
→ Close Gripper and Detect Contact
→ Apply Grasp Force
→ Verify Grasp
→ Hold or Release According to Policy
→ Return Structured Result
```

## Grasp Strategies

### Default Grasp

使用普通平行夹爪执行标准张开、闭合、接触、夹持和验证流程。

适用于：
- 目标抓取位置已经确定
- 机械臂已经到达抓取位置
- 使用普通平行夹爪或兼容的开合式夹爪
- 无特殊抓取序列或复杂力控要求
- 可以通过位置、力、接触或目标存在状态验证抓取

特点：
- 执行流程简单且确定
- 支持宽度、速度、夹持力和保持力配置
- 支持接触检测和抓取结果验证
- 支持有限次数重试，但不自动改变机械臂抓取位姿

详细策略：
Default Grasp

## Strategy Selection Policy

Grasp Skill 根据显式策略、夹爪类型、目标信息、运行时能力和策略可用性选择抓取策略。

选择优先级：

1. 用户或上层任务显式指定的合法策略优先级最高。
2. 当前仅存在 `default_grasp`，自动模式默认选择 Default Grasp。
3. 显式策略缺少必需参数或与当前夹爪不兼容时返回 `INVALID_REQUEST`。
4. Strategy Selector 只选择抓取执行流程，不生成抓取姿态或机械臂目标位姿。
5. 所有执行请求必须通过统一 Grasp Validator；Selector 不能覆盖参数范围、夹爪状态和安全否决。
6. 重试只能在 `retry_count > 0`、错误可恢复且夹爪和机械臂状态安全时执行。
7. 重试不得静默移动机械臂或修改抓取位姿。
8. 验证失败且已达到重试次数时返回 `GRASP_VERIFICATION_FAILED` 或对应结构化错误。

默认选择逻辑：

```text
grasp.strategy = default_grasp
→ Default Grasp

grasp.strategy = auto
+ compatible parallel gripper available
→ Default Grasp

unsupported or incompatible strategy
→ INVALID_REQUEST
```

职责边界：
- Agent：理解抓取意图并构造目标、抓取配置、约束和策略参数
- Strategy Selector：根据确定性规则选择 Grasp Strategy
- Grasp Strategy：执行夹爪张开、闭合、接触和夹持流程
- Grasp Validator：检查执行前状态、参数范围和执行结果证据
- Grasp Verifier：根据位置、力、触觉、滑移或视觉证据判断抓取是否成功
- Move Skill：负责机械臂到达抓取位姿、抓取后抬升和撤离
- Safety System：可以否决任何不安全的夹爪动作

## Input

### Target
需要抓取的目标对象及可选先验。

格式：

```yaml
target:
  type: object | grasp_pose
  object_id: optional string
  category: optional string
  expected_width: optional float
  expected_weight: optional float
  grasp_pose: optional pose
```

`grasp_pose` 只用于验证当前机械臂是否已到达预期抓取位姿；Grasp Skill 不负责移动到该位姿。

### Grasp
抓取策略和执行意图。

支持：
- strategy: auto | default_grasp
- operation: grasp | regrasp | verify_only
- gripper_id
- verification_mode: auto | position | force | tactile | visual | combined

格式：

```yaml
grasp:
  strategy: auto
  operation: grasp
  gripper_id: default_gripper
  verification_mode: auto
```

### Constraints
抓取执行与安全约束。

支持：
- verify_grasp
- retry_count
- hold_on_success
- timeout
- maximum_force
- maximum_width
- minimum_width
- contact_required
- slip_check
- require_pose_alignment

格式：

```yaml
constraints:
  verify_grasp: true
  retry_count: 0
  hold_on_success: true
  timeout: 10.0
  maximum_force: 40.0
  contact_required: true
```

### Strategy Parameters
由所选 Strategy 独立解释的参数：
- open_width
- close_width
- close_speed
- grasp_force
- hold_force
- contact_threshold
- position_tolerance
- force_tolerance
- settle_time

### Runtime Context
由机器人系统内部提供：
- Current Gripper State
- Current End-Effector Pose
- Gripper Model and Limits
- Gripper Controller State
- Finger Position and Velocity
- Force / Torque State
- Tactile State
- Object Presence State
- Slip Detection State
- Tool and Payload State
- Robot Safety State

## Output

### Success
执行是否成功。

### Status
状态：
- completed
- failed
- timeout

### Selection
实际选择结果：
- grasp_strategy
- selection_reason
- retry_used

### Grasp Result
执行结果：
- grasped
- contact_detected
- final_width
- final_force
- holding
- verified
- verification_method
- slip_detected
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
输入字段缺失、参数非法、策略不存在，或参数超出夹爪和目标允许范围。

Possible Causes:
- 缺少目标或抓取配置
- `open_width`、`close_width` 或夹持力非法
- `close_width` 大于 `open_width`
- 显式策略与当前夹爪不兼容
- 验证模式需要的传感器不可用

Recommended Action:
- 根据主 schema 和策略 schema 修正输入
- 检查夹爪宽度和力限制
- 选择当前硬件支持的验证方式
- 将 strategy 设置为 `auto` 后重新选择

### Gripper Not Ready

Description:
夹爪、控制器或安全系统当前不允许执行抓取。

Possible Causes:
- 控制器未连接或未使能
- 夹爪未完成初始化
- 夹爪处于故障或急停状态
- 当前夹爪状态未知
- 工具配置与实际夹爪不一致

Recommended Action:
- 停止抓取并检查控制器状态
- 重新初始化或复位夹爪
- 更新工具和夹爪模型
- 仅在状态明确安全时重新执行

### Target Not Graspable

Description:
目标无法通过当前夹爪、参数和抓取位姿完成可靠抓取。

Possible Causes:
- 目标尺寸超出夹爪工作范围
- 目标抓取姿态不合理
- 夹爪与目标未对齐
- 目标表面或质量不适合当前夹持力
- 抓取约束过强

Recommended Action:
- 检查目标尺寸和夹爪兼容性
- 请求 Location 或 Grasp Planning 重新生成抓取位姿
- 调整夹持力、宽度或验证条件
- 更换合适的末端工具

### Contact Not Detected

Description:
夹爪闭合过程中未检测到预期目标接触。

Possible Causes:
- 目标不在夹爪指间
- 抓取位姿存在偏差
- 接触阈值设置不合理
- 目标在闭合前发生移动
- 传感器数据不可用

Recommended Action:
- 停止继续加力
- 检查目标和夹爪相对位置
- 调整接触阈值
- 请求重新定位或重新到达抓取位姿

### Force Limit Exceeded

Description:
抓取过程中测得或请求的夹持力超过安全限制。

Possible Causes:
- `grasp_force` 或 `hold_force` 设置过大
- 目标物体刚性或尺寸与预期不符
- 夹爪发生卡滞
- 力传感器异常

Recommended Action:
- 立即停止夹爪闭合或降低力
- 检查目标和夹爪是否受阻
- 修正安全力限制
- 仅在确认无损伤风险后重新执行

### Grasp Verification Failed

Description:
夹爪完成闭合和夹持后，验证证据不足以确认目标被稳定抓取。

Possible Causes:
- 最终夹爪宽度不符合预期
- 夹持力未达到要求
- 目标发生滑移
- 目标未处于夹爪内部
- 视觉、触觉或位置验证不可用

Recommended Action:
- 在允许时执行有限次数重新夹紧
- 调整抓取力、闭合宽度或稳定时间
- 请求重新定位抓取姿态
- 不得在未确认抓取成功时执行高速搬运

### Object Slipped

Description:
抓取成功后检测到目标发生滑移或夹持状态不稳定。

Possible Causes:
- 保持力不足
- 目标表面摩擦不足
- 目标重量超过预期
- 机械臂运动导致惯性载荷过大

Recommended Action:
- 安全停止机械臂运动
- 在允许时增加保持力或重新抓取
- 降低后续运动速度和加速度
- 检查夹爪指端材料和目标重量

### Execution Error

Description:
抓取执行阶段发生控制器、机械结构、通信或硬件异常。

Possible Causes:
- 夹爪控制器异常
- 夹爪机械结构阻塞
- 通信中断
- 传感器故障
- 执行超时

Recommended Action:
- 保存当前夹爪和机器人状态
- 安全停止并检查夹爪、控制器和通信
- 清除故障后重新初始化
- 仅在错误可恢复且状态安全时重新执行
