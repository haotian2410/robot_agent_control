---
id: grasp.default
name: default_grasp
display_name: Default Grasp
parent_skill: grasp
version: 1.0.0

description: >
  使用兼容的开合式夹爪执行标准张开、闭合、接触、夹持、验证和保持流程。

input_schema: strategies/default_grasp/default_grasp.schema.yaml
implementation: strategies/default_grasp/default_grasp.py
---

# Default Grasp Strategy

## Name
Default Grasp

## Description
使用普通平行夹爪或兼容的开合式夹爪执行标准抓取。该策略假设机械臂已经到达有效抓取位姿，仅控制夹爪并验证抓取结果。

## When To Use
适用于：
- 机械臂已经到达目标抓取位姿
- 目标位于夹爪指间
- 使用平行夹爪、两指夹爪或兼容开合式夹爪
- 抓取过程不需要特殊指序列、吸附或复杂力控
- 可以使用位置、力、触觉或目标存在证据验证结果

不适用于：
- 需要吸盘、磁吸或多指灵巧抓取
- 需要重新生成抓取姿态
- 需要在抓取过程中移动机械臂
- 目标尺寸明显超出夹爪工作范围

## Input Requirements
- 有效夹爪模型、限制和控制器状态
- 合法的 `open_width` 和 `close_width`
- 合法的 `grasp_force` 和 `hold_force`
- 需要验证时存在至少一种可用验证证据
- `require_pose_alignment=true` 时存在目标抓取位姿和当前末端位姿

## Strategy Parameters
- `open_width`: 抓取前夹爪张开宽度
- `close_width`: 夹爪目标闭合宽度
- `close_speed`: 归一化闭合速度
- `grasp_force`: 抓取阶段目标夹持力
- `hold_force`: 抓取成功后的保持力
- `contact_threshold`: 接触检测阈值
- `position_tolerance`: 目标闭合宽度允许误差
- `force_tolerance`: 目标夹持力允许误差
- `settle_time`: 闭合或加力后的稳定等待时间

## Execution Logic
1. 合并请求参数和策略默认值。
2. 验证夹爪宽度、速度、力和传感器要求。
3. `operation=verify_only` 时跳过动作，仅采集并验证当前夹持证据。
4. 将夹爪张开至 `open_width`。
5. 按 `close_speed` 闭合夹爪并监测接触。
6. 检测到接触后施加 `grasp_force`。
7. 等待 `settle_time` 并采集位置、力、触觉和目标存在证据。
8. 执行统一抓取验证。
9. 成功且 `hold_on_success=true` 时维持 `hold_force`。
10. 返回抓取结果和完整证据。

## Output
返回：
- grasped
- contact_detected
- final_width
- final_force
- holding
- verified
- verification_method
- slip_detected
- evidence
- execution_time

## Failure Handling
- `INVALID_GRIPPER_PARAMETERS`: 宽度、速度或力参数非法
- `GRIPPER_NOT_READY`: 夹爪或控制器不可执行
- `GRIPPER_OPEN_FAILED`: 抓取前张开失败
- `CONTACT_NOT_DETECTED`: 闭合后未检测到目标接触
- `FORCE_LIMIT_EXCEEDED`: 请求或实际夹持力超过限制
- `GRASP_FORCE_NOT_REACHED`: 未达到目标夹持力
- `GRASP_VERIFICATION_FAILED`: 抓取验证失败
- `OBJECT_SLIPPED`: 检测到目标滑移
- `GRASP_TIMEOUT`: 执行超时
- `GRIPPER_EXECUTION_ERROR`: 夹爪执行异常

当前 MuJoCo 控制器适配位于 `control/gripper_controller.py`，接触与结果验证位于 `validators/grasp_validator.py`。真实硬件力传感器、触觉和视觉验证仍需在迁移硬件运行时后接入。
