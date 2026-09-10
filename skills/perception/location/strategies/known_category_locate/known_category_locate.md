---
id: locate.known_category
name: known_category_locate
display_name: Known Category Locate
parent_skill: locate
version: 1.0.0

description: >
  根据已知目标类别执行检测、实例选择和分割，并结合深度或点云估计目标位姿与操作点。

input_schema: strategies/known_category_locate/known_category_locate.schema.yaml
implementation: strategies/known_category_locate/known_category_locate.py
---

# Known Category Locate Strategy

## Name
Known Category Locate

## Description
根据已知目标类别执行目标检测、实例选择和实例分割，再结合深度图或目标区域点云估计三维目标位姿并生成操作点。

## When To Use
适用于：
- 已知目标类别但未知具体实例
- 场景中存在一个或多个同类对象
- 不具备精确三维模型
- 可使用类别规则生成操作点

不适用于：
- 类别未知或只提供模糊开放词汇描述
- 无 RGB、深度或点云观测
- 目标实例无法通过现有选择规则区分

## Input Requirements
- `target.category`、运行时类别先验或 `strategy_params.category`
- RGB 或 RGB-D 观测
- 相机内参和坐标变换
- 需要三维位姿时必须有深度或点云

## Strategy Parameters
- `category`
- `detector_model`
- `detection_threshold`
- `segmentation_method`
- `instance_selection`
- `minimum_point_count`
- `pose_estimation_method`
- `operation_point_rule`

## Execution Logic
1. 获取同步 RGB-D 或点云观测。
2. 检测目标类别并形成实例候选。
3. 根据指令、实例 ID、距离或置信度选择目标实例。
4. 分割目标实例并提取目标区域点云。
5. 估计对象位姿或局部操作表面。
6. 根据类别操作点规则生成候选位姿。
7. 输出置信度、实例信息和候选位姿。
8. 交由统一 Pose Validator 完成最终验证。

## Output
返回：
- target_pose
- optional object_pose
- confidence
- selected_instance
- source = known_category
- evidence

## Failure Handling
- `TARGET_NOT_FOUND`: 未检测到目标类别
- `INSTANCE_AMBIGUOUS`: 多个实例无法消歧
- `SEGMENTATION_FAILED`: 目标实例分割失败
- `SENSOR_DATA_UNAVAILABLE`: 深度或点云不可用
- `INSUFFICIENT_POINTS`: 有效目标点数量不足
- `POSE_ESTIMATION_FAILED`: 无法估计三维位姿
- `LOW_CONFIDENCE`: 结果低于阈值

本文件只描述策略语义；检测、分割、点云提取和位姿估计实现由用户接入。
