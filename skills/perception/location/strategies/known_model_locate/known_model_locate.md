---
id: locate.known_model
name: known_model_locate
display_name: Known Model Locate
parent_skill: locate
version: 1.0.0

description: >
  使用已知目标物体模型与场景点云进行配准，估计物体位姿并生成模型中定义的操作目标点。

input_schema: strategies/known_model_locate/known_model_locate.schema.yaml
implementation: strategies/known_model_locate/known_model_locate.py
---

# Known Model Locate Strategy

## Name
Known Model Locate

## Description
使用已知目标物体模型与场景点云进行配准，估计目标物体位姿，并将模型坐标系中的命名操作点转换到请求输出坐标系。

## When To Use
适用于：
- 目标物体具有可用三维模型
- 提供明确的 `model_id`
- 目标实例明确
- 需要较高定位精度
- 场景点云质量足够

不适用于：
- 无目标模型或模型版本不确定
- 只有开放词汇描述
- 场景无有效深度或点云

## Input Requirements
- `target.model_id`、运行时模型先验或 `strategy_params.model_id`
- 有效场景点云
- 模型仓库可访问
- 模型与场景坐标变换可用

## Strategy Parameters
- `model_id`
- `model_frame`
- `registration_method`: `auto | icp | gicp | ndt | feature_based`
- `voxel_size`
- `max_correspondence_distance`
- `fitness_threshold`
- `use_initial_guess`
- `initial_pose`
- `operation_point`

## Execution Logic
1. 加载目标模型和模型操作点定义。
2. 获取并预处理场景点云。
3. 根据需要生成或读取初始位姿估计。
4. 执行模型与场景点云配准。
5. 检查配准 fitness、重叠率和几何退化。
6. 将模型操作点转换为场景目标位姿。
7. 输出候选位姿、物体位姿和配准证据。
8. 交由统一 Pose Validator 完成最终验证。

## Output
返回后端无关的 Localization Candidate，包括：
- target_pose
- object_pose
- confidence / registration_fitness
- source = known_model
- evidence

## Failure Handling
- `MODEL_NOT_FOUND`: 模型或操作点定义不存在
- `SENSOR_DATA_UNAVAILABLE`: 场景点云不可用
- `REGISTRATION_FAILED`: 点云配准失败
- `LOW_CONFIDENCE`: 配准质量低于阈值
- `FRAME_TRANSFORM_FAILED`: 操作点无法转换到输出坐标系
- `POSE_ESTIMATION_FAILED`: 无法生成有效目标位姿

本文件只描述策略语义；模型仓库、点云处理、配准和位姿计算实现由用户接入。
