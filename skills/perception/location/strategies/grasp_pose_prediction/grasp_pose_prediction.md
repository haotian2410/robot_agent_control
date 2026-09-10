---
id: locate.grasp_pose_prediction
name: grasp_pose_prediction
display_name: Grasp Pose Prediction
parent_skill: locate
version: 1.0.0

description: >
  根据目标语义、RGB-D 或场景点云预测面向夹爪执行的 6-DoF 抓取候选位姿，
  对候选进行目标归属、质量、夹爪宽度和几何可行性筛选，并输出供 Move 与 Grasp Skill 使用的抓取位姿。

input_schema: strategies/grasp_pose_prediction/grasp_pose_prediction.schema.yaml
implementation: strategies/grasp_pose_prediction/grasp_pose_prediction.py
---

# Grasp Pose Prediction Strategy

## Name
Grasp Pose Prediction

## Description
针对 `target.type=grasp` 的任务，从 RGB-D、目标点云或场景点云中预测一个或多个 6-DoF 抓取候选位姿。该 Strategy 只负责抓取候选生成、目标归属和候选质量评估，不执行机械臂运动、夹爪闭合或抓取结果验证。

## When To Use
适用于：
- 需要为平行夹爪生成可执行的 6-DoF 抓取位姿
- 目标没有预定义抓取点，或需要根据当前场景动态选择抓取姿态
- 可获得 RGB-D、目标点云或场景点云
- 目标可能处于杂乱环境，需要从多个候选抓取中选择较优姿态
- 上层任务已经明确要抓取哪个物体或允许进行场景级抓取预测

不适用于：
- 已知模型中已经提供明确且有效的预定义抓取操作点，并明确要求使用该操作点
- 仅需要目标物体中心点或普通定位点
- 当前没有可用于抓取预测的深度或点云数据
- 需要吸盘、灵巧手等本 Strategy 未配置的末端执行器模型

## Input Requirements
必需或运行时可解析：
- `target.type = grasp`
- 目标描述、类别、实例、模型先验之一；或显式允许 `allow_scene_level=true`
- RGB-D 或 Scene Point Cloud
- Camera Intrinsics（使用 RGB-D 投影时）
- Transform Tree
- 当前夹爪模型和最大开口宽度
- Grasp Pose Predictor Backend

可选：
- 目标 mask / bounding box / instance point cloud
- `target_prior`
- 当前机器人状态和 Planning Scene

## Strategy Parameters
- `grasp_model`: `auto | contact_graspnet | graspnet | anygrasp | custom`
- `input_mode`: `auto | rgbd | point_cloud`
- `target_conditioning`: `auto | mask | bbox | description | category | scene`
- `target_description`
- `allow_scene_level`
- `num_candidates`
- `score_threshold`
- `min_gripper_width`
- `max_gripper_width`
- `roi_margin`
- `nms_translation_threshold`
- `nms_rotation_threshold`
- `require_collision_free`
- `require_reachable`
- `approach_clearance`

## Execution Logic
1. 获取同步 RGB-D / Point Cloud、相机标定、Transform Tree 和夹爪模型。
2. 解析目标实例：
   - 优先使用已有 instance mask / ROI / instance point cloud；
   - 否则根据 `object`、`category` 或 `instruction` 调用目标 grounding / segmentation 服务；
   - 仅当 `allow_scene_level=true` 时允许直接在整个场景生成抓取。
3. 构造目标点云或带目标条件的抓取预测输入。
4. 调用配置的 Grasp Pose Predictor 生成 6-DoF 抓取候选：
   - 抓取位置
   - 抓取姿态
   - 抓取质量分数
   - 建议夹爪宽度
   - 可选 approach direction
5. 将候选从预测坐标系转换到统一输出坐标系。
6. 根据目标 mask / 点云检查候选是否属于目标实例，移除抓取到邻近物体的候选。
7. 根据夹爪宽度、质量阈值、NMS、局部碰撞和 approach clearance 过滤候选。
8. 输出剩余 Candidate Grasp Poses。
9. 交由统一 Pose Validator 做最终坐标、完整性、可达性、碰撞和安全验证。
10. 返回最高质量的 `target_pose`，并保留候选抓取元数据供 Move / Grasp Skill 使用。

## Candidate Format
每个候选建议使用：

```yaml
pose:
  frame: base
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation:
    representation: quaternion
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0

score: 0.92
gripper_width: 0.045
approach_direction: [0.0, 0.0, -1.0]
target_instance_id: optional
source: grasp_pose_prediction
```

## Output
返回：
- `target_pose`: 选中的 6-DoF 抓取位姿
- `confidence`: 选中候选的抓取质量分数
- `candidates`: 抓取候选列表
- `grasp_metadata`
  - `gripper_width`
  - `approach_direction`
  - `grasp_score`
  - `predictor`
  - `target_instance_id`
- `source = grasp_pose_prediction`
- `evidence`

下游典型调用：

```text
Grasp Pose Prediction
        ↓ target_pose + grasp_metadata
Move Skill
        ↓ 移动至 pre-grasp / grasp pose
Grasp Skill
        ↓ 根据 gripper_width / force 等参数执行夹持
```

## Failure Handling
- `INVALID_REQUEST`: 目标不是 grasp，或缺少目标条件且未允许 scene-level 预测
- `SENSOR_DATA_UNAVAILABLE`: RGB-D / 点云不可用
- `TARGET_NOT_FOUND`: 无法从场景中解析目标实例
- `GRASP_POSE_PREDICTION_FAILED`: 抓取预测后没有产生候选
- `LOW_CONFIDENCE`: 所有候选低于质量阈值
- `NO_VALID_CANDIDATE`: 候选经宽度、目标归属、碰撞或可达性过滤后全部失效
- `POSE_VALIDATION_FAILED`: 最终候选未通过统一 Pose Validator

## Strategy Boundary
- 本 Strategy 预测“在哪里、以什么姿态抓”，不执行机械臂运动。
- 本 Strategy 不闭合夹爪，不判断抓取是否最终成功。
- 目标不在视野内或严重遮挡时应返回结构化失败，由上层调用 Search Skill 更新观察视角。
- Move Skill 负责到达抓取相关位姿；Grasp Skill 负责夹爪动作与抓取结果验证。
- 不允许抓取预测模型绕过机器人碰撞、可达性或安全验证直接驱动机械臂。
