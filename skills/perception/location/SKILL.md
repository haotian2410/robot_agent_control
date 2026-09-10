---
id: locate
name: locate
display_name: Locate
version: 1.0.0

description: >
  根据目标描述、物体先验、视觉观测、点云或参考位姿定位操作目标点，
  返回统一坐标系下的目标位姿，不执行机械臂运动或末端操作。

entrypoint: skill.py
input_schema: schema.yaml
---

# Locate Skill

## Name
Locate

## Description
根据目标物体、场景观测、参考位姿和任务需求定位指定操作目标点，并返回目标点位姿。Locate Skill 仅负责目标点生成、估计和有效性验证，不负责机械臂运动、路径规划、夹爪控制或末端操作执行。

Locate Skill 将定位过程拆分为两个阶段：
- Locate Strategy：定义目标位姿通过何种信息和方法生成
- Validation Pipeline：统一执行坐标系转换、置信度、可达性、碰撞和结果完整性验证

## Purpose
用于：
- 定位抓取、放置、按压、插入和拔出等操作目标点
- 根据 RGB-D 或点云预测面向夹爪执行的 6-DoF 抓取姿态
- 根据已知物体模型估计高精度目标位姿
- 根据已知类别检测目标实例并生成操作点
- 根据开放词汇描述定位未知物体或目标区域
- 在最终操作点附近生成安全靠近点
- 根据末端、物体、位姿或坐标系生成相对目标点
- 为 Move、Grasp、VLA、ACT 或其他操作技能提供目标位姿

不负责：
- 机械臂运动规划与执行
- 轨迹生成和控制器调用
- 夹爪闭合控制和抓取动作执行
- VLA 或 ACT 动作序列生成
- 场景主动探索和相机运动规划
- 高层任务逻辑规划

说明：
Locate Skill 可以调用视觉、点云、坐标变换、可达性和碰撞服务验证目标点，但这些检查只用于判断定位结果是否可用，不生成机械臂运动轨迹。

## When To Use
当机器人需要从物体模型、类别先验、视觉图像、深度数据、点云、参考位姿或自然语言空间关系中确定操作目标位姿时调用该技能。

典型场景：
- 定位杯子的抓取点或预测可执行 6-DoF 抓取姿态
- 定位按钮的按压点
- 定位孔位的插入点
- 定位物体的放置点或拔出点
- 为复杂操作生成预靠近点
- 在机械臂末端左上方生成一个目标点
- 在物体前方指定距离处生成相对目标位姿

## Localization Strategy Architecture

完整定位方案由 Locate Strategy 和统一验证流程组成：

```text
Localization Result = Locate Strategy + Validation Pipeline
```

策略分为四类：

```text
Perception-Based
├── Known Model Locate
├── Known Category Locate
└── Unknown Object Locate

Manipulation-Oriented
└── Grasp Pose Prediction

Geometry-Based
└── Relative Point Locate

Derived-Point
└── Approach Point Locate
```

所有策略输出统一的 Candidate Pose，再经过：

```text
Frame Transform
→ Pose Completeness Check
→ Confidence Check
→ Reachability Check (optional)
→ Collision / Clearance Check (optional)
→ Return Target Pose
```

## Locate Strategies

### Known Model Locate

使用已知物体模型与场景点云进行配准，估计目标物体位姿并转换模型中定义的操作点。

适用于：
- 已知目标物体模型或实例
- 提供 `model_id`
- 需要较高定位精度
- 场景点云质量足够

特点：
- 依赖模型仓库和点云配准
- 可输出物体位姿与模型操作点位姿
- 精度通常高于开放词汇定位
- 模型缺失或配准质量不足时失败

详细策略：
Known Model Locate

### Known Category Locate

根据已知目标类别执行检测、实例选择、分割和深度或点云位姿估计。

适用于：
- 已知目标类别但未知具体实例
- 场景中可能存在多个同类目标
- 需要根据类别规则生成操作点
- 可用 RGB-D 或点云观测

特点：
- 使用检测和分割获得目标区域
- 根据实例选择规则消除多目标歧义
- 结合深度或点云估计三维位姿
- 不要求目标实例的精确三维模型

详细策略：
Known Category Locate

### Unknown Object Locate

使用 VLM 或开放词汇视觉模型理解目标描述，并结合深度或点云生成三维目标位姿。

适用于：
- 无已知模型和类别先验
- 目标通过自然语言描述
- 需要开放词汇目标定位
- 目标可能是物体局部、区域或语义位置

特点：
- 泛化能力较强
- 依赖目标描述和视觉 grounding 质量
- 三维位置来自深度或点云投影
- 结果必须通过置信度和几何有效性验证

详细策略：
Unknown Object Locate

### Grasp Pose Prediction

根据目标语义、RGB-D 或场景点云预测面向夹爪执行的 6-DoF 抓取候选位姿，并根据目标归属、抓取质量、夹爪开口、局部碰撞和几何可行性进行筛选。

适用于：
- `target.type=grasp`
- 需要动态生成抓取姿态而不是只定位物体中心或普通操作点
- 目标没有明确的预定义抓取操作点
- 可获得 RGB-D、目标点云或场景点云
- 需要从多个候选抓取中选择更可靠的抓取姿态

特点：
- 输出完整 6-DoF 抓取 Pose，而不是单独三维点
- 可接入 Contact-GraspNet、GraspNet、AnyGrasp 或自定义预测后端
- 可使用目标 mask、bbox、类别或自然语言描述限制抓取候选属于指定目标
- 候选可携带 `gripper_width`、`approach_direction` 和 `grasp_score`
- 最终仍必须经过统一 Pose Validator 的坐标、可达性和碰撞验证
- 不执行 Move，也不闭合夹爪

详细策略：
Grasp Pose Prediction

### Approach Point Locate

根据已知最终目标位姿、接近方向和距离，在目标附近生成安全、可达的预靠近位姿。

适用于：
- 最终目标位姿已经存在
- 后续由 Move、VLA、ACT 或复杂控制器完成操作
- 需要先到达目标邻域
- 需要保持明确的接近方向和安全距离

特点：
- 不负责检测最终目标
- 可以生成并排序多个候选靠近点
- 可结合可达性、碰撞和最小间隙约束
- 输出的是准备位姿而不是最终操作位姿

详细策略：
Approach Point Locate

### Relative Point Locate

根据参考位姿和语义或笛卡尔偏移生成目标位姿。

适用于：
- 在机械臂末端附近生成目标点
- 在目标物体或指定坐标系附近生成偏移点
- 根据左、右、上、下、前、后描述定位
- 根据明确的 x、y、z 偏移生成目标位姿

特点：
- 不依赖目标检测，但必须获得有效参考位姿
- 支持末端、物体、显式 Pose 和命名坐标系
- 语义方向必须解析到明确参考坐标系
- 可在初始目标无效时执行有限邻域修正

详细策略：
Relative Point Locate

## Strategy Selection Policy

Locate Skill 根据目标类型、显式策略、目标先验、参考信息和运行时能力选择定位策略。

选择优先级：

1. 用户或上层任务显式指定的合法策略优先级最高。
2. `target.type=relative` 时选择 Relative Point Locate。
3. `target.type=approach` 时选择 Approach Point Locate。
4. `target.type=grasp` 且同时提供已知 `model_id` 与明确的非 `auto` `operation_point` 时，选择 Known Model Locate 使用预定义抓取操作点。
5. 其他 `target.type=grasp` 请求默认选择 Grasp Pose Prediction。
6. 非抓取任务中，提供有效 `model_id` 或运行时存在确定模型先验时选择 Known Model Locate。
7. 非抓取任务中，提供明确 `category` 或运行时存在确定类别先验时选择 Known Category Locate。
8. 非抓取任务中，提供开放词汇 `object` 或 `instruction` 且无更强先验时选择 Unknown Object Locate。
9. Grasp Pose Prediction 必须具有目标条件；如果没有 object、instruction、category、instance_id 或 model_id，则必须显式设置 `allow_scene_level=true`。
10. 显式策略缺少必需输入时返回 `INVALID_REQUEST`，不得静默改用其他策略。
11. Grasp Pose Prediction 失败不得自动回退到普通物体定位策略，因为普通定位结果不能等价替代 6-DoF 抓取姿态。
12. Relative Point 和 Approach Point 不得自动回退到感知定位策略。
13. 所有候选位姿必须经过统一 Validation Pipeline；验证失败不能被 Selector 覆盖。

默认选择逻辑：

```text
target.type = relative
→ Relative Point Locate

target.type = approach
→ Approach Point Locate

target.type = grasp
+ model_id
+ explicit operation_point
→ Known Model Locate

target.type = grasp
+ no explicit predefined grasp pose
→ Grasp Pose Prediction

non-grasp + model_id available
→ Known Model Locate

non-grasp + category available
→ Known Category Locate

non-grasp + object description or instruction available
→ Unknown Object Locate

no sufficient target evidence
→ INVALID_REQUEST
```

允许的自动 fallback：

```text
Known Model Locate failed
+ strategy=auto
+ non-grasp task
+ category prior available
→ Known Category Locate

Known Category Locate failed
+ strategy=auto
+ non-grasp task
+ open-vocabulary description available
→ Unknown Object Locate

Grasp Pose Prediction failed
→ return grasp-pose failure
→ upper layer may call Search Skill or request another observation

explicit strategy failed
→ return failure without silent fallback
```

职责边界：
- Agent：理解自然语言并生成结构化目标、先验、参考位姿、偏移和约束
- Strategy Selector：根据确定性规则选择 Locate Strategy
- Locate Strategy：生成候选目标位姿和策略置信度
- Grasp Pose Prediction：生成目标约束的 6-DoF 抓取候选及抓取元数据
- Pose Validator：执行坐标系、完整性、置信度、可达性和碰撞验证
- Move Skill：规划并执行到目标 / 抓取相关位姿的机械臂运动
- Grasp Skill：执行夹爪闭合、夹持和抓取结果验证
- Safety System：可以否决任何不可安全使用的定位结果

## Input

### Target
需要定位的操作目标。

支持：
- Grasp Pose / Grasp Point
- Place Point
- Press Point
- Insert Point
- Extract Point
- Approach Point
- Relative Point
- Object Pose

格式：

```yaml
target:
  type: grasp | place | press | insert | extract | approach | relative | object_pose
  object: optional string
  instruction: optional string
  instance_id: optional string
  category: optional string
  model_id: optional string
  operation_point: optional string
```

### Reference
Approach Point 或 Relative Point 使用的参考信息。

格式：

```yaml
reference:
  type: end_effector | object | pose | frame | target_pose
  id: optional string
  frame: base
  pose: optional
```

### Offset
Relative Point 使用的语义或笛卡尔偏移。

格式：

```yaml
offset:
  mode: semantic | cartesian
  semantic:
    direction: left | right | up | down | forward | backward
    distance: 0.05
  cartesian:
    x: 0.0
    y: 0.0
    z: 0.0
```

### Localization
定位策略和候选选择要求。

支持：
- strategy: auto | known_model_locate | known_category_locate | unknown_object_locate | grasp_pose_prediction | approach_point_locate | relative_point_locate
- precision: low | medium | high
- output_frame
- candidate_selection: highest_confidence | nearest | instruction | safest
- max_candidates
- allow_fallback

格式：

```yaml
localization:
  strategy: auto
  precision: medium
  output_frame: base
  allow_fallback: true
```

### Constraints
定位结果约束：
- confidence_threshold
- check_reachability
- avoid_collision
- minimum_clearance
- position_tolerance
- orientation_required
- timeout

### Strategy Parameters
策略专用参数，例如：
- registration_method
- detector_model
- segmentation_method
- vlm_model
- grasp_model
- target_conditioning
- num_candidates
- score_threshold
- max_gripper_width
- approach_clearance
- approach_direction
- approach_distance
- semantic_direction_mapping
- search_radius

最终由所选 Strategy 的 schema 进行校验。

### Runtime Context
由机器人系统内部提供：
- RGB / Depth Image
- Camera Intrinsics
- Scene Point Cloud
- Transform Tree
- Current End-Effector Pose
- Robot Model and State
- Planning Scene / Occupancy Information
- Object Model Repository
- Perception Model Registry
- Grasp Pose Predictor / Gripper Model
- Target Prior

## Output
返回：

### Success
定位是否成功。

### Status
状态：
- completed
- failed
- timeout

### Selection
实际选择结果：
- locate_strategy
- selection_reason
- fallback_used

### Localization Result
定位结果：
- target_pose
- object_pose
- confidence
- source
- candidates
- grasp_metadata（抓取任务可选）
- validation
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
输入字段缺失、参数非法，或所选策略与目标类型不兼容。

Possible Causes:
- Relative Point 缺少 reference 或 offset
- Approach Point 缺少最终 target pose、方向或距离
- Known Model Locate 缺少 `model_id`
- Known Category Locate 缺少 `category`
- Unknown Object Locate 缺少目标描述
- Grasp Pose Prediction 缺少 grasp 目标条件，且未允许 scene-level 预测

Recommended Action:
- 根据主 schema 和策略 schema 修正输入
- 补充目标先验、参考位姿或策略参数
- 在允许自动选择时将 `localization.strategy` 设置为 `auto`

### Target Not Found

Description:
当前观测中未检测、匹配或 grounding 到指定目标。

Possible Causes:
- 目标不在视野内
- 目标描述不明确
- 类别或模型与实际目标不匹配
- 场景存在遮挡、反光或视觉质量不足

Recommended Action:
- 更新 RGB-D、点云或相机视角
- 增加目标描述或实例约束
- 检查类别和模型标识
- 在自动模式下尝试合法 fallback

### Reference Invalid

Description:
无法获得 Approach Point 或 Relative Point 所需的有效参考位姿或坐标变换。

Possible Causes:
- 参考对象或命名坐标系不存在
- 当前末端位姿不可用
- 参考 Pose 不完整
- 坐标变换缺失、过期或查询失败

Recommended Action:
- 检查 reference.type、id、frame 和 pose
- 更新机器人状态或对象位姿
- 重新获取 Transform Tree
- 修改参考信息后重新定位

### Sensor Data Unavailable

Description:
所选感知策略需要的图像、深度或点云数据不可用。

Possible Causes:
- RGB-D 传感器未就绪
- 点云为空或时间戳过期
- 相机内参缺失
- 观测与坐标变换时间不同步

Recommended Action:
- 检查传感器和数据同步状态
- 重新采集观测
- 补充相机内参和坐标变换
- 在数据恢复后重新执行

### Pose Estimation Failed

Description:
已经检测到目标或获得参考信息，但无法生成有效三维目标位姿。

Possible Causes:
- 点云配准失败
- 深度数据缺失或噪声过大
- 目标区域有效点不足
- 局部几何不足以估计方向
- 相对点或靠近点生成参数退化

Recommended Action:
- 更新点云并调整预处理参数
- 改用更可靠的位姿估计方法
- 补充方向或姿态约束
- 检查策略专用参数

### Grasp Pose Prediction Failed

Description:
抓取目标已经确定，但抓取预测后没有得到满足任务和夹爪约束的有效 6-DoF 抓取候选。

Possible Causes:
- 抓取预测后端未返回候选
- 目标点云过少、遮挡严重或深度质量不足
- 所有候选抓取质量低于 `score_threshold`
- 候选夹爪宽度超过当前夹爪能力
- 候选抓取属于邻近物体而非目标实例
- 候选在接近方向上发生碰撞或不可达

Recommended Action:
- 更新 RGB-D、点云或通过 Search Skill 改变观察视角
- 改善目标 mask / ROI / 实例分割
- 调整抓取预测模型和合法的策略参数
- 检查夹爪模型、最大开口和 TCP 坐标约定
- 不得以普通目标中心点静默替代失败的抓取 Pose

### Low Confidence

Description:
候选位姿已生成，但置信度低于请求阈值。

Possible Causes:
- 目标存在遮挡或相似实例
- 模型配准适配度不足
- VLM grounding 不稳定
- 深度或点云有效样本不足

Recommended Action:
- 请求重新定位或使用多视角观测
- 增加目标实例描述
- 降低阈值前由上层任务评估风险
- 使用更强的模型或类别先验

### Pose Validation Failed

Description:
候选位姿未通过坐标系、完整性、可达性、碰撞或最小间隙验证。

Possible Causes:
- 输出坐标系转换失败
- 目标位姿格式或数值无效
- 目标超出机器人工作空间
- 目标点处于障碍物内部或安全间隙不足
- 要求姿态但策略未能生成有效姿态

Recommended Action:
- 检查输出坐标系和 Transform Tree
- 调整目标点、偏移、靠近距离或搜索半径
- 更新 Planning Scene
- 在任务允许时生成其他候选点

### Localization Timeout

Description:
定位过程在规定时间内未完成。

Possible Causes:
- 点云配准或候选搜索耗时过长
- 感知模型响应超时
- 候选数量或搜索空间过大
- 外部服务不可用

Recommended Action:
- 增加 timeout 或减少 max_candidates
- 调整配准、分割和搜索参数
- 检查感知服务状态
- 使用更强先验缩小搜索范围

### Localization Error

Description:
定位执行阶段发生未归类异常。

Possible Causes:
- 模型服务、传感器或坐标变换后端异常
- 依赖组件通信失败
- 策略实现抛出未处理异常
- 运行时上下文不一致

Recommended Action:
- 保存请求、选择结果和运行时上下文摘要
- 检查依赖服务和日志
- 将异常映射为结构化错误码
- 仅在状态有效时重新执行
