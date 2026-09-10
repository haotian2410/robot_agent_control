---
id: search
name: search
display_name: Search
version: 1.0.0

description: >
  当目标不在视野、部分遮挡或当前观测不足以可靠定位时，根据目标先验、
  视觉观测、遮挡关系、搜索区域和历史观测生成并选择下一相机观察位姿。
  不执行机械臂或相机运动，也不负责最终目标定位和操作执行。

entrypoint: skill.py
input_schema: schema.yaml
---

# Search Skill

## Name
Search

## Description
当目标不在当前视野内、部分被遮挡，或当前视角不足以支持可靠定位、识别和位姿估计时，Search Skill 根据目标信息、当前观测、位置先验、遮挡信息、相机模型、机器人状态和搜索历史，生成并选择下一相机观察位姿。

Search Skill 只负责主动视觉搜索中的视角规划：
- Search Strategy：定义采用何种搜索或视角更新方法
- Candidate Viewpoint Pipeline：生成、转换、评分并验证候选观察位姿
- Search State：记录已访问视角、失败视角、观测改善和停止条件

Search Skill 不直接控制机械臂或相机。所选视角由 Move Skill、云台控制器、移动底盘或其他运动技能执行；运动完成后应重新采集观测，并再次调用 Location Skill。

## Purpose
用于：
- 目标完全不在视野时执行系统化扫描
- 根据目标上次位置、语义区域或图像边缘提示进行先验引导搜索
- 目标部分遮挡时选择能够绕开遮挡物的新视角
- 目标已可见但置信度、深度覆盖或位姿质量不足时优化局部视角
- 为 Location、Inspect、Read、Recognize 等感知技能提供下一观察位姿
- 为腕部相机生成对应的机械臂末端目标位姿
- 管理多轮搜索中的已访问视角、失败视角和终止条件

不负责：
- 最终目标检测、分割或三维定位
- 机械臂路径规划和运动执行
- 云台、底盘或相机驱动控制
- 抓取、放置或末端工具操作
- 修改场景中的遮挡物
- 高层任务逻辑规划

说明：
Search Skill 可以调用 VLM、检测、分割、深度、点云、射线投射、可达性和碰撞服务来生成或评估候选视角，但 VLM 仅提供语义、方向和遮挡提示，最终执行位姿必须经过确定性几何、安全和历史重复检查。

## When To Use
当 Location Skill 或其他感知技能返回以下情况时调用：
- `TARGET_NOT_FOUND`
- `PARTIALLY_VISIBLE`
- `LOW_CONFIDENCE`
- `DEPTH_UNAVAILABLE`
- `INSUFFICIENT_POINTS`
- `POSE_ESTIMATION_FAILED`
- 当前视角的观测质量不足

典型场景：
- 杯子不在腕部相机视野中，需要左右或 Z 字形扫描
- 目标最后出现在图像左边缘，需要优先向左更新视角
- 目标被前方物体遮挡，需要从侧上方观察
- 目标可见但深度缺失，需要轻微改变观察角度
- 多轮定位失败，需要避免重复访问无效视角

不应调用的情况：
- 当前目标位姿已经满足任务置信度和精度要求
- 用户只要求机械臂移动而不涉及视觉搜索
- 相机为不可运动的固定相机且系统没有其他可移动观察设备
- 搜索区域、相机状态和目标信息均不足以生成安全候选视角

## Search Strategy Architecture

完整视角搜索方案由 Search Strategy 和统一候选视角流程组成：

```text
View Search Result = Search Strategy + Candidate Viewpoint Pipeline + Search State
```

策略结构：

```text
Global Search
├── Systematic Scan
└── Prior-Guided Search

Active View Selection
├── Occlusion-Aware View
└── Local View Refinement
```

所有策略输出统一的 Candidate Viewpoint，再经过：

```text
Candidate Viewpoint Generation
→ Camera / Robot Frame Transform
→ Look-at Orientation Generation
→ Field-of-View Check
→ History Duplicate Check
→ Reachability Check (optional)
→ Collision / Clearance Check (optional)
→ Candidate Scoring and Ranking
→ Return Next Viewpoint
```

## Search Strategies

### Systematic Scan

在没有可靠目标位置先验时，根据预定义搜索区域生成确定性的扫描视角序列。

适用于：
- 目标完全不在视野
- 没有可靠目标位置先验
- 已知需要搜索的工作区域或命名区域
- 需要可重复、可解释的覆盖式搜索

特点：
- 支持 horizontal、vertical、z_scan、grid、orbit 等扫描模式
- 根据搜索历史跳过已经访问或失败的视角
- 不依赖 VLM 直接生成精确机械臂位姿
- 搜索区域缺失或所有候选无效时返回失败

详细策略：
Systematic Scan

### Prior-Guided Search

根据目标上次位置、语义区域、跟踪预测、图像边缘提示或 VLM 粗方向提示，在局部区域内优先生成候选视角。

适用于：
- 目标当前不可见但存在位置先验
- 目标刚从图像边缘离开
- 用户或场景地图提供了大致区域
- 目标跟踪器能够预测目标方向或位置

特点：
- 优先搜索高概率区域，减少无效扫描
- 支持位置先验和方向先验
- 可以围绕先验位置生成球面、圆环或局部栅格候选
- 先验不确定性必须影响搜索半径和候选分布

详细策略：
Prior-Guided Search

### Occlusion-Aware View

根据目标可见区域、遮挡比例、遮挡物位置和目标粗位姿，生成能够改善目标可见性的侧向、上方或环绕观察候选。

适用于：
- 目标部分可见
- 已检测到或估计出遮挡关系
- 存在目标粗位置、目标射线或局部深度
- 需要绕开遮挡物而不是全局扫描

特点：
- VLM 可以输出遮挡语义和粗方向提示
- 几何模块负责生成精确候选视角
- 候选评分应考虑预期遮挡降低和目标保持在视野内
- 不负责移动或移除遮挡物

详细策略：
Occlusion-Aware View

### Local View Refinement

围绕当前相机位姿生成小范围平移和旋转候选，用于改善目标居中、深度覆盖、表面入射角、分辨率或位姿估计质量。

适用于：
- 目标已经可见
- 定位置信度或几何质量不足
- 深度缺失、反光或观察角度不佳
- 不需要进行大范围场景搜索

特点：
- 只执行局部视角扰动
- 保持目标区域尽量留在视野中
- 候选移动距离通常小于全局搜索策略
- 当前观测已足够时应停止，而不是继续移动

详细策略：
Local View Refinement

## Strategy Selection Policy

Search Skill 根据显式策略、目标可见性、遮挡状态、观测质量、位置先验、搜索区域和历史状态选择搜索策略。

选择优先级：

1. 用户或上层任务显式指定的合法策略优先级最高。
2. 当前观测已经满足请求的置信度和质量要求时返回 `search_required=false`，不得无意义更新视角。
3. `visibility=partial` 且存在遮挡证据时选择 Occlusion-Aware View。
4. 目标可见但置信度、深度覆盖、分辨率或位姿质量不足时选择 Local View Refinement。
5. 目标不可见或可见性未知，但存在目标位置、方向、图像边缘、跟踪或语义区域先验时选择 Prior-Guided Search。
6. 目标不可见且无可靠先验，但存在有效搜索区域时选择 Systematic Scan。
7. 显式策略缺少必需输入时返回 `INVALID_REQUEST`，不得静默改用其他策略。
8. 自动模式下只有配置允许且 fallback 所需输入完整时才能切换策略。
9. 所有候选视角必须经过统一 Viewpoint Validator；Selector 不能覆盖可达性、碰撞和重复视角否决。
10. 达到 `max_steps`、连续无改善次数或候选空间耗尽时返回 `SEARCH_EXHAUSTED`。

默认选择逻辑：

```text
observation sufficient
→ search_required = false

visibility = partial + occlusion evidence
→ Occlusion-Aware View

visibility = visible + observation quality insufficient
→ Local View Refinement

visibility = not_visible|unknown + target prior available
→ Prior-Guided Search

visibility = not_visible|unknown + no prior + valid search region
→ Systematic Scan

no sufficient evidence or search region
→ INVALID_REQUEST
```

允许的自动 fallback：

```text
Occlusion-Aware View failed
+ target prior available
→ Prior-Guided Search

Local View Refinement lost target
+ last target prior available
→ Prior-Guided Search

Prior-Guided Search exhausted
+ valid global search region available
→ Systematic Scan

explicit strategy failed
→ return failure without silent fallback
```

职责边界：
- Agent：根据 Location 结果构造目标、当前观测、搜索意图、搜索区域和约束
- VLM / Perception Analyzer：输出可见性、遮挡、图像边缘和粗方向提示
- Strategy Selector：根据确定性规则选择 Search Strategy
- Search Strategy：生成候选观察位姿和预期收益
- Viewpoint Validator：执行坐标转换、视场、历史、可达性、碰撞和候选排序
- Move / Camera Controller：执行所选观察位姿
- Location Skill：在新视角下重新定位目标
- Safety System：可以否决任何不安全的观察位姿

## Input

### Target
需要搜索或改善观测的目标。

格式：

```yaml
target:
  object: optional string
  instruction: optional string
  instance_id: optional string
  category: optional string
  model_id: optional string
  prior_pose: optional pose
  prior_region: optional object
```

### Trigger
触发搜索的上游结果。

支持：
- target_not_found
- partially_visible
- low_confidence
- poor_depth
- poor_geometry
- inspection_required

格式：

```yaml
trigger:
  reason: target_not_found
  source_skill: locate
  source_error_code: TARGET_NOT_FOUND
```

### Observation
当前视角下的目标可见性和观测质量。

格式：

```yaml
observation:
  visibility: visible | partial | not_visible | unknown
  confidence: 0.0
  bounding_box: optional
  mask: optional
  occlusion_ratio: optional
  occluder: optional
  image_edge_hint: left | right | top | bottom | none
  direction_hint: optional
  quality:
    depth_coverage: optional
    sharpness: optional
    pose_quality: optional
```

### Camera
当前观察设备及其运动映射。

支持：
- wrist
- head
- pan_tilt
- mobile
- movable_external
- fixed_external

格式：

```yaml
camera:
  id: wrist_camera
  mount_type: wrist
  optical_frame: camera_optical
  current_pose: optional
  controlled_by: move
```

对于腕部相机，Search Skill 根据手眼外参将候选 Camera Pose 转换为 End-Effector Pose；对于云台或底盘相机，返回对应控制目标。

### Search
搜索策略、目标和候选选择要求。

支持：
- strategy: auto | systematic_scan | prior_guided_search | occlusion_aware_view | local_view_refinement
- goal: locate | identify | pose_estimation | inspect
- mode: single_step | iterative
- output_frame
- candidate_selection: highest_score | max_information | min_motion | safest
- max_candidates
- max_steps
- allow_fallback

格式：

```yaml
search:
  strategy: auto
  goal: locate
  mode: single_step
  output_frame: base
  candidate_selection: highest_score
  max_candidates: 20
  max_steps: 8
  allow_fallback: true
```

### Search Region
限制系统化或先验引导搜索的空间范围。

格式：

```yaml
search_region:
  type: named_region | box | sphere | around_prior | viewpoint_set
  frame: base
  id: optional string
  center: optional
  size: optional
  radius: optional
  viewpoints: optional array
```

### Constraints
观察位姿约束：
- confidence_threshold
- quality_threshold
- check_reachability
- avoid_collision
- minimum_clearance
- min_view_distance
- max_view_distance
- max_translation
- max_rotation
- keep_target_in_view
- require_look_at_target
- timeout

### Search History
多轮搜索状态：
- step_index
- visited_viewpoints
- failed_viewpoints
- previous_observations
- no_improvement_steps
- best_observation

Search Skill 不在内部长期保存任务状态；Agent 或任务运行时应在每轮调用中传回 Search History。

### Strategy Parameters
策略专用参数，例如：
- scan_pattern
- scan_step
- prior_radius
- direction_bias
- orbit_radius
- azimuth_offsets
- elevation_offsets
- translation_step
- rotation_step
- look_at_mode

最终由所选 Strategy 的 schema 进行校验。

### Runtime Context
由机器人系统内部提供：
- RGB / Depth Image
- Camera Intrinsics and Field of View
- Current Camera Pose
- Camera-to-End-Effector Extrinsics
- Transform Tree
- Robot Model and State
- Planning Scene / Occupancy Information
- Scene Geometry / Point Cloud
- Semantic Map
- Target Prior / Tracking Prediction
- VLM / Detection / Segmentation Result

## Output
返回：

### Success
是否成功完成本轮视角选择。

### Status
状态：
- completed
- failed
- timeout
- exhausted

### Selection
实际选择结果：
- search_strategy
- search_required
- selection_reason
- fallback_used

### Search Result
搜索结果：
- next_viewpoint
- expected_visibility_gain
- expected_information_gain
- score
- candidates
- validation
- search_state
- execution_request

`next_viewpoint` 同时描述期望 Camera Pose 和实际控制目标：

```yaml
next_viewpoint:
  camera_pose: optional
  look_at_point: optional
  control_target:
    type: end_effector_pose | camera_pose | pan_tilt | base_pose
    pose: optional
    joints: optional
```

对于 `controlled_by=move` 的腕部相机，`execution_request` 可以直接作为 Move Skill 请求模板，但 Search Skill 本身不得执行该请求。

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
输入字段缺失、参数非法，或显式搜索策略与当前观测不兼容。

Possible Causes:
- 缺少 observation.visibility
- Systematic Scan 缺少有效 search_region 或预定义视角集
- Prior-Guided Search 缺少目标位置或方向先验
- Occlusion-Aware View 缺少目标粗位置、遮挡信息或目标射线
- 相机类型和 controlled_by 配置不兼容

Recommended Action:
- 根据主 schema 和策略 schema 修正输入
- 补充 Location 输出、目标先验或搜索区域
- 在允许自动选择时将 `search.strategy` 设置为 `auto`

### Search Not Required

Description:
当前目标可见性、置信度和观测质量已经满足请求要求。

Recommended Action:
- 返回 `search_required=false`
- 继续执行 Location、Move、Grasp 或其他后续技能
- 不生成无意义的新视角

### Observation Insufficient

Description:
缺少生成或评估新视角所需的观测信息。

Possible Causes:
- RGB-D 数据缺失或过期
- 缺少相机内参、视场角或当前相机位姿
- 遮挡策略缺少目标或遮挡物几何信息
- VLM 只给出模糊建议，无法映射到明确参考坐标系

Recommended Action:
- 重新采集同步观测
- 补充 Camera Info 和 Transform Tree
- 将粗语义方向解析到明确坐标系
- 在可用时切换到不依赖遮挡几何的合法策略

### Search Region Invalid

Description:
搜索区域为空、不可解析、超出设备可用范围或与目标先验不一致。

Possible Causes:
- 命名区域不存在
- Box、Sphere 或 viewpoint_set 参数不完整
- 搜索区域与机器人工作空间无交集
- 搜索区域坐标变换失败

Recommended Action:
- 检查 search_region.type、frame 和几何参数
- 更新场景地图或命名区域
- 缩小或移动搜索区域
- 重新获取坐标变换

### Viewpoint Generation Failed

Description:
所选策略无法生成有效的候选观察位姿。

Possible Causes:
- 扫描步长、半径或角度参数退化
- 目标先验不确定性过大
- 遮挡几何不足以计算侧向视角
- 当前相机附近没有满足约束的局部扰动

Recommended Action:
- 调整策略参数和候选数量
- 放宽非安全类 soft 约束
- 补充目标或遮挡先验
- 在自动模式下尝试合法 fallback

### No Valid Viewpoint

Description:
候选视角已生成，但均未通过视场、历史、可达性、碰撞或最小间隙验证。

Possible Causes:
- 所有候选超出相机或机械臂可用范围
- 候选与障碍物碰撞
- 相机无法朝向目标区域
- 所有候选均已访问或曾执行失败
- 视角距离不满足约束

Recommended Action:
- 更新 Planning Scene 和机器人状态
- 扩大搜索区域或增加候选数量
- 调整安全距离之外的 soft 参数
- 切换其他可移动相机或观察设备

### Frame Transform Failed

Description:
无法在 Camera、End-Effector、Base 或目标参考坐标系之间转换候选视角。

Possible Causes:
- 手眼标定外参缺失
- Transform Tree 不完整或时间戳不一致
- 当前相机或末端位姿不可用
- 输出坐标系不存在

Recommended Action:
- 检查相机外参和 Transform Tree
- 更新同步机器人状态
- 修正 camera.optical_frame 和 search.output_frame
- 重新执行视角生成

### Search Exhausted

Description:
已达到最大搜索步数、候选空间耗尽或连续多轮没有观测改善。

Possible Causes:
- 目标不在指定搜索区域
- 目标完全被遮挡
- 目标描述或先验错误
- 搜索区域无法由当前相机设备覆盖

Recommended Action:
- 返回 `SEARCH_EXHAUSTED` 给上层 Agent
- 扩大或更换搜索区域
- 切换相机、底盘或观察设备
- 请求用户或任务规划器提供新的目标先验
- 不得无限循环搜索

### Search Timeout

Description:
候选生成、可见性评估或验证过程超过请求时间限制。

Possible Causes:
- 候选数量过多
- 射线投射、点云或遮挡分析耗时过长
- 可达性或碰撞服务响应超时
- 外部感知服务不可用

Recommended Action:
- 增加 timeout 或减少 max_candidates
- 缩小搜索区域
- 使用更强目标先验
- 检查依赖服务状态

### Search Error

Description:
视角搜索阶段发生未归类异常。

Possible Causes:
- 候选生成器、评分器或坐标变换后端异常
- 依赖服务通信失败
- 策略实现抛出未处理异常
- 搜索历史状态不一致

Recommended Action:
- 保存请求、选择结果和搜索历史摘要
- 检查依赖服务与日志
- 将异常映射为结构化错误码
- 仅在状态有效时重新执行
