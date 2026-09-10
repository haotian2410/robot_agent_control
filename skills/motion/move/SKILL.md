---
id: move
name: move
display_name: Move
version: 1.0.0

description: >
  移动机器人机械臂到指定目标位置、姿态或关节状态，
  仅负责机械臂运动规划与执行。

entrypoint: skill.py
input_schema: schema.yaml
---

# Move Skill

## 当前实现

入口为 `skill.py`，当前实现面向 MuJoCo 中的 UR5e 仿真机械臂。执行顺序固定为：

```text
skill.py
  → utils/validation.py 校验并归一化请求
  → 根目录 utils/ 读取场景与机器人状态
  → selector.py 选择 Joint / Linear / Circular 路径类型
  → direct_planner.py 生成并验证直接路径
  → 根目录 collision/ 检查机器人自碰撞与环境碰撞
  → 路径碰撞时按请求约束选择 RRT-Connect 或返回结构化错误
  → MuJoCo 仿真执行并验证终点状态
```

模块职责：

- `utils/`：加载 MuJoCo 场景、识别 UR5e 关节和执行器、读取状态、执行轨迹；
- `kinematics/`：FK、Jacobian、IKFast 和 TRAC-IK 接口；
- `collision/`：独立 `MjData` 碰撞检测，不改变实时仿真状态；
- `libraries/`：IKFast/TRAC-IK 第三方源码和后端适配；
- `planners/direct_planner.py`：直接路径插值、关节限制和碰撞验证；
- `planners/collision_free_planner.py`：纯 Python 双向 RRT-Connect、捷径平滑和最终复检。

当前 Windows 环境没有可加载的 IKFast 原生扩展，因此显式指定 `ikfast` 时会返回 `IK_FAILED`。`auto` 会先尝试 IKFast，再使用 TRAC-IK 数值路径。当前 TRAC-IK 数值路径使用 MuJoCo Jacobian 多起点阻尼最小二乘兼容实现；迁移 Linux 并编译官方 TRAC-IK 后，只需替换 `libraries` 后端。

默认执行采用 MuJoCo 运动学轨迹回放，保证终点可重复。根目录 `utils.SceneRobotRuntime` 也保留 `execution_mode="actuator"`，用于以后控制器增益调好后的动力学跟踪。

Python 调用示例：

```python
from skills.motion.move.skill import MoveSkill

skill = MoveSkill(scene_path="world_model/robotsim/scene_000.xml")
result = skill.execute({
    "target": {
        "type": "joint",
        "joint_positions": [0.0, -1.2, 1.4, -1.75, -1.57, 0.0],
    },
    "motion": {"path_type": "joint", "path_constraint": "soft"},
    "planning": {"mode": "auto", "allow_replan": True},
    "constraints": {"avoid_collision": True},
})
```

可视化集成测试位于 `tests/move_skill/visual_move_test.py`，所有测试参数位于同级的 `move_test_config.json`。测试在同一窗口中依次执行直线、圆弧和关节角运动；每段轨迹按累计路径长度重采样为配置帧率（默认 60 FPS）。运行：

```powershell
.\.venv\Scripts\python.exe tests\move_skill\visual_move_test.py
```

测试会显示初始姿态并按规划路径流畅播放。直线与圆弧测试结束后，在 MuJoCo 窗口按 Space 或 Enter 才会进入下一项；三项结束后保持窗口打开，直至用户关闭。可在配置中分别修改三项运动的目标、路径参数、`allow_path_change`、规划时间、碰撞要求、帧率、播放速度和相机参数。

## Name
Move

## Description
移动机器人机械臂到指定目标位置、姿态或关节状态，仅负责机械臂运动规划与执行，不涉及目标检测、抓取生成和末端工具操作。

Move Skill 将运动方案拆分为两个独立维度：
- Path Strategy：定义机械臂采用何种路径形式运动
- Planning Mode：定义路径如何生成、验证和避障

## Purpose
用于：
- 移动机械臂末端到指定位置或姿态
- 移动机械臂到指定关节状态
- 执行自由空间转移运动
- 执行抓取前接近和抓取后撤离动作
- 执行放置前定位动作
- 执行插入、拔出、按压等接触运动
- 沿指定圆弧或旋转轴运动
- 返回 Home、Ready 或 Park 状态

不负责：
- 物体检测
- 目标位姿估计
- 抓取姿态生成
- 夹爪或末端工具控制
- 与位移任务无关的独立末端旋转操作
- 高层任务逻辑规划

说明：
Move Skill 可以在移动过程中改变末端姿态，但不负责将单独旋转解释为独立任务行为。

## When To Use
当机器人需要改变机械臂关节状态或末端空间位姿时调用该技能。

典型场景：
- 移动机械臂靠近目标物体
- 移动到预抓取、抓取或放置位置
- 抓取后沿指定方向抬升
- 沿指定方向接近、退出或按压
- 沿已知圆弧或旋转轴运动
- 返回预定义机器人状态
- 在障碍环境中安全移动到目标位置

## Motion Strategy Architecture

完整运动方案由 Path Strategy 和 Planning Mode 共同组成：

```text
Motion Plan = Path Strategy + Planning Mode
```

示例：

```text
Joint Move + Direct Planning
Joint Move + Collision-Free Planning
Linear Move + Direct Planning
Linear Move + Constrained Collision-Free Planning
Circular Move + Direct Planning
```


## Motion Strategies

### Joint Move

关节空间运动。

适用于：
- 目标为明确的关节状态
- Home、Ready、Park 等预定义状态
- 自由空间中的远距离转移
- 无严格末端几何路径要求
- 以效率、速度或可达性为主要目标

特点：
- 直接在关节空间生成轨迹
- 通常具有较高可达性和执行效率
- 不保证末端沿直线或指定几何路径运动
- 可以接收 Joint Target，也可以对 Pose Target 求解目标 IK 后执行

详细策略：
Joint Move

### Linear Move

末端笛卡尔空间直线运动。

适用于：
- 用户明确要求直线运动
- 抓取前沿接近方向移动
- 抓取后沿指定方向撤离或抬升
- 放置时沿指定方向下降
- 插入、拔出、按压等接触操作
- 需要保持末端姿态或运动方向

特点：
- 末端沿直线进行笛卡尔插值
- 可以保持或连续调整末端姿态
- 对连续 IK 和中间路径可达性要求较高
- 路径约束可以声明为 soft 或 hard

详细策略：
Linear Move

### Circular Move

末端笛卡尔空间圆弧运动。

适用于：
- 用户明确要求圆弧运动
- 已提供圆弧中间点
- 已提供旋转中心、旋转轴、半径或旋转角度
- 开门、转动机构或沿圆形边缘运动
- 工艺过程本身要求圆弧轨迹

特点：
- 根据明确的圆弧几何参数生成轨迹
- 保持连续、平滑的曲线路径
- 不作为通用避障策略
- 不得仅因为检测到障碍物而自动选择 Circular Move

详细策略：
Circular Move

## Planning Modes

### Direct Planning

根据已选择的 Path Strategy 直接生成候选轨迹，并执行：
- 关节限制检查
- 速度和加速度检查
- 奇异点检查
- 自碰撞检查
- 环境碰撞检查
- 轨迹连续性检查

当直接候选路径安全可行时使用该模式。

### Collision-Free Planning

当直接路径存在碰撞或不可行，并且任务约束允许改变路径时，搜索安全的替代轨迹。

适用于：
- 直接路径发生碰撞
- 环境中存在复杂障碍物
- 普通自由空间转移需要避障
- 需要搜索替代关节路径

规则：
- 可以与 Joint、Linear 或 Circular Path Strategy 组合
- 必须保留目标状态和必要任务约束
- soft 路径约束允许搜索替代路径
- hard 路径约束不可被静默修改
- 无法满足 hard 路径约束时返回失败

### Replanning

执行过程中检测到动态障碍、场景变化或轨迹失效时，停止当前运动并根据配置重新规划。

## Strategy Selection Policy

Move Skill 根据目标类型、运动意图、任务阶段、实时环境和机器人状态选择运动方案。

选择优先级：

1. 用户或上层任务明确指定的 hard 路径约束优先级最高。
2. Joint Target 默认选择 Joint Move。
3. 明确要求圆弧或提供有效圆弧参数时选择 Circular Move。
4. 明确要求直线，或阶段为 approach、retreat、contact 时优先选择 Linear Move。
5. 普通 transit 阶段且无严格路径要求时，Pose Target 默认优先选择 Joint Move。
6. 所有 Path Strategy 都必须进行碰撞和安全检查。
7. 直接路径安全可行时选择 Direct Planning。
8. 直接路径不可行且路径约束为 soft 时，选择 Collision-Free Planning。
9. 直接路径不可行且路径约束为 hard 时，返回 PATH_CONSTRAINT_INFEASIBLE。
10. 执行过程中检测到动态风险时，停止或减速，并根据配置重新规划。

默认选择逻辑：

```text
Joint Target
→ Joint Move

Pose Target + phase=transit + no hard path constraint
→ Joint Move

Pose Target + phase=approach|retreat|contact
→ Linear Move

Pose Target + valid arc definition
→ Circular Move

Direct path valid
→ Direct Planning

Direct path invalid + path_constraint=soft
→ Collision-Free Planning

Direct path invalid + path_constraint=hard
→ PATH_CONSTRAINT_INFEASIBLE
```

职责边界：
- Agent：理解自然语言并生成结构化目标、阶段和路径约束
- Strategy Selector：根据确定性规则选择 Path Strategy 和 Planning Mode
- Motion Planner：计算 IK 和实际轨迹，并验证可达性、碰撞、关节限制和奇异点
- Safety System：可以否决 Agent 或 Selector 的选择

## Input

### Target
目标机器人状态。

支持：
- Cartesian Pose
- Joint State
- Named Robot State

格式：

```yaml
target:
  type: pose | joint | named_state
```

### Motion
运动意图和路径要求。

支持：
- path_type: auto | joint | linear | circular
- path_constraint: soft | hard
- phase: transit | approach | retreat | contact
- optimization_goal: balanced | fastest | shortest | smoothest | safest

格式：

```yaml
motion:
  path_type: auto
  path_constraint: soft
  phase: transit
  optimization_goal: balanced
```

### Planning
规划模式和重规划要求。

支持：
- mode: auto | direct | collision_free
- allow_replan
- planning_time
- planner_type

格式：

```yaml
planning:
  mode: auto
  allow_replan: true
```

### Constraints
运动约束：
- avoid_collision
- velocity_scale
- acceleration_scale
- position_tolerance
- orientation_tolerance
- safety_distance
- timeout

### Strategy Parameters
策略专用参数：
- cartesian_step
- via_point
- arc_center
- arc_axis
- arc_angle

### Runtime Context
由机器人系统内部提供：
- Current Joint State
- Current End-Effector Pose
- Robot Model
- Planning Scene
- Obstacle State
- Tool and Payload State
- Controller State

## Output
返回：

### Success
执行是否成功。

### Status
状态：
- completed
- failed
- timeout
- stopped
- replanning

### Selection
实际选择结果：
- path_strategy
- planning_mode
- selection_reason
- fallback_used

### Execution Result
执行结果：
- final_joint_state
- final_pose
- position_error
- orientation_error
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
输入字段缺失、参数非法，或策略参数与目标类型不兼容。

Possible Causes:
- Pose Target 缺少位置或姿态信息
- Joint Target 维度与机器人不匹配
- Circular Move 缺少有效圆弧参数
- 用户指定策略不支持当前输入

Recommended Action:
- 根据 schema 修正输入
- 补充目标或策略参数
- 将 path_type 设置为 auto 后重新选择

### Target Unreachable

Description:
目标无法通过当前机器人状态、机器人模型和运动约束达到。

Possible Causes:
- 目标超出机器人工作空间
- 不存在有效 IK 解
- 目标关节状态超过关节限制
- 姿态约束过强

Recommended Action:
- 检查目标位置是否合理
- 请求重新估计目标位姿
- 尝试其他目标姿态或 IK 解
- 在任务允许时放宽 soft constraint

### Path Constraint Infeasible

Description:
目标可能可达，但指定的 hard 几何路径无法安全完成。

Possible Causes:
- 直线路径中间点不可达
- 圆弧几何定义无效
- hard 路径与障碍物冲突
- 连续 IK 不存在

Recommended Action:
- 修改目标或路径几何参数
- 清除障碍物
- 由上层任务决定是否允许放宽约束
- 不得由 Move Skill 静默改变路径类型

### Collision Detected

Description:
候选轨迹或执行中的预测轨迹存在碰撞风险。

Possible Causes:
- 环境存在障碍物
- 存在自碰撞风险
- 场景信息发生变化
- 安全距离不足

Recommended Action:
- 停止当前运动
- 更新 Planning Scene
- 在允许时使用 Collision-Free Planning
- 增大安全距离或重新选择目标

### Planning Failed

Description:
规划器未找到满足目标和约束的有效轨迹。

Possible Causes:
- 搜索空间过大
- 环境过于狭窄
- 规划时间不足
- 所有候选 IK 或路径均无效

Recommended Action:
- 增加 planning_time
- 尝试其他 IK 解或规划器
- 在任务允许时调整 soft constraint
- 请求上层任务重新定位中间目标点

### Execution Error

Description:
运动执行阶段发生异常。

Possible Causes:
- 控制器异常
- 硬件或通信故障
- 跟踪误差超过阈值
- 动态障碍物进入运动区域

Recommended Action:
- 安全停止机械臂
- 保存当前机器人状态
- 检查控制器和硬件状态
- 更新环境后重新规划
- 仅在错误可恢复且状态安全时重新执行
