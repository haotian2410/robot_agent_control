## Move Skill 结构
```text
move/
├── SKILL.md
├── schema.yaml
├── config.yaml
├── skill.py
├── selector.py
│
├── strategies/
│   ├── __init__.py
│   │
│   ├── joint_move/
│   │   ├── joint_move.md
│   │   ├── joint_move.schema.yaml
│   │   └── joint_move.py
│   │
│   ├── linear_move/
│   │   ├── linear_move.md
│   │   ├── linear_move.schema.yaml
│   │   └── linear_move.py
│   │
│   └── circular_move/
│       ├── circular_move.md
│       ├── circular_move.schema.yaml
│       └── circular_move.py
│
├── planners/
│   ├── __init__.py
│   ├── direct_planner.py
│   └── collision_free_planner.py
│
└── utils/
    ├── __init__.py
    ├── validation.py
    └── error_codes.py
```
## Move Skill 主文件
```text
move/
├── SKILL.md
├── schema.yaml
├── config.yaml
├── skill.py
└── selector.py
```
- SKILL.md：描述 Move Skill 的能力边界、输入输出、Strategy 选择规则和 Failure Handling。
- schema.yaml：定义 Move Skill 的统一输入和输出格式。
- config.yaml：保存全局默认参数、安全参数和各 Strategy 默认配置。
- skill.py：Move Skill 的统一执行入口，负责校验、选择、规划、执行和错误处理。
- selector.py：根据目标、任务阶段、路径约束和机器人状态选择 Path Strategy。


## Move Skill 执行顺序
```text
skill.py
   │
   ├── validation.py
   │
   ├── selector.py
   │      └── Joint / Linear / Circular
   │
   ├── selected strategy
   │      └── generate path requirement
   │
   ├── direct_planner.py
   │
   ├── collision check
   │      │
   │      └── failed
   │             ↓
   │         collision_free_planner.py
   │
   └── robot execution
```

## 完整调用流程
```text
用户任务
   ↓
Agent 理解任务
   ↓
任务分解
   ↓
判断当前步骤是否需要机械臂移动
   ↓
获取移动目标
   ↓
构造 Move Request
   ↓
调用 Move Skill
   ↓
输入校验和坐标转换
   ↓
选择 Path Strategy
   ↓
生成直接候选轨迹
   ↓
IK、关节限制、奇异点、碰撞检查
   ↓
轨迹是否有效？
   ├── 是 → 执行轨迹
   └── 否 → 判断是否允许改变路径
               ├── 允许 → Collision-Free Planner 重新规划
               └── 不允许 → 返回失败
   ↓
执行期间监控
   ↓
检查最终位置误差
   ↓
返回结构化结果
   ↓
Agent 根据结果继续下一步或恢复
```