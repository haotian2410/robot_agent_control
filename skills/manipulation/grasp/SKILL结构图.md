# Grasp Skill 结构图

```text
grasp/
├── SKILL.md
├── SKILL结构图.md
├── schema.yaml
├── config.yaml
├── selector.py
├── skill.py
├── strategies/
│   ├── __init__.py
│   └── default_grasp/
│       ├── default_grasp.md
│       ├── default_grasp.schema.yaml
│       └── default_grasp.py
├── validators/
│   ├── __init__.py
│   └── grasp_validator.py
└── utils/
    ├── __init__.py
    ├── error_codes.py
    └── validation.py
```

## 与其他 Skill 同级

```text
skills/
├── location/
├── search/
├── move/
├── grasp/
└── press/
```

## Agent 调用流程

```text
Location / Grasp Planning
        ↓ target grasp pose
Move Skill
        ↓ robot reaches grasp pose
Agent constructs Grasp Request
        ↓
GraspSkill.execute(request)
        ↓
Main Schema Validation
        ↓
Runtime Context Acquisition
        ↓
StrategySelector.select()
        ↓
GraspValidator.validate_preconditions()
        ↓
DefaultGrasp.execute()
        ↓
GraspValidator.validate_result()
        ↓
Grasp Result
        ↓
Move Skill performs lift / retreat
```

## 决策边界

```text
Agent
└── 选择 Grasp Skill，并构造目标、抓取配置、约束和策略参数

Strategy Selector
└── auto / explicit → Default Grasp

Grasp Strategy
├── Open
├── Close
├── Contact detection
├── Apply grasp force
└── Hold

Grasp Validator
├── Controller readiness
├── Width / force limits
├── Pose alignment (optional)
├── Verification evidence
└── Slip rejection

Move Skill
└── Approach, lift and retreat; never delegated to Grasp
```
