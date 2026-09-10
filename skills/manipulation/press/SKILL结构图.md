# Press Skill 结构图

```text
press/
├── SKILL.md
├── SKILL结构图.md
├── schema.yaml
├── config.yaml
├── selector.py
├── skill.py
├── strategies/
│   ├── __init__.py
│   ├── displacement_press/
│   │   ├── displacement_press.md
│   │   ├── displacement_press.schema.yaml
│   │   └── displacement_press.py
│   └── force_controlled_press/
│       ├── force_controlled_press.md
│       ├── force_controlled_press.schema.yaml
│       └── force_controlled_press.py
├── validators/
│   ├── __init__.py
│   └── press_validator.py
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
Location Skill
        ↓ press point + press direction + pre-press pose
Move Skill
        ↓ reaches pre-press pose
Agent constructs Press Request
        ↓
PressSkill.execute(request)
        ↓
Main Schema Validation
        ↓
Runtime Context Acquisition
        ↓
StrategySelector.select()
        ↓
PressValidator.validate_preconditions()
        ↓
DisplacementPress / ForceControlledPress
        ↓
PressValidator.validate_result()
        ↓
Press Result + Retract State
        ↓
Move Skill performs any further free-space retreat
```

## 决策边界

```text
Agent
└── 选择 Press Skill，并构造目标、控制模式、约束和验证要求

Location Skill
├── Press point
├── Surface normal / press direction
└── Pre-press pose

Strategy Selector
├── target_force + valid force feedback → Force-Controlled Press
└── press_depth / travel_distance → Displacement Press

Press Strategy
├── Establish contact
├── Execute bounded local contact motion
├── Hold
├── Verify
└── Retract along the original axis

Press Validator
├── Direction and frame validity
├── Controller / sensor readiness
├── Maximum force / travel
├── Target actuation evidence
└── Retract result

Move Skill
└── Free-space approach and later retreat; no contact execution
```
