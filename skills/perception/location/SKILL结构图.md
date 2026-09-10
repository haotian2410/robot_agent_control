# Locate Skill 结构图

```text
location/
├── SKILL.md
├── SKILL结构图.md
├── schema.yaml
├── config.yaml
├── selector.py
├── skill.py
├── strategies/
│   ├── __init__.py
│   ├── known_model_locate/
│   │   ├── known_model_locate.md
│   │   ├── known_model_locate.schema.yaml
│   │   └── known_model_locate.py
│   ├── known_category_locate/
│   │   ├── known_category_locate.md
│   │   ├── known_category_locate.schema.yaml
│   │   └── known_category_locate.py
│   ├── unknown_object_locate/
│   │   ├── unknown_object_locate.md
│   │   ├── unknown_object_locate.schema.yaml
│   │   └── unknown_object_locate.py
│   ├── grasp_pose_prediction/
│   │   ├── grasp_pose_prediction.md
│   │   ├── grasp_pose_prediction.schema.yaml
│   │   └── grasp_pose_prediction.py
│   ├── approach_point_locate/
│   │   ├── approach_point_locate.md
│   │   ├── approach_point_locate.schema.yaml
│   │   └── approach_point_locate.py
│   └── relative_point_locate/
│       ├── relative_point_locate.md
│       ├── relative_point_locate.schema.yaml
│       └── relative_point_locate.py
├── validators/
│   ├── __init__.py
│   └── pose_validator.py
└── utils/
    ├── __init__.py
    ├── error_codes.py
    ├── frame_transform.py
    └── validation.py
```

## Agent 调用流程

```text
Natural-Language Task
        ↓
Agent constructs Locate Request
        ↓
LocateSkill.execute(request)
        ↓
Main Schema Validation
        ↓
Runtime Context Acquisition
        ↓
StrategySelector.select()
        ↓
Selected Locate Strategy
        ↓
Candidate Pose Generation
        ↓
Optional Legal Fallback
        ↓
PoseValidator.validate()
        ↓
Structured Localization Result
        ↓
Move / Grasp / VLA / ACT Skill
```

## 策略选择边界

```text
Agent
└── 选择 Locate Skill，并构造目标、参考、偏移和约束

Locate Strategy Selector
├── Relative target  → Relative Point Locate
├── Approach target  → Approach Point Locate
├── Model prior      → Known Model Locate
├── Grasp target     → Grasp Pose Prediction
├── Category prior   → Known Category Locate
└── Open description → Unknown Object Locate

Pose Validator
├── Frame transform
├── Pose completeness
├── Confidence threshold
├── Reachability
├── Collision / clearance
└── Candidate ranking
```
