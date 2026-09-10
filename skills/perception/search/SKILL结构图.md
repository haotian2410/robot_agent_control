# Search Skill 结构图

```text
search/
├── SKILL.md
├── SKILL结构图.md
├── schema.yaml
├── config.yaml
├── selector.py
├── skill.py
├── strategies/
│   ├── __init__.py
│   ├── systematic_scan/
│   │   ├── systematic_scan.md
│   │   ├── systematic_scan.schema.yaml
│   │   └── systematic_scan.py
│   ├── prior_guided_search/
│   │   ├── prior_guided_search.md
│   │   ├── prior_guided_search.schema.yaml
│   │   └── prior_guided_search.py
│   ├── occlusion_aware_view/
│   │   ├── occlusion_aware_view.md
│   │   ├── occlusion_aware_view.schema.yaml
│   │   └── occlusion_aware_view.py
│   └── local_view_refinement/
│       ├── local_view_refinement.md
│       ├── local_view_refinement.schema.yaml
│       └── local_view_refinement.py
├── validators/
│   ├── __init__.py
│   └── viewpoint_validator.py
└── utils/
    ├── __init__.py
    ├── error_codes.py
    ├── frame_transform.py
    ├── history.py
    └── validation.py
```

## 与 Location Skill 同级

```text
skills/
├── location/
│   ├── SKILL.md
│   ├── skill.py
│   └── ...
├── search/
│   ├── SKILL.md
│   ├── skill.py
│   └── ...
├── move/
└── grasp/
```

## Agent 调用流程

```text
Natural-Language Task
        ↓
Location Skill
        ↓
located ──────────────────────────────────→ Return Target Pose
        │
        └─ not_visible / partial / low_quality
                         ↓
                 Agent constructs Search Request
                         ↓
                 SearchSkill.execute(request)
                         ↓
                 Main Schema Validation
                         ↓
                 Runtime Context Acquisition
                         ↓
                 StrategySelector.select()
                         ↓
                 Selected Search Strategy
                         ↓
                 Candidate Viewpoint Generation
                         ↓
                 Optional Legal Fallback
                         ↓
                 ViewpointValidator.validate()
                         ↓
                 Next Viewpoint + Execution Request
                         ↓
                 Move / Camera Controller
                         ↓
                 Capture New Observation
                         ↓
                 Location Skill
```

## 决策边界

```text
Agent
└── 选择 Search Skill，并构造目标、观测、搜索区域和约束

VLM / Perception
├── Visibility estimate
├── Occlusion evidence
├── Image-edge hint
└── Coarse semantic direction

Search Strategy Selector
├── Partial + occlusion → Occlusion-Aware View
├── Visible + low quality → Local View Refinement
├── Not visible + prior → Prior-Guided Search
└── Not visible + no prior + region → Systematic Scan

Viewpoint Validator
├── Frame transform
├── Look-at orientation
├── Field of view
├── History duplicate rejection
├── Reachability
├── Collision / clearance
└── Candidate ranking

Move / Camera Controller
└── Execute the selected control target
```
