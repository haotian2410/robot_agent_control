---
id: search.systematic_scan
name: systematic_scan
display_name: Systematic Scan
parent_skill: search
version: 1.0.0

description: >
  在没有可靠目标位置先验时，根据预定义搜索区域生成可重复的覆盖式相机扫描视角。

input_schema: strategies/systematic_scan/systematic_scan.schema.yaml
implementation: strategies/systematic_scan/systematic_scan.py
---

# Systematic Scan Strategy

## Name
Systematic Scan

## Description
在没有可靠目标位置先验时，根据预定义搜索区域生成可重复的覆盖式相机扫描视角。

## When To Use
适用于：
- 目标完全不在视野
- 不存在可靠位置或方向先验
- 已知工作空间、命名区域或预定义视角集
- 需要确定性覆盖搜索

不适用于：
- 目标已经部分可见并存在明确遮挡关系
- 缺少任何可搜索区域或可移动观察设备

## Input Requirements
- `search_region.type` 或运行时默认搜索区域
- 当前相机位姿和相机模型
- 腕部相机需要 Camera-to-End-Effector 外参
- 搜索历史

## Strategy Parameters
- `scan_pattern`: `horizontal | vertical | z_scan | grid | orbit | viewpoint_set`
- `scan_axis`
- `scan_step`
- `row_step`
- `candidate_count`
- `look_at_mode`
- `skip_visited`

## Execution Logic
1. 解析搜索区域并裁剪到设备可用范围。
2. 根据扫描模式生成有序候选视角。
3. 为每个候选生成相机朝向或 look-at 目标。
4. 根据 Search History 去除已访问和失败视角。
5. 输出候选视角、覆盖进度和扫描索引。
6. 交由统一 Viewpoint Validator 完成最终验证和排序。

## Output
返回：
- candidate_viewpoints
- expected_visibility_gain
- expected_information_gain
- source = systematic_scan
- evidence
- search_progress

## Failure Handling
- `SEARCH_REGION_INVALID`: 搜索区域无效
- `VIEWPOINT_GENERATION_FAILED`: 扫描参数无法生成候选
- `SEARCH_EXHAUSTED`: 所有扫描视角均已访问
- `NO_VALID_VIEWPOINT`: 候选均未通过验证

本文件只描述策略语义；候选生成、射线投射、视场计算、几何分析和机器人验证实现由用户接入。
