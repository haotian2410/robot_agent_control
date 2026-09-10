# Skill Command 技能列表

当前 `skill_command` 支持以下 5 个 skill。所有命令都必须包含 `parameters.target`。

| Skill | 必填参数 | 可选参数 | 描述 |
| --- | --- | --- | --- |
| `move` | `target` | `planning_method`、`keep_end_effector_orientation`、`relation`、`distance_m` | 移动到目标；发生碰撞时自动切换避障规划 |
| `grasp` | `target` | 无 | 闭合夹爪抓取目标 |
| `release` | `target` | 无 | 松开夹爪释放目标 |
| `pull` | `target` | 无 | 拉动目标，例如打开柜门 |
| `push` | `target` | 无 | 推动目标，例如关闭柜门 |

## 通用命令格式

```json
{
  "skill_name": "技能名称",
  "parameters": {
    "target": "目标"
  }
}
```

`target` 可以填写场景对象 ID、场景别名、命名目标，或者目标对象结构。例如：

```json
{
  "skill_name": "locate",
  "parameters": {
    "target": "red_ball"
  }
}
```

## 各 skill 示例

### locate

从场景真值中获取目标的位置、姿态和相关空间信息。

```json
{
  "skill_name": "locate",
  "parameters": {
    "target": "red_ball"
  }
}
```

### move

将机械臂直接移动到目标位置。仿真中的目标信息由场景注册表统一提供；发生碰撞时会自动切换到对应空间的 RRT 避障规划。

```json
{
  "skill_name": "move",
  "parameters": {
    "target": "blue_cabinet_handle",
    "planning_method": "linear"
  }
}
```

在`parameters`中`target`为必要变量，其余可省略。

支持的 `planning_method` 值：

- `linear`：直线规划
- `circular`：圆弧规划
- `joint`：关节空间规划

`keep_end_effector_orientation` 用于指定是否保持末端姿态不变：

```json
{
  "skill_name": "move",
  "parameters": {
    "target": "red_ball",
    "planning_method": "linear",
    "keep_end_effector_orientation": true
  }
}
```

当需要执行抓取后的上抬、下降或撤离等末端相对运动时，将目标写为 `end_effector`，并使用 `relation` 与 `distance_m`：

```json
{
  "skill_name": "move",
  "parameters": {
    "target": "end_effector",
    "relation": "above",
    "distance_m": 0.125,
    "planning_method": "linear"
  }
}
```

支持的末端相对方向包括：

- `above`：上抬
- `below`：下降
- `front`：前移
- `behind`：后撤
- `left`：左移
- `right`：右移

### grasp

闭合夹爪并抓取目标物体。

```json
{
  "skill_name": "grasp",
  "parameters": {
    "target": "red_ball"
  }
}
```

### release

松开夹爪并释放当前抓取的目标。

```json
{
  "skill_name": "release",
  "parameters": {
    "target": "red_ball"
  }
}
```

### pull

拉动目标，通常用于打开柜门、抽屉等可移动部件。

```json
{
  "skill_name": "pull",
  "parameters": {
    "target": "blue_cabinet_door"
  }
}
```

### push

推动目标，通常用于关闭柜门、抽屉等可移动部件。

```json
{
  "skill_name": "push",
  "parameters": {
    "target": "blue_cabinet_door"
  }
}
```

## 写入 `commands.json`

多个 skill 命令放在 `commands` 数组中，按数组顺序执行：

```json
{
  "registry": "../common/scenes/scene_001.interactions.json",
  "commands": [
    {
      "skill_name": "move",
      "parameters": {
        "target": "red_ball",
      }
    },
    {
      "skill_name": "grasp",
      "parameters": {
        "target": "red_ball"
      }
    }
  ]
}
```
