# robot-agent-control

> **Moved:** active development has moved to [`haotian2410/robot_agent_stack`](https://github.com/haotian2410/robot_agent_stack). This repository is retained as a compatibility/frozen source; new features and fixes belong in the monorepo.

`robot-agent-control` 是 `robot-agent-sim` 的执行层：负责把 versioned command document
转换成 MuJoCo 中的 IK、轨迹、夹爪和交互动作。它不负责自然语言理解、Qwen 调用、Route A/B
选择或场景/模型检索。

## 与 sim 的关系

```text
robot_agent_sim: instruction → TaskIntent → GroundedTask → SkillPlan → commands.json
                                                                        ↓
robot-agent-control: preflight → IK/skills → one persistent MuJoCo runtime → report/trace
```

部署运行控制链时需要两个仓库，并安装到同一个克隆环境：

```bash
conda create -n robot_agent_integ --clone robot_agent
conda activate robot_agent_integ
python -m pip install -e /home/cscvlab/lht/robot-agent-control
python -m pip install -e /home/cscvlab/lht/robot_agent_sim
```

独立验证 legacy fixture：

```bash
robot-agent-control-demo \
  --commands /home/cscvlab/lht/robot-agent-control/demo/skill_command/config_001.commands.json \
  --viewer-mode headless
```

端到端整合、Route A/B、Qwen 和 viewer 模式请看
[robot_agent_sim/INTEGRATION.md](https://github.com/haotian2410/robot_agent_sim/blob/main/INTEGRATION.md)。
`demo/skill_command/SKILL.md` 是 `config_001` 的技能命令入口；各技能的详细协议仍位于
`skills/*/SKILL.md`。
