# Skill-command demo

This directory is the executable entry point for the legacy command demo.
`config_001.commands.json` is a versioned command document: each item has a
`skill_name` and JSON `parameters`, and the optional `registry` is resolved
relative to this file.

The command names map to the canonical skills as follows:

- `move`: `skills/motion/move/SKILL.md`
- `locate` / `search`: `skills/perception/location/SKILL.md` and `skills/perception/search/SKILL.md`
- `grasp`: `skills/manipulation/grasp/SKILL.md`
- `release`: `skills/manipulation/release/SKILL.md`
- `pull` / `push`: `skills/manipulation/push_pull/SKILL.md`
- `press`: `skills/manipulation/press/SKILL.md`

Run the fixture without opening a viewer:

```bash
conda run -n robot_agent_integ robot-agent-control-demo \
  --commands demo/skill_command/config_001.commands.json \
  --viewer-mode headless
```

The runner keeps one MuJoCo model/data runtime for the whole document and
writes a structured execution report and skill trace.  The simulator's
`compile` command emits the same command-document format; `run_demo.py` is a
legacy wrapper around this executor.
