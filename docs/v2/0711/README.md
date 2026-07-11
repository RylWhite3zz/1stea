# Panda+Allegro 第三 backend：阶段 1 文档索引

本目录是 `franka_allegro_mujoco` 的当前实施依据：

- [stage1_franka_allegro_backend_design.md](stage1_franka_allegro_backend_design.md)：为什么要拆独立 scene、第三 backend 怎样做 capability admission、MJCF/mount/canonical/frame/碰撞边界。
- [stage1_acceptance_and_usage.md](stage1_acceptance_and_usage.md)：compile、canonical、FK、碰撞和 7+16 actuator 的验收标准、API 与运行命令。
- [next_stage_research.md](next_stage_research.md)：进入完整动作前必须完成的 phase IR、连续 IK、路径、碰撞 proxy、时间参数化、接触控制和 wrist F/T 调研。

阶段 1 只证明模型与底层 joint 接口成立。第三 backend 当前明确声明
`supported_primitives=frozenset()`；`poke/slide/manipulation/heft/shake` 尚未迁移。

快速验收：

```bash
conda run -n probebench python -m pytest -q \
  tests/test_backends.py tests/test_franka_scene.py
conda run -n probebench python -m examples.run_franka_allegro_stage1
```
