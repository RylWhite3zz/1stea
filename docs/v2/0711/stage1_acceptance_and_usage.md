# 阶段 1 验收、运行与诊断约定

## 1. 验收结论的范围

本文件只验收 `franka_allegro_mujoco` 的模型级能力。四项主 gate 为：

```text
G1 组合 MJCF 可编译且结构完整
G2 canonical q 合法、稳定、无未声明自碰撞
G3 frame/FK 正确
G4 7+16 actuator 可独立小幅动作
```

四项全部通过后，允许进入 phase/IK/规划研究；不能据此宣称任何 probe 或 manipulation
已可由 Panda 执行。

---

## 2. G1：模型编译和结构验收

### 2.1 必须成立

固定基座、无任务物体的独立机器人模型应满足：

| 项目 | 预期 |
| --- | --- |
| backend ID | `franka_allegro_mujoco` |
| mount profile | `sim.synthetic_panda_allegro_mount.v1` |
| `nq / nv / nu` | `23 / 23 / 23` |
| Panda joint/actuator | 7 / 7，名称与索引完整唯一 |
| Allegro joint/actuator | 16 / 16，名称与索引完整唯一 |
| carriage joint/actuator | 0；不得出现 `wx/wy/wz/wr/wt/wyaw` 或对应 actuator |
| 必要 site | attachment、protocol wrist、palm、四个 fingertip |
| collision geom | arm、mount、palm、各手指刚体均有可审计 proxy |
| keyframe | 完整 canonical `qpos23` 与 `ctrl23` |

还应验证：

- 所有 mesh 和 material 引用可从任意当前工作目录解析；
- asset、body、joint、geom、site、actuator 名称无冲突；
- 编译产物和 diagnostic 中记录模型/profile provenance；
- 重复构建不会因 XML 元素顺序或全局缓存改变 ID 集合。

### 2.2 应拒绝的负例

- Menagerie root 不存在或缺少任一源模型；
- synthetic mount profile 未知；
- 组合 keyframe 少于或多于 23 维；
- joint/actuator 名称缺失、重复或映射到错误 group；
- asset prefix/reference 重写不完整；
- 把第三 backend 传给只接受 carriage scene 的 `SceneConfig`；
- 把阶段 1 backend 交给 `ProbeHarness` 执行四个动作。

错误应在构建/admission 时给出明确原因，不能等到第一个 simulation step 才报 KeyError。

---

## 3. G2：canonical 与碰撞验收

### 3.1 canonical 基础检查

最终冻结的 `q23` 和 `ctrl23` 应满足：

1. 全部值有限；
2. 每个受限 joint 和 actuator 均在 range 内；
3. 每个 range margin 都进入 diagnostic；
4. `reset()` 后 q/ctrl 与 keyframe 一致；
5. `mj_forward` 后 position、quaternion、velocity、force 均有限；
6. 短时 settle 后没有无界速度、控制发散或姿态漂移。

canonical 的来源与候选值见
[stage1_franka_allegro_backend_design.md](stage1_franka_allegro_backend_design.md)。测试应读取
代码中的权威 profile，不要在测试中复制另一组数字后形成双真值。

### 3.2 碰撞报告至少包含

```text
contacting body/geom pair
pair role: structural / excluded / forbidden
signed distance or penetration depth（可计算时）
normal force
maximum penetration pair
minimum audited clearance pair
simulation step/time
```

通过条件：

- canonical `mj_forward` 后无 forbidden self-contact；
- settle 窗口中无 forbidden self-contact；
- arm-hand、mount-hand、非相邻 arm 以及非相邻 finger 的 audited pair 无未声明穿透；
- structural overlap 只出现在显式小型白名单，且每项有固定接口理由；
- 不能通过扩大 `exclude`、关闭 collision mask 或删除 collision geom 获得绿色结果。

特别注意：`data.ncon == 0` 只能作为其中一个观测，不是完整证明。测试必须覆盖被 MuJoCo
父子/welded 过滤或显式 exclude 的危险近邻，至少包括 `link7 ↔ mount/palm/finger base`。

### 3.3 canonical 失败时的修复顺序

```text
先检查 frame 和 mount 方向
→ 再检查 collision proxy 是否过度保守或建模错误
→ 再检查结构接口白名单
→ 最后才在合理范围内调整 canonical q
```

禁止用任意弯曲手指、删除 palm collision 或全局排除 arm-hand collision 来隐藏错误安装。

---

## 4. G3：FK/site 验收

### 4.1 attachment 保真测试

选择 canonical 和若干不接近限位的 Panda `q7` 样本。对每个样本：

1. 在原始 `panda_nohand.xml` 中计算 `attachment_site` world pose；
2. 在组合模型中设置相同 `q7` 和固定 canonical hand q；
3. 比较组合模型原始 attachment site pose。

它验证组合过程没有改变 Panda kinematic tree、joint 顺序或 attachment frame。

### 4.2 固定链组合测试

从独立的齐次变换组合计算：

```text
T_world_attachment × T_attachment_adapter × T_adapter_site
```

分别与 MuJoCo 中的 `protocol_wrist`、`allegro_palm` 和四个 fingertip site 比较。建议同一
MuJoCo 双精度链路采用：

```text
position error <= 1e-8 m
rotation geodesic error <= 1e-8 rad
```

若实现使用另一独立 FK 库，可单独设置合理但仍显式的 tolerance；不能静默放宽。

### 4.3 方向语义的人工检查

数值测试之外，viewer smoke 应检查：

- 右手而不是镜像左手；
- 拇指和食指方位与 site 命名一致；
- `protocol_wrist` 坐标轴可视化与文档定义一致；
- mount、link7、palm 没有明显穿模；
- canonical 不依赖地面或任务物体支撑。

人工 viewer 只用于发现语义/视觉错误，不能替代自动 FK 和碰撞 gate。

---

## 5. G4：7+16 actuator 独立小动作

### 5.1 测试目的

这个 gate 只证明：名称、索引、ctrl slice、方向和基础 position servo 接线正确。它不验证
最终控制品质，也不说明该 trajectory 满足接触任务或实机限制。

### 5.2 单 actuator 流程

对 7 个 Panda actuator 和 16 个 Allegro actuator 逐个执行：

```text
canonical reset + settle
→ 选择不越 joint/ctrl range 的 +delta
→ 只改变被测 actuator target
→ 运行固定短窗口
→ 记录 q/dq/ctrl/contact
→ 回 canonical 并 settle
→ 对可行方向重复 -delta
```

建议初始扰动：

```text
arm:  0.01 rad
hand: 0.02 rad
```

若该方向的 limit margin 不足，应按 margin 缩小或只测另一方向，并在结果中说明，不得
clip 后仍报告原始 delta。

### 5.3 通过条件

- 只有目标 ctrl 元素改变，另一 group 的 ctrl slice 保持逐元素不变；
- 目标 joint 实际位移方向与命令一致，且绝对位移大于数值噪声阈值；
- 其他 group 可以因动力学发生微小状态响应，但不得收到隐藏控制命令；
- q/dq/ctrl/actuator force 全部有限；
- joint/ctrl 不越限；
- 不新增 forbidden self-contact；
- 回零阶段能重新达到 canonical tolerance；
- 每个 actuator 单独输出 pass/fail 和测量值，不能只用“23 个循环没有异常”作为证明。

应另加 group smoke：同时给 7 维 arm target 时 hand target 不变，同时给 16 维 hand target
时 arm target 不变。它用于验证 API slice，而不是替代逐 actuator 测试。

---

## 6. 后端 API 的阶段 1 使用方式

当前阶段 1 API 的最小使用方式如下：

```python
from allegro_probe.backends import FrankaAllegroMujocoBackend

backend = FrankaAllegroMujocoBackend.create()
assert backend.scene.mount_profile.profile_id == (
    "sim.synthetic_panda_allegro_mount.v1"
)

backend.reset()
q_arm0 = backend.arm_qpos
q_hand0 = backend.hand_qpos

backend.command_arm_joints(q_arm_target)   # 7 values
backend.step(n_steps)

backend.command_hand_joints(q_hand_target) # 16 values
backend.step(n_steps)

pose = backend.frame_pose("protocol_wrist")
contacts = backend.collision_snapshot()
clearance = backend.distance_audit()
```

必要约束：

- `command_arm_joints` 与 `command_hand_joints` 按名称映射到各自 slice；
- 长度、非有限值和越限命令在写 `data.ctrl` 前拒绝；
- reset 由 backend 权威 canonical profile 驱动；
- q/frame/contact 读数能追溯到 scene 的 profile/frame 语义；
- 这些低层方法不接受 `ProbeCommand` 或 `ManipulationPlan`。

---

## 7. 运行和测试命令

安装与全量回归沿用仓库现有方式：

```bash
conda run -n probebench python -m pip install -e .
conda run -n probebench python -m pytest -q
```

阶段 1 的专属自动 gate 为：

```bash
# 自动 gate：compile/canonical/FK/collision/23-actuator smoke
conda run -n probebench python -m pytest -q tests/test_franka_scene.py
```

backend capability/admission 重构由独立测试覆盖：

```bash
conda run -n probebench python -m pytest -q tests/test_backends.py
```

另有一个结构化 smoke 入口，会执行 canonical 碰撞审计以及全部 23 个 actuator 的正负
小扰动，并以 JSON 输出结果：

```bash
conda run -n probebench python -m examples.run_franka_allegro_stage1

# 可选人工观察；自动验收不依赖 viewer
conda run -n probebench python -m examples.run_franka_allegro_stage1 --viewer
```

`--viewer` 只用于检查手性、方向和明显穿模；无图形环境下运行默认命令和专属测试即可。

---

## 8. 诊断输出和完成证据

建议 smoke 入口输出一份结构化摘要：

```json
{
  "backend": "franka_allegro_mujoco",
  "mount_profile_id": "sim.synthetic_panda_allegro_mount.v1",
  "model_provenance": {
    "mujoco_version": "...",
    "panda_xml_sha256": "...",
    "allegro_xml_sha256": "...",
    "controller_profile_id": "sim.panda_menagerie_pd+allegro_position.v1"
  },
  "dimensions": {"nq": 23, "nv": 23, "nu": 23},
  "canonical": {
    "passed": true,
    "minimum_joint_limit_margin_rad": 0.0,
    "minimum_ctrl_limit_margin_rad": 0.0,
    "forbidden_contacts": [],
    "forbidden_penetrations": []
  },
  "frames": {"attachment": {"position_m": [], "quaternion_wxyz": []}},
  "actuator_smoke": {"passed": true, "checks": []}
}
```

字段中的零只是格式示意，不是预填测试结果。阶段完成证据应同时包括：

1. 针对第三 backend 的测试通过；
2. 原有全量测试通过，证明两个 carriage backend 未回归；
3. 结构化 smoke 结果保存模型/profile provenance；
4. 文档运行命令与实际 CLI 一致。

viewer 人工检查是推荐补充证据，不是无图形环境下的完成前置条件。

### 8.1 本次本地验收结果

2026-07-11 在 MuJoCo `3.10.0`、Menagerie revision
`71f066ad0be9cd271f7ed58c030243ef157af9f4` 上得到：

```text
nq / nv / nu                         23 / 23 / 23
canonical forbidden contact          0
canonical forbidden penetration      0
minimum non-filtered clearance       0.003165807 m
actuator direction/isolation checks  46 / 46 passed
named FK frames                      10
```

该净空属于 `sim.synthetic_panda_allegro_mount.v1`，不是实机 adapter 公差。

只有以上证据齐全，才能把阶段 1 标为完成。
