# 当前 Probe → Manipulation 数据流与控制信号全链路

> 本文的 manipulation 半段记录原始 canonical `short_can_pick_place` 纵向切片；
> probe 半段已同步为当前 full-collision top-pinch heft。v1 manipulation 只能在显式
> partial-collision 兼容 scene 中执行，因此示例现在使用两个独立 MuJoCo model。
> 根据实时物体位姿计算
> wrist 目标、使用完整手部碰撞并放到固定绝对位置的新接口见
> [learning_free_pose_pick_place.md](learning_free_pose_pick_place.md)。两条路径并存，
> 不应把本文的托架/相对放置语义套到 pose-conditioned v2。

## 1. 文档目的与状态

本文只描述仓库中截至 2026-07-10 **已经实际跑通** 的这一条纵向链路：

```text
mass 场景中的指定 short_can
→ Allegro heft probe
→ ProbeResult
→ short_can_pick_place plan admission
→ ManipulationContext + ManipulationPlan
→ canonical reset
→ 6-DoF 理想腕部 + 16-DoF Allegro 闭环执行
→ ManipulationExecutionResult
```

本文不是未来通用 manipulation 架构提案。它以以下代码为唯一事实来源：

- `examples/run_short_can_pick_place.py`
- `allegro_probe/interfaces.py`
- `allegro_probe/primitives.py`
- `allegro_probe/manipulation.py`
- `allegro_probe/scene.py`
- `tests/test_manipulation.py`

与已有文档的关系：

- `../sgst_mani_2_dataflow.md` 描述的是早期建议的数据流，包含尚未落地的通用层。
- `../sgst_mani_3_short_can_pick_place.md` 记录当前纵向切片的方案、阈值与验证结果。
- 本文进一步沿实际函数调用，解释每个数据字段和控制信号如何产生、何时被消费、
  如何通过反馈改变下一步命令。

当前能力的准确名称是：

> 规范化 MuJoCo 场景中，由可信 Allegro heft 结果条件化的 short_can 固定模板
> pick/place 闭环执行。

它不是通用抓取、机械臂规划、视觉定位、VLA policy 或完整 ProbeBench solver。

---

## 2. 先明确当前链路的边界

### 2.1 已经打通的内容

1. `heft` 会真实执行 Allegro 抓取、抬升和腕部力传感采样。
2. `heft` 会生成独立的 `ProbeResult`，而不是直接返回 manipulation 命令。
3. plan builder 会校验 `ProbeResult`，并用其中的质量相关信号生成：
   - 目标手—物体总法向力；
   - 最大腕部平移速度；
   - 轻物体或普通/重物体的放置分支。
4. manipulation 会显式下发 16 个 Allegro 关节位置目标。
5. 腕部轨迹在执行时按阶段生成，并通过每步接触反馈做抓力调节和安全停止。
6. 放置结束后会检查位置、倾角、漂移、桌面支撑、手是否退出以及全过程安全峰值。

### 2.2 没有打通的内容

1. 没有根据 instruction 自动选择 target；示例中的 `--target` 已由外部明确给出。
2. 没有机械臂，腕部由理想 6-DoF carriage 直接驱动。
3. 没有视觉估计；对象 pose 来自 MuJoCo scene sensor，属于 oracle pose。
4. 没有通用 skill selector；只有 `short_can_pick_place`。
5. 没有从 probe 后的 live physics state 继续执行；probe 和 v1 manipulation 使用两个
   配置不同的 scene，后者会 reset 到规范初态。
6. 没有把 `ProbeResult.trace` 编译成 action chunk 或模仿轨迹。
7. 没有 reference gripper manipulation；这一动作只接受 Allegro backend。

因此当前的“probe → manipulation 连续”具体是：

```text
ProbeResult 数据连续
但 MuJoCo 物理状态不连续
```

probe 的测量结果被保留并用于生成 plan；probe 后已经扰动的手和物体状态不会直接传给
manipulation executor。

---

## 3. 运行时对象和总调用链

示例程序的实际调用链如下：

```text
examples.run_short_can_pick_place.main()
│
├─ make_demo_scene("mass", candidates, seed)
│    └─ ProbeSceneSpec + ObjectSpec[]
│
├─ AllegroHandBackend.create(spec)                 # 安全 probe scene
│    └─ support-free + full hand collision
│
├─ ProbeHarness(backend).execute(ProbeCommand("heft", target))
│    └─ run_probe()
│         └─ heft()
│              └─ ProbeResult
│
├─ AllegroHandBackend.create(                      # 显式 v1 compatibility scene
│      spec, allegro_grasp_lift=0.09,
│      full_hand_collisions=False,
│      wrist_roll_limit_rad=0.9,
│  )
│
├─ build_short_can_pick_place_plan(
│      legacy_scene,
│      probe_result,
│      ShortCanPickPlaceRequest(target),
│  )
│    └─ ManipulationPlanDecision
│         ├─ ManipulationContext
│         └─ ManipulationPlan
│
└─ execute_short_can_pick_place(legacy_scene, decision.plan)
     ├─ canonical reset
     ├─ phase-by-phase command generation
     ├─ MuJoCo step + contact feedback
     └─ ManipulationExecutionResult
```

其中不存在一个隐藏的通用 `ManipulationController`。`interfaces.py` 中的
`ManipulationCommand` 和 `ManipulationController` 仍是未来占位接口；当前真实执行入口是
类型化函数：

```python
build_short_can_pick_place_plan(...)
execute_short_can_pick_place(...)
```

---

## 4. 数据在各层之间如何分工

当前链路中有五类关键数据对象：

| 数据对象 | 谁产生 | 谁消费 | 作用 | 是否包含高频命令 |
|---|---|---|---|---|
| `ProbeCommand` | 示例/上层 | `ProbeHarness` | 指定 `heft` 和 target | 否 |
| `ProbeResult` | `heft()` | plan builder | 携带 probe 事实、估计与可信性 | 否 |
| `ShortCanPickPlaceRequest` | 示例/上层 | plan builder | 携带 manipulation intent | 否 |
| `ManipulationContext` | plan builder | 日志/上层 | 记录此次 plan 的事实依据 | 否 |
| `ManipulationPlan` | plan builder | executor | 固定模板、阈值、阶段和分支 | 否 |
| `ManipulationExecutionResult` | executor | 示例/测试/上层 | 报告执行结果、质量和失败原因 | 否 |

真正的高频控制信号不会提前塞进 `ManipulationPlan`。它们在 executor 的每个 phase 中，
根据 plan、当前 actuator target 和最新反馈在线生成：

```text
ManipulationPlan
+ 当前 phase
+ 当前 wrist/hand control target
+ 当前 object pose
+ 当前 ContactSnapshot
→ 下一仿真步 wrist command / q_hand_target[16]
```

这也是当前实现不需要定义 VLA action chunk 语义的原因：没有模型输出一段 action chunk，
也没有跨设备传输 chunk。当前 plan 是低频阶段规格，executor 在本地逐步生成控制量。

---

## 5. 场景与对象信息如何产生

### 5.1 `ProbeSceneSpec`

示例首先调用：

```python
spec = make_demo_scene("mass", candidates, seed)
```

`mass` family 中每个候选对象均为：

```text
shape = short_can
size = (0.030, 0.030, 0.036)
friction_mu = 1.4
```

这里的 `size` 遵循 MuJoCo cylinder geom 约定：

```text
(radius_x, radius_y, half_height)
```

不是完整直径和完整高度。对象质量从预设质量集合中按 seed 抽样并打乱。质量被写进
MuJoCo 物理模型，用来形成真实动力学差异，但 plan builder 不读取
`ObjectSpec.mass_kg`；它只读取 probe 估计。

### 5.2 backend 与 scene

```python
probe_backend = AllegroHandBackend.create(spec)
scene = probe_backend.scene
```

`AllegroProbeScene` 持有：

- `model`：MuJoCo 模型、joint、actuator、geom、sensor 定义；
- `data`：当前 qpos、qvel、ctrl、sensor data 和 contact；
- `dt`：当前默认为 `0.002 s`；
- 对象 pose sensor；
- wrist force/torque sensor；
- Allegro fingertip 与接触分类；
- 6-DoF wrist position actuator；
- 16 个 Allegro joint position actuator。

对象 pose 由对象顶部 site 的 `framepos/framequat` 传感器返回。因此代码中的：

```python
scene.object_pos(i)[2]
```

是对象顶部 site 的世界坐标 z，不是对象质心 z。short can 中：

```text
object_center_z = object_top_z - half_height
object_bottom_z = object_top_z - 2 × half_height
```

四元数顺序沿用 MuJoCo：`wxyz`。

---

## 6. Probe 阶段：`ProbeCommand` 如何变成 `ProbeResult`

### 6.1 高层命令

示例构造：

```python
ProbeCommand(
    primitive="heft",
    target=args.target,
    params={},
)
```

`ProbeHarness.execute()` 不做推理，只把命令交给 `run_probe()`：

```text
ProbeCommand.primitive → primitive dispatcher
ProbeCommand.target    → 被探测对象索引
ProbeCommand.params    → heft() 可选参数
```

`run_probe()` 先检查：

1. primitive 是否属于 `poke/heft/shake/slide`；
2. 当前 `mass` family 是否应该使用 `heft`；
3. target 是否在候选范围内。

只有通过这些检查才调用 `heft()`。

### 6.2 `heft()` 的默认参数

当前默认值：

| 参数 | 数值 | 含义 |
|---|---:|---|
| `lift_height` | Allegro `0.130 m`；reference `0.025 m` | probe 抬升腕部目标量 |
| `hold_time` | `0.45 s` | 离开支撑后的测量窗口 |
| `osc_amp` | `0.001 m` | 测量阶段腕部 z 小幅振荡 |
| `osc_freq` | `1.5 Hz` | 腕部振荡频率 |
| `penetration_limit` | `0.0055 m` | 手—目标接触穿透硬上限 |
| Allegro `min_grasp_force` | `7.0 N` | top pinch 要求的总法向力 |

这里 `min_grasp_force` 的语义同样是当前所有手—物体 contact normal-force magnitude 的
总和，而不是某一个手指的目标力矩。

### 6.3 probe 抓取准备

`_prepare_grasp()` 依次做：

```text
approach
→ guarded_descent
→ contact_establish
→ contact_quality_gate
→ wrist force/torque baseline
```

控制过程：

1. 对象直接位于桌面；根据实测几何中心计算 wrist：
   `x=center_x, y=center_y-0.020, wrist_z=object_top+0.094`。
2. 手先变到 `q_preshape=allegro_grip_pose(0.10)`，腕部在高位完成平移。
3. 高位单独翻腕到 `Rx(pi)`，再依次下降到 pregrasp 和 grasp；每步要求完全无预接触。
4. 16-D 目标按 `q_preshape → q_contact(0.80) → q_squeeze_limit(0.98)` 分段插值。
5. 只有 `mf + th`、目标 `*_top_lip`、且接触 link 属于 fingertip/thumbtip/distal
   白名单时才是 legal grasp；palm/base/proximal、ff/rf 均不能充当抓取接触。
6. 总法向力达到 `7 N` 后进入 100 步稳定窗口，之后在 lift/measurement 中继续用
   contact feedback 微调 16-D closure progress。

reference 后端仍保留左右夹爪和窄 pedestal；它与 Allegro 不共享上述 top-entry pose。

### 6.4 抬升与有效性 gate

Allegro `_lift_and_gate()` 用 smoothstep 把 wrist z target 增加 `0.130 m`，reference
增加 `0.025 m`，随后检查：

- 对象实际抬升距离是否足够；
- 对象是否已经脱离 table（reference 为 pedestal/table）；
- 是否仍有 legal `mf + th` top-lip 接触；
- 接触穿透是否超限；
- post-lift drift 是否过大。

只有 `grasp.established` 和 post-lift stable 同时成立，测量才被认为有效。

### 6.5 腕部力信号如何变成质量相关特征

在 probe 抓取稳定后，先采 50 步 wrist force 和 torque 均值作为 baseline：

```text
baseline_force[3]
baseline_torque[3]
```

抬升并脱离支撑后，在 `hold_time=0.45 s` 内执行：

```python
z_cmd(t) = z0 + 0.001 × sin(2π × 1.5 × t)
force_delta(t) = wrist_force(t) - baseline_force
```

由于当前焊接 child 和 sensor 方向约定会影响符号，代码取重力轴增量中位数的绝对值：

```python
weight_signal_N = abs(median(force_delta[:, 2]))
m_est_kg = weight_signal_N / 9.81
Fz_delta_std_N = std(force_delta[:, 2])
```

测量期间如果重新碰 table/pedestal，或连续丢失 legal grasp 超过 20 步，probe 会变为无效，
`weight_signal_N` 不会作为可信结果继续使用。

需要注意：

- `m_est_kg` 是当前传感和控制条件下的估计，不是读取 hidden mass。
- `weight_signal_N` 是 baseline-subtracted wrist z force 的稳定统计量。
- manipulation 当前只读取 `m_est_kg` 和 `weight_signal_N`；
  `Fz_delta_std_N`、完整 trace 暂不参与 plan。

### 6.6 放回、退场与 `ProbeResult` 封装

`heft()` 不会在高处松手。它先把 tilt/yaw 归零并保持 top-entry roll，闭环下降到原桌面
接触连续稳定，低刚度完全张手，垂直退到高位后才把 roll 转回 0。放回、release 和
retreat 仍逐步执行相同 collision audit；任何新违例都会令 probe 无效。随后生成：

```python
ProbeResult(
    object_id="objN",
    target=N,
    primitive="heft",
    backend="allegro",
    valid=...,
    violations=[...],
    phase_reached=...,
    features={
        "m_est_kg": ...,
        "weight_signal_N": ...,
        "Fz_delta_median_N": ...,
        "Fz_delta_std_N": ...,
        "lifted": ...,
        "hand_contact_group_count": ...,
    },
    quality={...},
    params={...},
    raw_summary={...},
    trace={...},
)
```

此时 probe controller 已经结束。`ProbeResult` 是不可隐式解释的事实记录；它不会自动
触发 manipulation。

---

## 7. Handoff：probe 结果和 manipulation intent 如何合并

### 7.1 target 不是由 `ProbeResult` 自动选择的

示例程序接收：

```bash
--target 0|1|2
```

同一个 target 同时被放入：

```python
ProbeCommand("heft", target)
ShortCanPickPlaceRequest(target)
```

所以当前链路回答的是：

> 已经明确要操作 target N 后，如何用 target N 的 heft 结果条件化 pick/place？

它没有回答：

> 对多个候选分别 probe 后，谁来比较结果并选出最重的对象？

候选比较和目标选择仍在本仓库之外。

### 7.2 manipulation intent

当前 request 为：

```python
ShortCanPickPlaceRequest(
    target=N,
    place_offset_xy_m=(0.0, 0.12),
    reset_before_execute=True,
)
```

字段分工：

| 字段 | 来源 | 作用 |
|---|---|---|
| `target` | 上层/CLI | 指定操作哪个对象 |
| `place_offset_xy_m` | 当前固定 skill | 指定相对初始对象位置的放置位移 |
| `reset_before_execute` | 当前 Sim v1 协议 | 要求从规范初态开始执行 |

`ProbeResult` 不携带 place target，因此不能单独生成完整 manipulation intent。

### 7.3 plan admission 的硬检查

`build_short_can_pick_place_plan()` 在生成任何可执行 plan 前检查：

1. scene backend 必须是 `allegro`；
2. 必须允许 canonical reset；
3. place offset 必须等于 scene 固定值 `(0.0, 0.12)`；
4. family 必须是 `mass`；
5. target 必须在范围内；
6. object shape 必须是 `short_can`；
7. `ProbeResult.target/object_id` 必须与 request 和 scene 对象一致；
8. probe backend 必须是 `allegro`；
9. primitive 必须是 `heft`；
10. `valid=True` 且 `violations` 为空；
11. `m_est_kg` 必须是有限正数；
12. `weight_signal_N` 必须是有限正数。

失败时返回：

```python
ManipulationPlanDecision(
    executable=False,
    reason="明确的拒绝原因",
    context=None,
    plan=None,
)
```

executor 不会被调用。也就是说，无效 probe 不会被悄悄替换成默认质量继续抓取。

### 7.4 canonical reset 发生在哪里

plan builder 只记录：

```text
handoff_policy = reset_to_canonical_checkpoint
```

真正的 `scene.reset()` 在 `execute_short_can_pick_place()` 的 `handoff` phase 中发生。

reset 会：

- 调用 `mj_resetData()`；
- 把所有对象恢复到候选初始位置和单位四元数；
- 把 wrist 恢复到固定 neutral pose；
- 打开 Allegro 手；
- `mj_forward()` 后仿真 150 步稳定场景；
- 重新保存 `_initial_object_pos`。

因此 plan 生成阶段并不偷偷改变场景，executor 开始时才执行物理 handoff。

---

## 8. `ProbeResult` 如何生成 `ManipulationContext`

通过 admission 后，builder 生成 `ManipulationContext`。字段来源如下：

| Context 字段 | 实际来源 | 是否由 probe 决定 |
|---|---|---|
| `schema_version` | 固定为 `allegro_manip.v1` | 否 |
| `scene_id` | `scene.task.scene_id` | 否 |
| `backend` | 已校验的 scene backend | 否 |
| `family` | 固定且已校验为 `mass` | 否 |
| `target` | request | 否 |
| `object_id` | scene object | 否 |
| `shape` | scene visible object spec | 否 |
| `collision_size_m` | scene visible object spec | 否 |
| `probe_primitive` | 已校验为 `heft` | 否 |
| `mass_estimate_kg` | `ProbeResult.features["m_est_kg"]` | 是 |
| `weight_signal_N` | `ProbeResult.features["weight_signal_N"]` | 是 |
| `target_total_normal_force_N` | probe signal 经分段公式 | 是 |
| `max_wrist_speed_mps` | mass estimate 经公式 | 是 |
| `probe_quality` | `ProbeResult.quality` 的副本 | 来自 probe，但当前不再计算控制量 |
| `handoff_policy` | request | 否 |

Context 的作用是保留“这个 plan 为什么这样生成”的可检查依据。当前 executor 直接消费
`ManipulationPlan`，不会在执行中再次读取或修改 `ProbeResult`。

---

## 9. Probe 特征如何生成控制参数

### 9.1 轻物体分支判断

```python
lightweight_release = mass_estimate_kg < 0.27
```

`0.27 kg` 是 v1 compatibility controller 针对当前 short_can 的经验分界，不是通用
物理常数。它已按 full-collision top-pinch heft 的近物理质量信号重新标定。

### 9.2 目标总法向力

轻物体：

```python
target_total_normal_force_N = 6.8 if mass_estimate_kg < 0.18 else 7.6
max_wrist_speed_mps = 0.070
```

普通/重物体：

```python
target_total_normal_force_N = clamp(
    8.0 + 0.45 * weight_signal_N,
    8.0,
    11.0,
)
```

它的严格语义是：

```text
所有合法 hand-object contacts 的 normal-force magnitude 之和
```

它不是：

- 单个指尖的法向力；
- 每个手指都要达到的力；
- actuator torque；
- wrist force sensor 的目标值。

### 9.3 最大腕部平移速度

```python
max_wrist_speed_mps = clamp(
    0.075 / (1.0 + 0.65 * m_est_kg),
    0.040,
    0.070,
)
```

质量估计越大，允许的腕部平移速度越低。该速度只参与 executor 的 trajectory timing，
不是直接写入一个 MuJoCo velocity actuator。

### 9.4 放置控制分支

| 参数 | 轻物体 | 普通/重物体 |
|---|---:|---:|
| `use_gravity_settle` | `True` | `False` |
| `post_descent_xy_correction` | `False` | `True` |
| `max_release_height_m` | `0.025` | `0.010` |
| `hand_table_release_guard_N` | `20` | `30` |
| descent 时 Allegro kp | `2.0` | `8.0` |
| gravity settle kp | `0.35` | 不使用 |

这一分支说明 probe 结果不是只进入日志：它确实改变后续力目标、速度、放置高度、
手—桌面 guard、是否做 XY 纠偏以及是否执行 gravity settle。

---

## 10. `ManipulationPlan` 中哪些是固定模板参数

除 probe-conditioned 参数外，当前 plan 固定：

| Plan 字段 | 数值 | 用途 |
|---|---:|---|
| `skill` | `short_can_pick_place` | executor 类型检查 |
| `place_offset_xy_m` | `(0.0, 0.12)` | 固定 +Y 放置目标 |
| `lift_height_m` | `0.035` | wrist 抬升命令量 |
| `min_carry_distance_m` | `0.080` | 对象实际搬运验收下限 |
| `hold_time_s` | `0.50` | 最终稳定性观察时间 |
| `max_total_normal_force_N` | `20` | grasp/carry 总手力上限 |
| `max_place_normal_force_N` | `30` | place/release 总手力上限 |
| `max_hand_table_force_N` | `40` | 全局手—桌面力硬上限 |
| `max_penetration_m` | `0.0052` | grasp/carry 穿透上限 |
| `max_place_penetration_m` | `0.0055` | place/release 穿透上限 |
| `max_place_error_m` | `0.035` | 最终 XY 放置误差上限 |
| `max_final_tilt_rad` | `0.20` | 最终倾角上限 |
| `max_final_drift_m` | `0.005` | 0.5 s 内漂移上限 |

`phases` 只是允许的阶段顺序，不是每个仿真步的命令数组：

```text
handoff
→ preshape
→ approach
→ contact_acquire
→ grip_regulate
→ lift
→ carry
→ place_descent
→ settle_to_surface
→ release
→ retreat
→ final_verify
```

---

## 11. 16-DoF Allegro 手指目标如何产生

### 11.1 actuator 顺序

所有手指目标必须严格按 `scene.py` 中的 actuator 顺序排列：

```text
ffa0 ffa1 ffa2 ffa3
mfa0 mfa1 mfa2 mfa3
rfa0 rfa1 rfa2 rfa3
tha0 tha1 tha2 tha3
```

即 index finger、middle finger、ring finger、thumb，每指 4 个关节，共 16 维。

### 11.2 当前 short can 模板

`short_can_hand_template()` 从已有 cylinder synergy 采样四个显式姿态：

```python
q_open          = allegro_grip_pose(0.00)
q_preshape      = allegro_grip_pose(0.10)
q_contact       = allegro_grip_pose(0.78)
q_squeeze_limit = allegro_grip_pose(0.94)
```

synergy 本身是：

```python
q(alpha) = (1 - alpha) * q_open_base + alpha * q_cylinder_closed
```

manipulation 的闭合进度则在 `q_preshape` 和 `q_squeeze_limit` 之间插值：

```python
q_template(progress) =
    (1 - progress) * q_preshape
    + progress * q_squeeze_limit
```

因此当前已经做到：

- executor 每次下发完整 `q_target[16]`；
- 16 个目标会分别经过各 actuator ctrl range 裁剪并写入 `data.ctrl`；
- 反馈可以微调 `closure_progress`，从而重新生成下一组 16-D 目标。

但还没有做到：

- 根据对象 pose 在线求 finger IK；
- 独立优化每个手指；
- 根据 tactile 分布重规划接触点。

### 11.3 合法抓取模板约束

当前模板指定：

```text
active_fingers = (mf, th)
required_contact_groups = (mf, th)
required_object_geom = objN_geom  # can waist geom
wrist_y_offset = 0.020 m
wrist_to_object_center_z = 0.414 m
```

接触建立阶段要求 middle finger 和 thumb 都参与，并且至少一个手接触落在 can waist geom。
这防止仅靠顶部 lip 偶然挂住就被判为合法 acquisition。

---

## 12. 腕部控制信号如何产生

### 12.1 六个 wrist position target

`scene.command()` 可直接写：

```text
x, y, z, roll, tilt, yaw
```

它们分别进入：

```text
act_wx, act_wy, act_wz, act_wr, act_wt, act_wyaw
```

这些都是 MuJoCo position actuator target，不是速度或 torque command。模型当前使用：

| actuator | kp |
|---|---:|
| wrist x/y | `650` |
| wrist z | `900` |
| wrist roll/tilt | `120` |
| wrist yaw | `80` |

这正是“理想腕部 carriage”的含义：代码可以直接给出 wrist world-like pose target，
中间没有真实机械臂 joint trajectory、IK 或 dynamics controller。

### 12.2 waypoint 如何展开成逐步命令

`_move_wrist()` 读取当前 actuator target 作为 start，然后计算平移距离：

```python
distance = norm(goal_xyz - start_xyz)
duration = distance / max_wrist_speed_mps
steps = max(ceil(duration / dt), min_steps)
```

每个仿真步使用 smoothstep：

```python
s = clip((k + 1) / steps, 0, 1)
alpha = s² * (3 - 2s)
u_k = u_start + alpha * (u_goal - u_start)
```

然后执行：

```text
scene.command(**u_k)
→ scene.step(1)
→ ContactSnapshot
→ safety/grasp feedback
```

因此 plan 只保存速度上限和阶段目标。具体有多少控制步由 waypoint 距离、`dt`、
`max_wrist_speed_mps` 和 phase 的 `min_steps` 共同决定。

### 12.3 hand target 如何展开

`_move_hand()` 同样从当前 `q_target[16]` 出发，用 smoothstep 插值到 goal：

```python
q_k = (1 - alpha_k) * q_start + alpha_k * q_goal
```

每次调用：

```text
command_allegro_joints(q_k)
→ data.ctrl[16 Allegro actuators]
→ mj_step
→ observe contact
```

---

## 13. 接触反馈如何产生

每次 `mj_step` 后，`contact_snapshot(target)` 遍历 `data.contact`，并调用：

```python
mujoco.mj_contactForce(model, data, contact_index, wrench)
```

对 contact 的两个 geom/body 做身份分类后，形成：

| 反馈字段 | 生成方式 | executor 用途 |
|---|---|---|
| `hand_groups` | 发生手—物体接触的 `ff/mf/rf/th` 集合 | 对向抓取和释放判断 |
| `hand_force_by_group_N` | 按手指累计法向力 | 诊断；当前控制主要用总和 |
| `hand_object_geoms` | 被手接触的对象 geom 名称 | acquisition 要求 waist contact |
| `hand_normal_force_N` | 所有手—物体 contact 法向力幅值之和 | 力目标和硬上限 |
| `support_contact` | 对象—自身 pedestal 接触 | lift/carry 失败 gate |
| `table_contact` | 对象—table/floor 接触 | carry gate、落桌和最终支撑 |
| `hand_table_contact` | 手—table/floor 接触 | descent guard 和最终退出 |
| `hand_support_contact` | 手—任意 pedestal 接触 | 环境碰撞 gate |
| `hand_max_penetration_m` | 手—物体 contact 最大 `-dist` | phase penetration gate |
| `table/support_max_penetration_m` | 对象与环境的最大穿透 | 诊断 |

法向力用 `max(wrench[0], 0)`，穿透量用 `max(-contact.dist, 0)`。

executor 的 `_Execution.observe()` 还会维护全过程聚合量：

```text
max_grasp_carry_penetration_m
max_place_release_penetration_m
peak_grasp_carry_force_N
peak_place_release_force_N
peak_hand_table_force_N
```

并每 10 次 observe 保存一次抽样 trace。trace 是调试输出，不反馈给 plan builder。

---

## 14. Manipulation 各阶段的信号生成与反馈闭环

### 14.1 `handoff`

输入：

```text
plan.reset_before_execute = True
```

动作：

```text
scene.reset()
scene.step(50)
```

随后读取 canonical object top-site pose：

```python
initial_object_pos = object_pos(target)
place_xy = initial_object_pos[:2] + [0.0, 0.12]
```

这里生成的是目标位置，不是预录轨迹。

### 14.2 `preshape`

控制信号：

```text
当前 q_target[16]
→ 120 步 smoothstep
→ q_preshape[16]
```

执行过程中会采集接触信息，但这一阶段当前没有独立的“到位误差”验收；MuJoCo position
actuator 负责跟踪目标。

### 14.3 `approach`

首先基于当前对象 pose 计算抓取 wrist target：

```python
object_center_z = object_top_z - half_height
grasp_x = object_x
grasp_y = object_y + 0.020
grasp_z = object_center_z - 0.414
```

两段 approach：

```text
1. 到 (grasp_x, grasp_y, grasp_z + 0.075)，姿态固定为零
2. 仅沿 z 下降到 grasp_z，至少 160 步
```

每个控制步检查手—物体 penetration；超限立即退出。

### 14.4 `contact_acquire`

闭合进度：

```python
progress ∈ linspace(0.0, 1.0, 181)
q_cmd[16] = template.pose(progress)
```

每个 progress 保持并执行 5 个 MuJoCo step，再读取 contact。

成功条件：

```text
mf ∈ hand_groups
∧ th ∈ hand_groups
∧ objN_geom ∈ hand_object_geoms
∧ hand_normal_force_N >= target_total_normal_force_N
```

同时监视：

- hand penetration；
- 手—pedestal 碰撞；
- 手—table 力硬上限。

`first_legal_contact_progress` 和最终 `closure_progress` 被写入 quality。这里的
`q_contact` 是模板资产中的参考姿态，但 acquisition 实际停止点由 contact feedback 和
目标力决定，并不是无条件播放到固定终点。

### 14.5 `grip_regulate`

建立抓取后保持 100 个 step，要求至少 80 个 step 满足：

```text
合法 mf+th+waist 接触
∧ hand force >= 0.80 × target force
```

不足则返回 `unstable_pregrasp`。这一步防止刚碰到阈值就立即抬升。

### 14.6 `lift`

腕部 z target：

```python
z_goal = current_wrist_z_ctrl + 0.035
```

至少 180 步 smoothstep 执行。结束后检查：

```text
对象不再接触 pedestal/table
∧ 手不接触 pedestal/table
∧ mf+th 对向接触仍存在
∧ actual object lift >= 0.020 m
```

`0.035 m` 是 wrist 命令量，`0.020 m` 是对象实际运动 gate；两者不能混用。

### 14.7 `carry`

第一段把 wrist x/y 移到 `place_xy`。移动时 `_regulate_grasp()` 每步工作：

```python
if hand_force < 0.70 * target_force:
    closure_progress += 0.0015

elif hand_force > 1.35 * target_force:
    closure_progress -= 0.0010

q_cmd = template.pose(closure_progress)
```

这就是 carry 期间的手指闭环：反馈不是直接转成 torque，而是微调模板闭合进度，再生成
新的 16-D position target。

同时检查：

```text
hand force <= 20 N
hand penetration <= 5.2 mm
lost opposing contact <= 80 steps
对象不接触 pedestal/table
手不接触 pedestal/table
```

由于抓取有柔顺性，wrist 到位不保证 object 到位。代码最多进行两轮 object-space XY
误差修正：

```python
correction_xy = place_xy - object_xy
wrist_goal_xy = current_wrist_ctrl_xy + correction_xy
```

误差小于 `4 mm` 时不再修正。最后以对象实际 XY 位移检查 carry distance 是否至少
`80 mm`。

这属于固定目标的 pose feedback，不是路径规划。

### 14.8 `place_descent`

进入时设置 Allegro position servo kp：

```text
轻物体: 2.0
普通/重物体: 8.0
```

wrist z 从当前值最多向下扫描 `0.26 m`，共 1301 个候选 descent target；每个 target
执行 3 步，让 position carriage 跟踪后再观察 contact。

下降停止条件取三者之一：

```text
对象已经 table_contact
OR object_bottom_z 到 table 的 clearance <= max_release_height_m
OR hand_table_force >= hand_table_release_guard_N
```

同时存在硬 gate：

```text
hand penetration <= 5.2 mm
hand-table force <= 40 N
place/release hand force <= 30 N
```

触发正常停止后，不是继续压 wrist，而是把 z target 设为实际 wrist z 再上移 `2 mm`，
消除 position actuator 已积累的向下误差。

普通/重物体随后最多做两轮 XY 修正，误差小于 `2 mm` 时停止；轻物体跳过这一步，避免
近桌面横向修正向低惯量物体注入过多能量。

### 14.9 `settle_to_surface`：仅轻物体

触发条件：

```text
plan.use_gravity_settle == True
```

控制策略：

```text
固定 wrist
Allegro kp = 0.35
当前 q_target[16] → q_open[16]，最多 800 步
```

目的不是直接松手，而是让 short can 在低刚度手指笼约束中靠重力下滑到桌面。

反馈状态：

- `table_steps`：对象连续落桌步数；达到 12 步即成功；
- `unsupported_clear_steps`：对象既无手接触又无桌面接触的连续步数；超过 100 步判定
  `dropped_before_surface`；
- 继续检查 penetration 和 `40 N` 手—桌面力上限。

落桌后 wrist 上移 `2 mm` 并执行 20 步，再进入 release。

### 14.10 `release`

release 不反向播放抓取轨迹，而是：

```text
Allegro kp = 0.1
所有 16 个关节从当前 target 对称插值到 q_open
最多 600 步
```

释放成立条件：

```text
完全没有 hand-object contact
```

或：

```text
对象已经 table_contact
∧ mf+th 对向抓取已经消失
∧ residual hand force <= 1.5 N
```

条件连续成立 12 步后才认为 clear。过程中继续检查：

```text
place penetration <= 5.5 mm
place/release hand force <= 30 N
```

函数无论成功还是失败都会在 `finally` 中把 Allegro kp 恢复为 `8.0`。

release 前、刚 clear 时和再等待 80 步后分别读取对象位置，用于计算释放扰动：

```text
release_to_clear_displacement_m
release_displacement_m
```

### 14.11 `retreat`

wrist z target 上移 `0.10 m`，且最终不超过当前 carriage 约束中的 `0.10`；至少执行
180 步。随后 80 步把手移动到 `q_open[16]`。

retreat 的目标是让最终状态中：

```text
hand-object contact = 0
hand-table contact = 0
```

### 14.12 `final_verify`

保持 `0.50 s`：

```python
hold_steps = int(0.50 / dt)
```

默认 `dt=0.002 s` 时为 250 步。每步记录 object position 并观察 contact，最后生成：

```text
place_error_m
final_tilt_rad
final_drift_m
final_table_contact
final_hand_contact_group_count
final_hand_table_contact
```

倾角由对象 `wxyz` 四元数中 local z 轴相对 world z 的夹角计算。漂移取观察窗口中位置
相对第一帧的最大欧氏距离。

最终成功是所有条件的合取：

```text
place error <= 35 mm
∧ tilt <= 0.20 rad
∧ drift <= 5 mm / 0.5 s
∧ object-table contact exists
∧ no hand-object contact
∧ no hand-table contact
∧ grasp/carry penetration <= 5.2 mm
∧ place/release penetration <= 5.5 mm
∧ grasp/carry hand force <= 20 N
∧ place/release hand force <= 30 N
∧ hand-table force <= 40 N
```

---

## 15. 一次控制循环中究竟发生什么

以 carry 中一个 wrist step 为例：

```text
1. 根据 smoothstep 计算本步 wrist x/y/z target
2. scene.command() 把 target 写入 wrist actuator ctrl
3. 如果上一反馈要求调抓力：
     closure_progress 改变
     → template.pose(progress)
     → q_hand_target[16]
     → 写入 Allegro actuator ctrl
4. mujoco.mj_step()
5. MuJoCo 根据 position actuator、刚体动力学和 contact 求解新状态
6. contact_snapshot() 读取并分类当前 contacts
7. _Execution.observe() 更新峰值、穿透和抽样 trace
8. _regulate_grasp() 判断：
     - 是否需要稍微闭合/打开
     - 是否丢失对向抓取
     - 是否碰 pedestal/table
     - 是否超力/超穿透
9. 若无 failure，生成下一步 target；否则立即返回失败结果
```

所以当前不是两种极端情况中的任何一种：

- 不是完全播放一条与物体无关的固定高频轨迹；
- 也不是在线运动规划器每步重新规划完整抓取路径。

更准确的描述是：

> 固定 phase 和对象模板提供 nominal motion；对象 pose、质量 probe 信号和实时 contact
> feedback 决定参数、停止时刻、局部修正和是否失败。

---

## 16. 失败信号如何传播

### 16.1 plan 生成前失败

```text
admission gate failed
→ ManipulationPlanDecision.executable = False
→ reason = one explicit reason
→ example exits before executor
```

典型 reason：

```text
allegro_backend_required
canonical_reset_required
fixed_place_offset_required
short_can_required
probe_target_mismatch
heft_probe_required
probe_invalid
mass_estimate_missing
weight_signal_missing
```

### 16.2 executor 中失败

phase helper 调用：

```python
run.fail("violation_name")
```

同一 violation 不重复加入列表。执行路径随后返回 `_result(success=False)`，其中：

```text
status = violations[0]
success = False
phase_reached = 当前 phase
violations = 全部已记录 violation
```

典型 execution violation：

```text
legal_grasp_not_acquired
unstable_pregrasp
not_lifted
lost_legal_grasp
pedestal_contact_after_lift
hand_support_collision
hand_table_collision
hand_force_limit
penetration_limit
release_height_not_reached
dropped_before_surface
release_incomplete
place_error
object_tilted
object_not_settled
object_not_supported_after_place
hand_not_retreated
```

`_result()` 在任何退出路径再次把 Allegro kp 恢复到 `8.0`，避免一次失败污染下一次执行。

---

## 17. `ManipulationExecutionResult` 如何生成

成功结果结构：

```python
ManipulationExecutionResult(
    object_id="objN",
    target=N,
    skill="short_can_pick_place",
    status="ok",
    success=True,
    backend="allegro",
    phase_reached="final_verify",
    violations=[],
    quality={...},
    params={...},
    trace={...},
)
```

### 17.1 `quality`

包含实际执行测量，例如：

```text
first_legal_contact_progress
closure_progress
lift_distance_m
carry_distance_m
release_to_clear_displacement_m
release_displacement_m
place_error_m
final_tilt_rad
final_drift_m
final_table_contact
final_hand_contact_group_count
final_hand_table_contact
max_grasp_carry_penetration_m
max_place_release_penetration_m
peak_grasp_carry_force_N
peak_place_release_force_N
peak_hand_table_force_N
```

### 17.2 `params`

记录本次执行实际使用的策略和 plan 参数，例如：

```text
schema_version
template
target_total_normal_force_N
max_place_normal_force_N
hand_table_release_guard_N
max_hand_table_force_N
max_wrist_speed_mps
release_policy
post_descent_xy_correction
use_gravity_settle
handoff_policy
```

### 17.3 `trace`

在 `include_trace=True` 时输出抽样时序：

```text
phase
sample_phase
planned_place_xy_m
object_pos_m
wrist_pos_m
hand_groups
hand_object_geoms
hand_force_N
penetration_m
source_support_contact
hand_table_contact
hand_table_force_N
hand_support_contact
```

`ManipulationExecutionResult` 不会覆盖或修改 `ProbeResult`。上层如果需要完整 episode
记录，应同时保存二者以及中间的 `ManipulationPlanDecision`。

---

## 18. 一次具体运行中的数据变化示例

以下用已经回归过的 `seed=0, target=2` 说明信号方向；数值只代表当前场景：

```text
heft weight_signal_N ≈ 5.47 N
        ↓
m_est_kg = weight_signal_N / 9.81
        ↓
mass_estimate_kg >= 0.27
        ↓
选择普通/重物体放置分支
        ↓
target_total_normal_force_N ≈ 10.46 N
post_descent_xy_correction = True
use_gravity_settle = False
hand_table_release_guard_N = 30 N
        ↓
实际 lift ≈ 23.9 mm
实际 carry ≈ 121.1 mm
最终 place error ≈ 23.3 mm
最终 object-table contact = True
最终 hand-object contact = 0
        ↓
ManipulationExecutionResult.success = True
```

这里最重要的因果关系是：

```text
heft signal
→ plan 参数与 place controller 分支
→ executor 每步命令与反馈 gate
→ 独立 manipulation result
```

而不是：

```text
heft trace
→ 直接重放成 pick/place trajectory
```

---

## 19. 如何运行与查看四段输出

```bash
conda activate probebench
python -m examples.run_short_can_pick_place \
  --seed 0 \
  --target 2 \
  --viewer
```

需要时序 trace：

```bash
python -m examples.run_short_can_pick_place \
  --seed 0 \
  --target 2 \
  --include-trace
```

程序按顺序输出：

```text
SCENE
PROBE_RESULT
PLAN_DECISION
MANIPULATION_RESULT
```

阅读顺序建议：

1. `PROBE_RESULT.valid/violations`：probe 是否可被信任；
2. `PROBE_RESULT.features`：实际进入下游的质量信号；
3. `PLAN_DECISION.executable/reason`：是否允许执行；
4. `PLAN_DECISION.context`：plan 的事实依据；
5. `PLAN_DECISION.plan`：最终阈值、分支和模板；
6. `MANIPULATION_RESULT.phase_reached/violations`：在哪里结束；
7. `MANIPULATION_RESULT.quality`：实际物理结果；
8. trace：只有定位时序问题时再展开。

回归测试：

```bash
conda run -n probebench python -m pytest tests/test_manipulation.py -q
```

当前测试覆盖 3 个 seed × 3 个 target 的 9 条真实 MuJoCo 闭环，以及 admission 拒绝、
16-D command shape 和 gain 恢复。

---

## 20. 当前实现中最关键的事实与限制

### 20.1 当前不是完全固定轨迹

固定的是：

- skill phase 顺序；
- short can 抓型模板；
- nominal offset 和阈值；
- wrist 姿态始终为固定零姿态。

在线变化的是：

- canonical reset 后读取的对象 pose；
- probe-conditioned 目标抓力和 wrist 速度；
- light/heavy place 分支；
- 接触 acquisition 的停止进度；
- carry 中 closure progress；
- object-space XY 修正；
- descent 停止时机；
- release 停止时机；
- 所有安全 gate。

### 20.2 当前仍不是规划系统

没有：

- arm joint-space trajectory；
- obstacle-aware path search；
- grasp candidate search；
- collision-free IK；
- 在线接触点优化。

因此代码中的“plan”是 phase-and-threshold plan，不应被表述为机械臂运动规划结果。

### 20.3 当前 probe 结果的使用很窄

实际用于控制生成的只有：

```text
m_est_kg
weight_signal_N
```

`probe_quality` 当前被保存在 Context 中，但 admission 只使用总体 `valid/violations`，并未
根据 quality 连续调节控制参数。未来如果扩展 quality-conditioned safety margin，需要新增
明确公式和测试，不能假定当前已经存在。

### 20.4 当前定位是 oracle

executor 每次通过 `scene.object_pos/object_quat` 读取真实仿真 pose。定位误差、感知延迟和
坐标标定误差尚未进入闭环，因此当前 place error 只代表执行器和接触误差。

### 20.5 当前 handoff 是 reset

这使回归可重复，也让 probe 信息和 manipulation 控制的因果关系更容易审计；但它尚未
证明同一连续 episode 中 probe 后立即抓取和搬运的能力。

### 20.6 当前参数不能直接迁移实机

以下数值都是当前解析几何和 MuJoCo 接触参数下的标定结果：

- `0.27 kg` light/heavy 分界；
- 两段 target force 公式；
- `5.2/5.5 mm` penetration gate；
- 手—桌面 `20/30/40 N` guard；
- `kp=2.0/0.35/0.1` 放置与释放调度。

在实机或新对象上必须重新定义传感单位、force 语义、actuator 映射和安全阈值。

---

## 21. 后续扩展时应保持的接口边界

即使未来接入真实机械臂、视觉定位或更多对象，建议保持以下分层：

```text
ProbeResult
  只保存 probe 事实、估计、质量与 violations

Manipulation intent/request
  由上层明确 target、skill 和 goal

Plan admission/context
  校验 probe 是否能用于本 skill，并保留参数来源

ManipulationPlan
  保存 skill 级阶段、模板、约束和 feedback requirement

Execution backend
  把阶段目标变成设备相关的 wrist/arm/hand 控制信号

ManipulationExecutionResult
  独立报告执行成败和物理质量
```

如果以后用真实机械臂替换理想 carriage，应该替换的是：

```text
当前 _move_wrist() + scene.command(x/y/z/rpy)
```

而不是改变 `ProbeResult` 的语义。arm planner 应把 wrist waypoint 转成 arm joint trajectory，
并增加可达性、自碰撞、环境碰撞和跟踪误差反馈。

如果以后用视觉定位替换 oracle pose，应该替换的是对象 pose provider，并在 Context 和
executor 中显式携带 frame、timestamp、confidence 和定位误差界，而不是让 executor
继续假设 pose 无误差。

如果以后新增对象模板，应为每个模板分别定义：

- 16-D `q_open/q_preshape/q_contact/q_squeeze_limit`；
- required contact groups 和 object geom；
- wrist/object 几何关系；
- force、penetration、release 和 placement 验收；
- 对应的闭环测试。

---

## 22. 文档概念到代码符号的索引

| 本文概念 | 当前代码符号 | 文件 |
|---|---|---|
| probe 高层入口 | `ProbeHarness.execute()` | `allegro_probe/interfaces.py` |
| primitive 校验与分发 | `run_probe()` | `allegro_probe/primitives.py` |
| heft 信号生成 | `heft()` | `allegro_probe/primitives.py` |
| manipulation admission 与 plan | `build_short_can_pick_place_plan()` | `allegro_probe/manipulation.py` |
| 16-D short can 模板 | `short_can_hand_template()` | `allegro_probe/manipulation.py` |
| manipulation 总执行入口 | `execute_short_can_pick_place()` | `allegro_probe/manipulation.py` |
| wrist 插值和逐步 guard | `_move_wrist()` | `allegro_probe/manipulation.py` |
| carry 抓力调节 | `_regulate_grasp()` | `allegro_probe/manipulation.py` |
| 轻物体 gravity settle | `_settle_object_to_table()` | `allegro_probe/manipulation.py` |
| 低刚度释放 | `_release_until_clear()` | `allegro_probe/manipulation.py` |
| result 汇总和 kp 恢复 | `_result()` | `allegro_probe/manipulation.py` |
| wrist/hand actuator 写入 | `command()` / `command_allegro_joints()` | `allegro_probe/scene.py` |
| Allegro kp 调度 | `set_allegro_position_kp()` | `allegro_probe/scene.py` |
| 对象/wrist sensor 读取 | `object_pos()` / `object_quat()` / `wrist_force_vec()` | `allegro_probe/scene.py` |
| contact 分类与力汇总 | `contact_snapshot()` | `allegro_probe/scene.py` |
| 可运行示例 | `main()` | `examples/run_short_can_pick_place.py` |
| 闭环验收 | `test_short_can_pick_place_closed_loop()` | `tests/test_manipulation.py` |

---

## 23. 最终总结

当前数据流可以压缩为以下四句话：

1. `heft()` 通过真实抓取、脱离支撑后的 wrist force 增量生成可信 `ProbeResult`。
2. plan builder 把 `ProbeResult` 与明确的 pick/place request 合并，生成质量条件化的
   `ManipulationContext` 和固定 short-can `ManipulationPlan`。
3. executor 在 canonical reset 后，以固定 phase/模板为 nominal，以对象 pose 和 contact
   feedback 为闭环，逐步生成 wrist position target 和 16-D Allegro joint target。
4. 执行结束后独立生成 `ManipulationExecutionResult`，只有落桌、稳定、手退出且全过程
   安全 gate 均满足时才返回成功。

这就是目前仓库中真正打通的 probe → manipulation 数据流和控制信号链路。
