# ProbeResult 到 Manipulation 结束的数据流与控制信号生成

## 目标

本文细化 `sgst_mani_0.md` 和 `sgst_mani_1_fixed_pose.md` 中的接口路线，专门回答：

1. `ProbeResult` 产生后，哪些字段进入 manipulation？
2. `ManipulationContext` 如何从 probe、场景和传感摘要中生成？
3. `ManipulationPlan` 的 phase、pose、force hint、gate 如何诞生？
4. 固定位置执行器如何把 plan 变成每一步的 wrist / gripper 控制信号？
5. manipulation 执行结束后，如何生成独立的执行结果？

本文仍然只设计数据流和控制信号诞生过程，不要求当前仓库马上实现完整 manipulation。

## 总链路

```text
ProbeHarness.execute(ProbeCommand)
        ↓
ProbeResult
        ↓
ProbeResultValidator
        ↓
ManipulationContextBuilder
        ↓
ManipulationContext
        ↓
SkillSelector / FixedPosePlanGenerator
        ↓
ManipulationPlan
        ↓
PhaseCompiler
        ↓
CompiledPhaseCommand[]
        ↓
FixedPoseExecutor
        ↓
per-step low-level command
        ↓
scene.command(x, y, z, roll, tilt, yaw, grip)
        ↓
sensor/contact feedback
        ↓
ManipulationExecutionResult
```

核心原则：

- `ProbeResult` 只说明 probe 是否可信以及探测出了什么，不直接等于 manipulation
  命令。
- `ManipulationContext` 只做“事实和约束整理”，不做具体轨迹。
- `ManipulationPlan` 是阶段化任务规格，不是高频控制序列。
- `CompiledPhaseCommand` 才开始接近控制信号，包含固定位置、速度限制、gripper
  策略和 stop condition。
- 最低层控制信号在 `FixedPoseExecutor` 每个仿真步生成。

## 第 0 层：ProbeResult 的输入契约

`ProbeResult` 是所有后续决策的唯一 probe 输入。第一版只允许读取公开字段：

| 字段 | 用途 | 是否可直接变成控制信号 |
|---|---|---|
| `valid` | 判断 probe feature 是否可信 | 否，只作为 gate |
| `status/controller_status` | 判断控制层是否正常完成 | 否，只作为诊断 |
| `phase_reached` | 判断 probe 失败阶段 | 否，只作为回退/重试依据 |
| `violations` | 判断是否存在超力、失联、支撑接触等问题 | 否，只作为 gate |
| `features` | 属性估计，如 `m_est_kg`、`mu_est`、`k_est`、`fill_proxy` | 可以间接生成 force/speed/limit |
| `quality` | lift distance、contact fraction、path completion 等 | 可以影响 confidence 和是否允许执行 |
| `raw_summary` | baseline、简要诊断 | 只作辅助 |
| `trace` | 时序调试 | 第一版不作为在线 plan 输入 |

第一道硬 gate：

```python
if not result.valid:
    return no_plan("probe_invalid", result.violations)

if result.violations:
    return no_plan("probe_has_violations", result.violations)
```

但这个规则可以按 skill 放宽。例如 material 的 slide 如果 `valid=false`，不能生成可信
`slide_contact` plan；但也许可以生成一个“重新 probe”或“保守 pick 禁止执行”的
diagnostic plan。第一版建议简单处理：无效 probe 不生成可执行 manipulation。

## 第 1 层：ProbeResultValidator

Validator 负责把 probe 结果转成明确的准入状态：

```python
@dataclass
class ProbeAdmission:
    admitted: bool
    reason: str
    family: str
    primitive: str
    target: int
    trusted_features: dict[str, float]
    trusted_quality: dict[str, float]
    violations: list[str]
```

### family-specific admission

| family | 必须 feature | 必须 quality/gate | 拒绝条件 |
|---|---|---|---|
| `mass` | `m_est_kg`, `weight_signal_N` | `lift_distance_m`、`support_contact_after_lift == 0`、`postlift_group_count >= 2` | 未脱离支撑、丢失对向接触、估计质量为 0 |
| `fill` | `fill_proxy`, `slosh_proxy`, `weight_proxy_N` | 通过 heft gate、shake 后无支撑接触 | `heft_invalid`、shake 中重新接触支撑 |
| `stiffness` | `k_est_N_per_m`, `compliance_m_per_N` 或等价字段 | 接触完成率、峰值力、压入量在安全范围内 | 未接触、超力、路径未完成 |
| `material` | `mu_est`, `friction_ratio` | `path_completion_ratio`、`contact_fraction` | slide 未完成、preload 失效、长时间 lost contact |

这一层不计算控制量，只回答：“这些 probe 结果能不能被后续使用？”

## 第 2 层：ManipulationContextBuilder

ContextBuilder 把 probe admission、公开场景信息和当前对象 pose 合并。

输入：

```text
ProbeAdmission
ProbeSceneSpec.visible_dict()
ObjectSpec.visible_dict()
scene.object_pos(target)
scene.object_quat(target)
scene.contact_snapshot(target)
backend name
```

输出：

```python
@dataclass
class ManipulationContext:
    schema_version: str
    scene_id: str
    backend: str
    family: str
    primitive: str
    target: int
    object_id: str
    shape: str
    size_m: tuple[float, float, float]
    object_pose_world: Pose
    object_frame: str
    estimates: dict[str, float]
    quality: dict[str, float]
    confidence: dict[str, float]
    safety_limits: dict[str, float]
    required_feedback: list[str]
    missing_requirements: list[str]
    fallback_policy: str
```

### 字段如何诞生

| Context 字段 | 来源 | 生成逻辑 |
|---|---|---|
| `backend` | backend/scene config | `reference` 或 `allegro`，决定后续 force hint 到 actuator 的映射 |
| `shape/size_m` | `ObjectSpec.visible_dict()` | 用于选择固定模板和接触高度 |
| `object_pose_world` | `scene.object_pos/quat` | 用于把 object-frame 模板转成 world-frame goal |
| `estimates` | `ProbeResult.features` | 只复制 trusted feature，并规范单位 |
| `quality` | `ProbeResult.quality` | 透传，用于 confidence 和 gate |
| `confidence` | feature + quality | 通过规则估计，不是模型输出 |
| `safety_limits` | family rule + estimates | 生成速度、倾角、力、压入量等上限 |
| `required_feedback` | skill/family rule | 指定执行中必须读哪些传感 |
| `missing_requirements` | 需求表 - 已有 estimate | 明确缺少质量、摩擦或几何 affordance 等信息 |

### confidence 的第一版规则

第一版不做复杂概率，只做可解释分数：

```python
confidence["probe"] = 1.0 if result.valid and not result.violations else 0.0
confidence["contact"] = clamp(contact_quality, 0.0, 1.0)
confidence["estimate"] = family_specific_quality_score(result)
confidence["overall"] = min(confidence.values())
```

示例：

- `mass`：`overall` 受 `postlift_stable_fraction`、`lift_distance_m`、
  `postlift_group_count` 影响。
- `material`：`overall` 受 `path_completion_ratio`、`contact_fraction`、
  `max_lost_contact_steps` 影响。
- `stiffness`：`overall` 受 `target_force_ratio`、有效压入量和接触持续时间影响。
- `fill`：`overall` 受 heft gate、shake 中支撑接触、torque signal 稳定性影响。

## 第 3 层：从 Context 生成物理控制 hint

这一层开始生成 manipulation 相关参数，但仍然不是 actuator command。

### 3.1 质量到抓取力

输入：

```text
m_est_kg or weight_proxy_N
mu_est or conservative_mu_default
shape/backend safety factor
quality/confidence
```

生成：

```python
weight_N = m_est_kg * 9.81
mu_eff = clamp(mu_est_or_default, 0.25, 1.5)
quality_scale = 1.0 + (1.0 - confidence["overall"]) * 0.5
raw_force_N = weight_N * safety_factor * quality_scale / mu_eff
target_normal_force_N = clamp(raw_force_N + preload_N, min_force_N, max_force_N)
```

第一版默认值建议：

| 参数 | reference | allegro | 说明 |
|---|---:|---:|---|
| `mu_default` | 0.8 | 0.7 | 未做 material probe 时保守取值 |
| `safety_factor` | 2.0 | 2.5 | Allegro 接触不确定性更大 |
| `preload_N` | 1.0 | 2.0 | 补偿建模误差 |
| `min_force_N` | 2.0 | 4.0 | 低质量物也需要最低闭合 |
| `max_force_N` | 20.0 | 30.0 | 防止过度穿透/损伤 |

注意：这些是物理 hint，不是 `grip_alpha`。

### 3.2 抓取力到 grip policy

由于当前 hand command 更接近 `grip_alpha`，需要第二层映射：

```text
target_normal_force_N
→ backend-specific grip policy
→ per-step grip_alpha command
```

建议的 `GripPolicy`：

```python
@dataclass
class GripPolicy:
    mode: str  # open, close_until_contact, force_hint_hold, bounded_squeeze
    target_normal_force_N: float
    alpha_min: float
    alpha_max: float
    alpha_rate: float
    stop_condition: dict
```

执行时不是一次性设置 `grip=alpha_max`，而是逐步闭合：

```python
while alpha < alpha_max:
    alpha += alpha_rate * dt
    scene.command(grip=alpha)
    snapshot = scene.contact_snapshot(target)
    if has_opposing_grasp(snapshot) and snapshot.hand_normal_force_N >= target_force:
        break
    if snapshot.max_penetration_m > penetration_limit:
        fail("penetration_limit")
```

这就是“力道大小”真正变成低层控制信号的过程：

```text
m_est_kg / mu_eff
→ target_normal_force_N
→ GripPolicy(alpha_min/max/rate)
→ loop 中的 scene.command(grip=alpha)
→ contact_snapshot 反馈决定停止/继续/失败
```

### 3.3 fill/slosh 到运动包络

输入：

```text
fill_proxy
slosh_proxy
weight_proxy_N
```

生成：

```python
slosh_norm = normalize(slosh_proxy, family_calibration)
slosh_scale = 1.0 + k_slosh * slosh_norm
max_tcp_speed_mps = base_speed_mps / slosh_scale
max_accel_mps2 = base_accel_mps2 / slosh_scale
max_tilt_rad = base_tilt_rad / slosh_scale
max_yaw_rate_rps = base_yaw_rate_rps / slosh_scale
settle_time_s = base_settle_s * slosh_scale
```

这些参数进入 `ManipulationPlan.control_limits`，随后由 `PhaseCompiler` 转成每个 phase
的步数、插值速度和姿态变化上限。

### 3.4 stiffness 到接触限制

输入：

```text
k_est_N_per_m
compliance
peak_force_N
```

生成：

```python
softness = clamp(k_ref / max(k_est, eps), min_softness, max_softness)
safe_preload_N = base_preload_N / softness
max_indentation_m = min(base_depth_m, max_energy_J / max(safe_preload_N, eps))
close_speed = base_close_speed / softness
```

这些参数影响：

- `press` phase 的目标力或最大压入量。
- `pick` soft box 时的 `GripPolicy.alpha_rate` 和 `target_normal_force_N` 上限。
- `failure_gates` 中的 `max_indentation_m`、`max_force_N`。

### 3.5 material 到滑移边界

输入：

```text
mu_est
friction_ratio
slide_vibration
contact_fraction
```

生成：

```python
mu_eff = clamp(mu_est, 0.15, 2.0)
min_normal_force_N = weight_N * safety_factor / mu_eff
slip_margin = clamp(mu_eff / mu_reference, 0.2, 2.0)
max_slide_speed_mps = base_slide_speed_mps * min(1.0, slip_margin)
slip_gate_threshold = base_slip_threshold / max(slip_margin, eps)
```

如果没有 `weight_N`，plan 必须记录：

```json
"missing_requirements": ["object_weight_for_material_pick"]
```

然后只能生成 `slide_contact`，不能生成严格可信的 `pick`。

## 第 4 层：SkillSelector

第一版固定位置路线不需要复杂策略，skill 可以由 task/family 或外部调用指定。

默认映射：

| family | 默认 skill | 可选 skill |
|---|---|---|
| `mass` | `pick` | `carry` |
| `fill` | `carry` | `pour` |
| `stiffness` | `press` | `pick` |
| `material` | `slide_contact` | `pick` |

如果外部指定 skill，需要检查 compatibility：

```python
if skill == "pour" and context.family != "fill":
    reject("skill_family_mismatch")

if skill == "pick" and "object_weight" missing and "mu_est" missing:
    allow_only_with_conservative_defaults()
```

## 第 5 层：FixedPosePlanGenerator

PlanGenerator 选择模板，并把 context 中的对象 pose 和控制 hint 填入模板。

输入：

```text
ManipulationContext
skill
TemplateKey(shape, skill, backend)
```

输出：

```python
@dataclass
class ManipulationPlan:
    skill: str
    target: int
    object_id: str
    phases: list[PhaseSpec]
    control_limits: dict
    success_criteria: dict
    failure_gates: list[str]
    diagnostics: dict
```

### pose 如何诞生

每个模板先定义 object-frame offset：

```python
CanSideWrapTemplate:
    pregrasp_offset = [0.0, +approach_y, +0.02]
    grasp_offset    = [0.0, +contact_y,  0.0]
    lift_offset     = [0.0, 0.0, +lift_height]
```

再由 object pose 转成 world pose：

```python
goal_world = object_pose_world @ pose_from_object_offset(offset, rpy)
```

当前 v1 对象多为规范摆放，第一版可简化为：

```python
x = object_pos[0] + offset_x
y = object_pos[1] + offset_y
z = object_pos[2] + offset_z
roll, tilt, yaw = template_rpy
```

但文档和代码里必须保留 frame 字段，避免以后随机姿态时全部重写。

### phase 如何诞生

以 `opaque_cup/carry` 为例：

```text
pregrasp_upright
  goal_pose: cup side above, cup remains upright
  controller: position
  stop: pose reached

side_contact_below_rim
  goal_pose: side contact height below rim
  controller: guarded_position
  stop: first hand/object contact or pose reached

bounded_close
  gripper: GripPolicy(force_hint_hold)
  controller: gripper_with_contact_gate
  stop: opposing grasp and force >= target

vertical_lift
  goal_pose: current pose + z lift
  controller: position_with_force_guard
  stop: support contact gone and lift distance reached

slosh_settle
  goal_pose: hold current upright pose
  controller: hold_pose
  stop: timeout + torque variation below threshold

slow_carry
  goal_pose: task-specific carry offset
  controller: slow_position_with_tilt_limit
  stop: pose reached
```

此时 phase 还不是每步 command。它只是说明下一层应该如何编译。

## 第 6 层：PhaseCompiler

PhaseCompiler 把 `PhaseSpec` 变成更接近执行的 `CompiledPhaseCommand`。

```python
@dataclass
class CompiledPhaseCommand:
    name: str
    start_pose: Pose
    goal_pose: Pose | None
    n_steps: int
    interpolation: str
    grip_policy: GripPolicy | None
    controller: str
    stop_condition: dict
    failure_gates: list[str]
    sensor_requirements: list[str]
```

### pose phase 编译

输入：

```text
current wrist pose
goal pose
max_tcp_speed_mps
max_yaw_rate_rps
timestep
```

生成：

```python
distance = norm(goal_xyz - current_xyz)
duration_s = max(distance / max_tcp_speed_mps, min_phase_duration_s)
n_steps = ceil(duration_s / timestep)

for i in range(n_steps):
    alpha = smoothstep(i / (n_steps - 1))
    cmd_pose = interpolate(current_pose, goal_pose, alpha)
```

输出到执行器的是一个可按步采样的 trajectory generator。

### gripper phase 编译

输入：

```text
target_normal_force_N
alpha_min/alpha_max
alpha_rate
timestep
```

生成：

```python
delta_alpha_per_step = alpha_rate * timestep
```

每步执行：

```python
alpha = min(alpha + delta_alpha_per_step, alpha_max)
scene.command(grip=alpha)
```

然后立刻读取 contact feedback 来决定是否继续。

### force/guard phase 编译

`position_with_force_guard` 不是真正的力控，而是位置插值 + feedback gate：

```python
scene.command(x, y, z, roll, tilt, yaw, grip)
snapshot = scene.contact_snapshot(target)
wrist_force = scene.wrist_force_vec()
wrist_torque = scene.wrist_torque_vec()

if snapshot.max_penetration_m > limit:
    fail("penetration_limit")
if lost_opposing_contact(snapshot):
    fail("lost_grasp")
if support_contact_after_lift(snapshot):
    fail("support_contact_after_lift")
```

也就是说，第一版固定位置路线的“力控”主要是：

- 用 probe estimate 生成 target force hint。
- 用 gripper alpha 逐步逼近。
- 用 contact_snapshot 和 wrist force/torque 做停止和失败判定。

它不是成熟的 impedance controller。

## 第 7 层：FixedPoseExecutor 每步控制信号

执行器循环结构：

```python
for phase in compiled_phases:
    phase_result = run_phase(phase)
    if phase_result.failed:
        return manipulation_failed(phase_result)
return manipulation_success()
```

### run_phase

每一步产生一组低层命令：

```python
LowLevelCommand:
    x: float | None
    y: float | None
    z: float | None
    roll: float | None
    tilt: float | None
    yaw: float | None
    grip: float | None
```

然后调用：

```python
scene.command(
    x=cmd.x,
    y=cmd.y,
    z=cmd.z,
    roll=cmd.roll,
    tilt=cmd.tilt,
    yaw=cmd.yaw,
    grip=cmd.grip,
)
scene.step(1)
```

### 控制信号来源表

| 控制信号 | 来源 | 生成方式 |
|---|---|---|
| `x/y/z` | phase goal pose | object pose + template offset，经速度限制插值 |
| `roll/tilt/yaw` | phase goal orientation | 模板姿态 + fill/material safety limit，经角速度限制插值 |
| `grip` | GripPolicy | target force hint + alpha 标定/搜索，经 contact gate 停止 |
| phase duration | control_limits | 距离/速度、角度/角速度、settle time 共同决定 |
| stop condition | PhaseSpec | 由 contact、force、pose reached、timeout 判断 |
| failure gate | ManipulationPlan | 每步检查 lost contact、support contact、penetration、drop 等 |

### 固定位置执行的状态机

```text
READY
→ PHASE_START
→ COMMAND_STEP
→ SENSOR_READ
→ STOP_CHECK
→ FAILURE_CHECK
→ PHASE_DONE
→ NEXT_PHASE
→ SUCCESS / FAIL
```

`STOP_CHECK` 和 `FAILURE_CHECK` 必须分开：

- stop condition 满足：当前 phase 正常结束。
- failure gate 触发：整个 manipulation 失败或进入 recovery。

例如 lift phase：

```python
stop = object_lifted and support_contact_gone
fail = penetration_limit or lost_grasp or timeout
```

## 第 8 层：反馈信号如何进入控制

第一版固定位置执行只使用仓库已有传感：

| 反馈 | 来源 | 用途 |
|---|---|---|
| `contact_snapshot.hand_groups` | MuJoCo contact buffer | 判断对向抓取是否存在 |
| `hand_normal_force_N` | contact force 汇总 | 判断是否达到 target force hint |
| `support_contact/table_contact` | contact buffer | 判断是否脱离支撑、是否掉落 |
| `max_penetration_m` | contact distance | 判断穿透/过力风险 |
| `wrist_force` | wrist sensor | 估计 lift 后负载变化、异常冲击 |
| `wrist_torque` | wrist sensor | fill/carry 中判断晃动或姿态风险 |
| `object_pos/quat` | object sensors | 判断 lift distance、drift、drop |

### feedback 到控制调整

| 条件 | 控制调整 |
|---|---|
| 接触未建立 | 继续小步闭合或微调 wrist pose |
| 达到 target force | 停止闭合，进入 hold |
| 穿透超过 limit | 回退 alpha 或 fail |
| lift 中丢失 opposing contact | 暂停并小幅增大 grip；超过次数则 fail |
| support contact 仍存在 | 继续 lift 到上限；仍存在则 fail |
| cup torque 波动过大 | 延长 settle，降低后续 speed/tilt |
| object drift 过大 | fail 或进入 regrasp |

第一版建议只实现 fail，不实现复杂 recovery；但文档和 trace 要保留 recovery 的位置。

## 第 9 层：ManipulationExecutionResult

执行结束后必须生成独立结果，不要写回 `ProbeResult`。

```python
@dataclass
class ManipulationExecutionResult:
    schema_version: str
    status: str
    valid: bool
    skill: str
    target: int
    object_id: str
    phase_reached: str
    violations: list[str]
    quality: dict[str, float]
    final_object_pose: dict
    control_summary: dict
    trace: dict
```

### status 建议

| status | 含义 |
|---|---|
| `ok` | 所有 phase 完成，success criteria 满足 |
| `no_plan` | probe 无效或缺少必要条件，未执行 |
| `phase_timeout` | 某 phase 超时 |
| `lost_contact` | 执行中丢失必要抓取接触 |
| `support_contact` | 需要 lift 后脱离支撑但未达成 |
| `penetration_limit` | 穿透或过力风险 |
| `drop` | 目标掉落或偏移过大 |
| `unsafe` | 触发安全 gate |

### quality 建议

| quality 字段 | 含义 |
|---|---|
| `phase_count_completed` | 完成的 phase 数 |
| `max_penetration_m` | 执行中最大穿透 |
| `min_opposing_contact_fraction` | 必要阶段对向接触保持比例 |
| `lift_distance_m` | 最终抬升距离 |
| `object_drift_m` | 相对 wrist 或目标轨迹的漂移 |
| `support_contact_fraction` | lift/carry 后支撑接触比例 |
| `peak_wrist_force_N` | 最大腕部力 |
| `peak_wrist_torque_Nm` | 最大腕部力矩 |
| `mean_grip_alpha` | 平均闭合程度 |
| `peak_grip_alpha` | 最大闭合程度 |

### trace 建议

第一版 trace 可以按 phase 压缩：

```json
{
  "phases": [
    {
      "name": "bounded_close",
      "steps": 120,
      "start_time": 0.8,
      "end_time": 1.4,
      "final_grip_alpha": 0.47,
      "final_hand_force_N": 13.8,
      "stop_reason": "target_force_reached",
      "violations": []
    }
  ]
}
```

如果要调试，再保留 full time series。

## 完整例子：mass / short_can / pick

### 输入 ProbeResult

```json
{
  "primitive": "heft",
  "target": 1,
  "valid": true,
  "features": {
    "m_est_kg": 0.42,
    "weight_signal_N": 4.12
  },
  "quality": {
    "lift_distance_m": 0.026,
    "support_contact_after_lift": 0.0,
    "postlift_group_count": 2.0,
    "postlift_stable_fraction": 0.92
  },
  "violations": []
}
```

### Context

```json
{
  "family": "mass",
  "shape": "short_can",
  "backend": "allegro",
  "estimates": {
    "m_est_kg": 0.42,
    "weight_signal_N": 4.12,
    "mu_eff": 0.7
  },
  "confidence": {
    "overall": 0.92
  },
  "safety_limits": {
    "max_tcp_speed_mps": 0.06,
    "max_accel_mps2": 0.20,
    "penetration_limit_m": 0.005
  },
  "missing_requirements": ["measured_mu_for_grip"]
}
```

### 控制 hint

```python
weight_N = 0.42 * 9.81           # 4.12 N
raw_force = 4.12 * 2.5 / 0.7     # 14.7 N
target_normal_force_N = clamp(14.7 + 2.0, 4.0, 30.0)  # 16.7 N
```

### Plan phase

```text
pregrasp_above_side:
  pose = object_pos + [0, approach_y, 0.04]
  grip = open

guarded_close:
  grip_policy = close_until hand_force >= 16.7 N

lift_vertical:
  pose = current + [0, 0, 0.05]
  stop = support_contact_gone and lift_distance >= 0.02
```

### 每步 command

```python
scene.command(x=interp_x, y=interp_y, z=interp_z, roll=0, tilt=0, yaw=0)
scene.command(grip=alpha)
```

其中 `alpha` 每步增加，直到：

```python
snapshot.hand_normal_force_N >= 16.7
and scene.has_opposing_grasp(snapshot)
```

这时停止闭合，进入 lift。

## 完整例子：fill / opaque_cup / carry

### 输入 ProbeResult

```json
{
  "primitive": "shake",
  "target": 0,
  "valid": true,
  "features": {
    "weight_proxy_N": 2.2,
    "fill_proxy": 0.78,
    "slosh_proxy": 0.31,
    "torque_peak_Nm": 0.69
  },
  "quality": {
    "lift_distance_m": 0.024,
    "support_contact_after_lift": 0.0
  },
  "violations": []
}
```

### Context 到运动包络

```python
slosh_norm = normalize(0.31)
slosh_scale = 1.0 + 1.5 * slosh_norm
max_tcp_speed_mps = 0.08 / slosh_scale
max_tilt_rad = 0.35 / slosh_scale
settle_time_s = 0.4 * slosh_scale
```

### Plan phase 差异

与 can 不同，cup 额外生成：

```text
slosh_settle:
  hold upright pose
  duration = settle_time_s

slow_carry:
  max_tcp_speed_mps = reduced speed
  max_tilt_rad = reduced tilt
  failure = wrist_torque spike or cup tilt beyond limit
```

关键点：`slosh_proxy` 不直接生成某个 motor command，而是降低后续 phase 的速度、角速度
和姿态变化上限。

## 控制信号诞生的最终总结

```text
ProbeResult.features
  → 物理估计: mass / fill / stiffness / friction
  → 控制 hint: target force / speed limit / tilt limit / indentation limit
  → Plan phase: pregrasp / close / lift / carry / press / slide
  → Compiled phase: goal pose + step count + grip policy + gates
  → 每步命令: x/y/z/roll/tilt/yaw/grip
  → 传感反馈: contact / force / torque / object pose
  → stop/fail decision
  → ManipulationExecutionResult
```

要特别区分三件事：

1. `target_normal_force_N` 是物理期望，不是 actuator command。
2. `grip_alpha` 是 backend-specific 的执行变量，需要标定和反馈 gate。
3. `scene.command(...)` 才是当前 MuJoCo v1 里真正下发的低层控制信号。

## 当前仍需调研或实验验证的断点

1. `target_normal_force_N → grip_alpha` 的标定曲线。
2. `hand_normal_force_N` 作为闭合停止条件的稳定性和噪声。
3. 不同 shape 的固定模板接触高度和 palm 姿态。
4. cup 的 `slosh_proxy → speed/tilt limit` 标定。
5. soft box 的 `k_est → safe_preload/max_indentation` 标定。
6. material 的 `mu_est → slip gate/min normal force` 是否能跨接触模式复用。
7. 固定位置 plan 在 object pose 扰动下的鲁棒性。
8. ManipulationExecutionResult 的 success criteria 是否足以和 benchmark success 分离。

这些断点解决前，固定位置路线只能被描述为“规范化场景中的可解释计划生成与执行”，
不能描述为通用 manipulation。

其中 `short_can_pick_place` 已实现显式 16-D hand template、probe-conditioned plan、
抓取/搬运/放置状态机和独立执行结果，详见
`docs/v1/sgst_mani_3_short_can_pick_place.md`。
