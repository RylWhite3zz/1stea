# 固定位置 Manipulation 后续设计草案与问题清单

## 目标与范围

本文承接 `sgst_mani_0.md` 的接口路线：

```text
ProbeResult
→ ManipulationContext
→ ManipulationPlan
→ FixedPoseExecutor / PlanningControlExecutor
```

当前阶段优先设计 **FixedPoseExecutor** 这条线。目标不是马上做出通用 manipulation，
而是先把“probe 后如何生成一个可执行的、阶段化的固定位置计划”设计清楚，并提前
列出必须调研或实验验证的问题。

范围限制：

- 只面向当前 v1 的四类 family：`stiffness / mass / fill / material`。
- 只面向当前 demo 对象族：`compressible_box / short_can / opaque_cup /
  surface_block`。
- 固定位置不等于通用抓取规划；它只在规范化摆放、已知对象尺寸、已知目标位姿的
  前提下工作。
- 不实现机械臂、IK、任意 mesh 抓取、避碰规划或真实任务成功判定。

## 总体数据流

```text
ProbeResult
  ├─ valid / violations / quality
  └─ features
        ↓
ManipulationContextBuilder
        ↓
ManipulationContext
  ├─ object geometry / current pose
  ├─ physical estimates
  ├─ confidence / quality gates
  └─ safety limits
        ↓
FixedPosePlanGenerator
        ↓
ManipulationPlan
  ├─ skill
  ├─ object-specific fixed pose template
  ├─ gripper / force hints
  ├─ phase stop conditions
  └─ failure gates
        ↓
FixedPoseExecutor
```

第一版建议先只做 **plan generation**，不急着做完整 executor。也就是说，先确保：

1. `ProbeResult` 能稳定转成 `ManipulationContext`。
2. `ManipulationContext` 能稳定转成对象特定的 `ManipulationPlan`。
3. plan 里的每个 phase、pose、force hint、gate 都能解释为什么这样设。

## 固定位置不是单一抓取动作

固定位置路线最容易踩的坑是把所有物体都当成“一个固定 pregrasp + 一个固定 close”
处理。这个假设不成立。

| 对象 | 推荐固定模板 | 不能共用的原因 |
|---|---|---|
| `short_can` | 侧向环抱/对向抓取，接触高度靠近中部或略低于中部 | 重点是承重和抗滑，质量变化会改变所需法向力 |
| `opaque_cup` | 保持杯身直立的低加速度侧向抓取，避开杯口/上沿 | 杯内液体带来晃动和倾倒风险，动作包络应比 can 保守 |
| `compressible_box` | 控力压接或低力夹持，限制压入量 | 软物体可能被过度挤压，不能用 heavy can 的闭合策略 |
| `surface_block` | 面接触优先的侧向抓取或 preload slide | 材质/摩擦决定滑移风险，接触法向和表面方向更重要 |

因此第一版模板库应该至少按 `shape + skill + backend` 分开，而不是只按
`skill` 分开。

建议结构：

```python
TemplateKey = tuple[str, str, str]  # (shape, skill, backend)

FIXED_POSE_TEMPLATES = {
    ("short_can", "pick", "allegro"): CanSideWrapTemplate(...),
    ("opaque_cup", "carry", "allegro"): CupUprightCarryTemplate(...),
    ("compressible_box", "press", "allegro"): BoxForceLimitedPressTemplate(...),
    ("surface_block", "slide_contact", "allegro"): SurfaceSlideTemplate(...),
}
```

## ManipulationContext 设计细化

第一版 `ManipulationContext` 应该只承载下游控制确实需要的信息。

建议字段：

```python
@dataclass
class ManipulationContext:
    schema_version: str
    scene_id: str
    family: str
    target: int
    object_id: str
    shape: str
    size_m: tuple[float, float, float]
    object_pose: dict
    probe_valid: bool
    probe_primitive: str
    estimates: dict[str, float]
    quality: dict[str, float]
    violations: list[str]
    confidence: dict[str, float]
    safety_limits: dict[str, float]
    missing_requirements: list[str]
```

`missing_requirements` 很关键。许多 manipulation 参数不能只靠当前 family 的一次
probe 得到。例如：

- `mass` 任务知道质量，但未必知道摩擦。
- `material` 任务知道摩擦，但未必知道质量。
- `fill` 任务有晃动 proxy，但不一定有绝对液体质量。
- `stiffness` 任务知道局部刚度，但不一定知道整体可抓取稳定性。

如果 plan 需要某个缺失量，第一版不要假装它已知，而是明确写入：

```json
{
  "missing_requirements": ["mu_est_for_grip_force"],
  "fallback_policy": "use_conservative_default"
}
```

## Context 到力道大小的第一版映射

### 基本思路

抓取力不是直接从 `m_est_kg` 线性复制出来的。更合理的是把它看成一个保守估计：

```text
required_normal_force
≈ object_weight × safety_factor / effective_friction
```

可以先用这个近似：

```python
weight_N = max(m_est_kg * 9.81, min_weight_N)
mu_eff = clamp(mu_est_or_default, 0.25, 1.5)
force_N = weight_N * safety_factor / mu_eff
target_normal_force_N = clamp(force_N + preload_N, min_force_N, max_force_N)
```

`fill` 可以使用 `weight_proxy_N`，但必须承认它目前更像 proxy，不是严格标定质量：

```python
weight_N = max(weight_proxy_N, conservative_fill_weight_N)
slosh_scale = 1.0 + k_slosh * normalized_slosh_proxy
target_normal_force_N = base_force_N * slosh_scale
max_tcp_speed_mps = base_speed_mps / slosh_scale
max_tilt_rad = base_tilt_rad / slosh_scale
```

### 第一版默认策略

| family | 主要输入 | 输出参数 | 默认策略 |
|---|---|---|---|
| `mass` | `m_est_kg` | `target_normal_force_N`、`max_tcp_speed_mps`、`max_accel_mps2` | 质量越大，抓取力越大，速度/加速度越小 |
| `fill` | `weight_proxy_N`、`slosh_proxy` | `target_normal_force_N`、`max_tilt_rad`、`max_yaw_rate_rps` | 晃动越强，动作越慢，姿态变化越小 |
| `stiffness` | `k_est`、`compliance` | `safe_preload_N`、`max_indentation_m` | 越软越限制接触力和闭合速度 |
| `material` | `mu_est`、`friction_ratio` | `min_normal_force_N`、`slip_gate_threshold` | 摩擦越低，法向力裕度越大，滑移 gate 越严格 |

### 这里的关键问题

1. `grip_alpha` 不是力控命令。Allegro 后端当前用关节位置/闭合程度驱动，不能保证
   某个 alpha 对应某个法向力。
2. MuJoCo contact buffer 里的 `hand_normal_force_N` 是接触求解结果，不等价于实际
   可控的指尖法向力。
3. `mu_est` 只在 material probe 中自然得到；mass/fill 的抓取力计算仍然缺摩擦。
4. `m_est_kg` 是基于 heft 的估计，受抬升动态、基线、姿态和接触稳定性影响。
5. `fill_proxy` / `slosh_proxy` 是当前仿真的代理量，不能直接当成真实液位或真实
   液体质量。
6. 同一 `target_normal_force_N` 对 reference gripper 和 Allegro 的含义不同，必须
   分 backend 标定。
7. 对软物体，增大法向力可能提高稳定性，也可能破坏物体或改变任务属性。

因此第一版实现时，建议把“力道大小”拆成两层：

```text
desired physical force hint
→ backend-specific alpha/actuator policy
```

其中第二层必须通过仿真扫描或闭环接触反馈标定，不能只靠公式。

## 固定位置 Plan 模板

### 1. `short_can` 的 `pick/carry`

目标：抓起并短距离搬运外观一致但质量不同的罐。

推荐 phase：

```text
pregrasp_above_side
→ lateral_align
→ guarded_close
→ squeeze_to_force_hint
→ lift_vertical
→ stabilize_postlift
→ carry_or_retreat
```

关键参数：

- 抓取高度：罐体中部或略低于中部，避免太靠近顶部导致翻转。
- 抓取方式：thumb + 至少一个对向手指，优先形成多点包络。
- 力道：由 `m_est_kg` 和保守摩擦默认值决定。
- gate：opposing contact、support/table contact gone、postlift drift、penetration。

待解决问题：

- 当前 Allegro 的固定 wrist pose 只是为了 heft 调过，未必适合 carry。
- 对圆柱体，手指应该接触圆柱侧面，而不是靠穿透或底缘托举。
- 质量越大时，单纯增大 close alpha 可能导致穿透或弹飞。

### 2. `opaque_cup` 的 `carry/pour`

目标：识别欠满/满杯后稳定端起或移动，必要时受限倾倒。

推荐 phase：

```text
pregrasp_upright
→ side_contact_below_rim
→ bounded_close
→ vertical_lift
→ slosh_settle
→ slow_carry
→ optional_conservative_tilt
```

与 `short_can` 的关键区别：

- 杯子必须默认保持直立；yaw/tilt 的速度限制比 can 更严格。
- 抓取点应避开杯口和上沿，防止模拟中出现“卡住杯口”的假稳定。
- `slosh_proxy` 越高，越应该降低搬运速度和姿态变化。
- 如果执行 `pour`，第一版只能做很保守的小角度模板，不能声称真实液体控制。

待解决问题：

- 当前 fill 是 slosh proxy，不是真实流体；“洒出”成功判据没有真实液体体积。
- 满杯和欠满杯的质量、晃动、稳定性耦合，单一 `fill_proxy` 不够决定操作。
- 杯子的抓取策略可能需要避开薄壁/软壁，但当前对象几何很简化。
- carry 和 pour 的成功标准需要另行定义，否则 plan 只能验证姿态/接触，不验证任务。

### 3. `compressible_box` 的 `press/pick`

目标：根据刚度结果选择压接或低力抓取策略。

推荐 phase：

```text
precontact
→ guarded_press
→ force_or_depth_limited_hold
→ release
```

如果后续要 `pick` 软方块，不能直接复用 can/cup 的强闭合策略：

- 越软，`safe_preload_N` 越低。
- 越软，`close_speed` 越低。
- `max_indentation_m` 必须作为硬 gate。

待解决问题：

- 当前 stiffness 对象的可压缩性主要通过一个 top joint/proxy 表达，不等于真实软体。
- 软方块的“可抓起”和“可压缩探测”是两个不同问题。
- 即使是方块，Allegro 多指接触点也不好通过 IK 直接算出；第一版应采用模板接近 +
  接触反馈闭合，而不是求解精确指尖位姿。

### 4. `surface_block` 的 `slide_contact/pick`

目标：根据摩擦 probe 结果设置滑动或抓取策略。

推荐 phase：

```text
precontact
→ preload_to_force
→ tangential_motion
→ contact_quality_check
→ optional_pick_with_slip_margin
```

待解决问题：

- `surface_block` 更像测试表面，不一定是自然可抓取物体。
- slide 的 `mu_est` 来自 preload 下滑动，不一定等于抓取接触时的有效摩擦。
- 低摩擦时增大法向力可能有效，也可能超过对象/手指接触安全范围。

## 固定位置执行器的最小接口

第一版 `FixedPoseExecutor` 可以只接受 `ManipulationPlan`，并按 phase 执行：

```python
class FixedPoseExecutor:
    def execute(self, plan: ManipulationPlan) -> ManipulationExecutionResult:
        for phase in plan.phases:
            self.move_to_pose_or_hold(phase.goal_pose)
            self.apply_gripper_policy(phase.gripper)
            self.wait_until(phase.stop_condition, phase.timeout_s)
            self.check_failure_gates(plan.failure_gates)
        return result
```

执行结果建议字段：

```python
@dataclass
class ManipulationExecutionResult:
    status: str
    valid: bool
    phase_reached: str
    violations: list[str]
    quality: dict[str, float]
    final_object_pose: dict
    trace: dict
```

注意：这个结果应独立于 `ProbeResult`，不能把 manipulation 成功和 probe 属性估计混在
同一个 status 里。

## 当前已知问题清单

下面这些问题建议作为后续调研/实验的输入，而不是在第一版里回避掉。

### A. 抓取模板与对象几何

1. 方块、圆柱罐、杯子、表面块不能共用同一个 Allegro 抓取模板。
2. 杯子需要保持直立并避开杯口；罐子更关注承重；软方块更关注限力。
3. 当前对象是解析几何 proxy，固定模板可能在真实 mesh 上失效。
4. `shape` 只有粗标签，不足以描述可抓取区域、禁抓区域、薄壁、把手、杯口等 affordance。
5. 固定位置模板依赖对象初始姿态；如果对象有 yaw/roll/pitch 随机化，模板需要跟随
   object frame。

### B. Allegro 手本体控制

1. Allegro 的 16 个关节不能简单通过“目标接触点 IK”稳定解出自然抓取。
2. 当前 `grip_alpha` 是抽象闭合程度，不是物理法向力。
3. 手指接触顺序会影响最终抓取；同一个 alpha 可能得到不同接触组合。
4. thumb + opposing finger 的 gate 只说明有对向接触，不说明 grasp wrench closure 足够。
5. 过度依赖底缘托举会让“抓取成功”变成几何卡住，不是真正稳定抓取。
6. reference backend 和 Allegro backend 的同一 plan 不能假设等价。

### C. Context 到控制参数的映射

1. 抓取力需要质量和摩擦共同决定，但单个 family 的 probe 通常只提供其中一部分。
2. mass/fill 场景缺少 `mu_est` 时，只能用保守默认值或要求额外 material probe。
3. material 场景有 `mu_est`，但缺少目标质量时，抓取力仍然无法严格计算。
4. `fill_proxy`、`slosh_proxy` 当前未标定成真实液位/质量/晃动幅值。
5. `m_est_kg` 和 `weight_proxy_N` 的可信度应依赖 `ProbeResult.quality`，不能只看 feature。
6. 需要建立 backend-specific 的 `target_normal_force_N → grip_alpha/actuator command`
   标定表。

### D. 固定位置与定位

1. 当前 probe 控制里部分动作仍依赖 candidate slot；manipulation 应优先使用
   `object_pos/object_quat`。
2. 固定位置模板如果没有对象坐标系定义，会在对象位姿扰动下失效。
3. 只靠 top site 的 pose 不一定能描述杯口、侧壁、底缘等关键部位。
4. 预抓取路径可能和其他候选物、pedestal 或 table 发生碰撞。
5. 当前 6-DoF wrist carriage 不是机械臂；它没有关节极限、连杆避碰和真实可达性。

### E. 安全和成功判据

1. carry 成功、pour 成功、press 成功的判据目前还没有定义。
2. fill 没有真实液体，不能直接评估真实 spill volume。
3. 软物体是否“损伤”需要定义最大压入量或形变能量 proxy。
4. material 低摩擦时，滑移是失败还是可恢复状态，需要明确策略。
5. manipulation 的失败不能回写成 probe 失败；两者应分开记录。

### F. 调试与标定

1. 需要扫描 `grip_alpha` 与接触力、穿透、抓取稳定性的关系。
2. 需要为每个 shape/backend 标定安全 close speed 和 force limit。
3. 需要记录 phase-level trace，便于定位失败发生在 pregrasp、close、lift 还是 carry。
4. 需要设计“固定模板失败时”的重试策略：微调 wrist pose、减小/增大 alpha、换接触高度。
5. 需要避免在仿真中利用 pedestal、底缘、碰撞厚度获得不真实的稳定性。

## 优先调研问题

建议后续调研按以下顺序展开：

1. **Allegro 固定模板抓取库**：针对 box/cylinder/cup/surface 的手掌姿态、接触高度、
   thumb/finger 配置如何设计。
2. **`grip_alpha → 接触力` 标定**：不同对象、不同 backend、不同接触高度下，alpha、
   normal force、penetration、lift success 的关系。
3. **抓取力公式的工程化**：如何用 `m_est_kg`、默认/估计摩擦、safety factor 转成
   target force，再转成可执行 actuator command。
4. **杯子搬运模板**：如何限制 tilt/yaw/accel，如何定义 slosh proxy 到速度/倾角的
   映射。
5. **软物体限力策略**：如何从 `k_est/compliance` 生成 safe preload、close speed 和
   max indentation。
6. **固定位置验收指标**：每个 skill 的 success criteria、failure gate、trace 字段
   应该如何定义，才能和 probe result 分离。

## 建议的下一步最小实现

如果进入代码阶段，建议只做下面三件事：

1. 新增 dataclass：
   - `ManipulationContext`
   - `PhaseSpec`
   - `ManipulationPlan`
   - `ManipulationExecutionResult`
2. 新增纯函数：
   - `build_manipulation_context(task, scene, probe_result)`
   - `generate_fixed_pose_plan(context, skill)`
3. 新增测试，不执行真实 manipulation，只验证 plan 参数方向：
   - 重物比轻物生成更大 `target_normal_force_N` 和更低 `max_tcp_speed_mps`。
   - 高 `slosh_proxy` 比低 `slosh_proxy` 生成更低 `max_tilt_rad`。
   - 软物体比硬物体生成更低 `safe_preload_N`。
   - 低摩擦比高摩擦生成更高 `min_normal_force_N` 或更严格 slip gate。
   - `probe_valid == false` 时不生成可执行 plan。

这样可以先把接口和逻辑方向跑通，不把系统提前推进到“通用灵巧手抓取”这个更难的
问题上。

上述路线的第一条真实执行闭环已经落到 `mass / short_can / allegro`，实现、关键问题
处理和 3 seed × 3 target 验收见 `docs/v1/sgst_mani_3_short_can_pick_place.md`。
