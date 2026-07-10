# Probe 结果到 Manipulation 规划控制接口设计报告

## 执行摘要

当前仓库已经把 **probe 执行层** 的边界定义得比较清楚：上层发出
`ProbeCommand`，`ProbeHarness.execute()` 执行 `poke / heft / shake / slide`
四种固定原语，并返回带 `valid / violations / quality / features /
raw_summary / trace` 的 `ProbeResult`。`ManipulationCommand` 与
`ManipulationController` 仍只是占位协议；因此后续真正需要补的是
**probe 结果如何被下游 manipulation 规划/控制消费**，而不是把 probe 层改成完整
manipulation 系统。

最合理的路线是保留现有 probe 层不动，在其后新增一个很薄的桥接层：

```text
ProbeResult
→ ManipulationContext
→ ManipulationPlan
→ FixedPoseExecutor / PlanningControlExecutor
```

这里的 `ManipulationContext` 是 probe 后得到的物理语义包；`ManipulationPlan`
是阶段化的规划/控制规格；最终执行可以有两种模式：

- **固定位置模式**：每个技能绑定预定义位姿和轨迹模板，适合当前可控仿真任务。
- **规划控制模式**：同一个 plan 被外部 IK、轨迹规划、阻抗/力控控制器消费。

这条路线避免把高频控制、机械臂规划和最终任务成功判定混进当前仓库，同时又把
probe 输出整理成下游真正能用的接口。

## 必须保留的不变边界

| 边界 | 当前状态 | 对后续接口的含义 |
|---|---|---|
| 四类 family 与 probe 原语映射 | `stiffness→poke`、`mass→heft`、`fill→shake`、`material→slide` | manipulation 接口不能回写改变 probe 词表 |
| `ProbeHarness.execute(ProbeCommand)->ProbeResult` | 当前只执行一个 probe 命令 | 新接口应接在 probe 之后，不侵入 `ProbeCommand / ProbeResult` |
| 显式分阶段状态机 | `approach → contact → gate → execution → post-check → retreat` | manipulation plan 也应采用阶段化、可审计结构 |
| 编译期固定碰撞角色 | probe/hand 碰撞角色由 scene 类型决定 | 后续技能不能依赖运行时切换碰撞角色制造捷径 |
| 统一传感语义 | `probe_force`、`wrist_force`、`wrist_torque`、contact snapshot 等 | 下游只能消费公开传感摘要和 probe 结果 |
| `valid` 与 `features` 分离 | 控制失败不会伪装成属性估计成功 | manipulation 必须先检查 probe 是否可信 |
| 不暴露 hidden answer | `ObjectSpec` 区分 visible 与 hidden 字段 | 桥接层不能把隐藏真值当作输入 |

当前 MuJoCo 里的 wrist 是 6-DoF task-space carriage，不是机械臂、IK、关节空间避碰
或通用运动规划。后续接口可以为外部规划器预留字段，但本仓库不实现这些能力。

## 新增接口的核心概念

### ManipulationContext

`ManipulationContext` 是从 `ProbeResult` 和公开场景观测整理出来的条件化输入。它不
表示动作，也不表示轨迹，只描述“下游操作应该依据哪些物理估计和安全边界”。

建议字段：

```python
@dataclass
class ManipulationContext:
    schema_version: str
    scene_id: str
    family: str
    target: int
    object_id: str
    source_probe: dict
    estimates: dict
    confidence: dict
    quality: dict
    violations: list[str]
    safety_limits: dict
    required_feedback: list[str]
```

核心约定：

- `source_probe.valid == false` 时，默认不生成可信 manipulation plan。
- `estimates` 只放 probe 可见或由 probe 推断出的量，例如 `m_est_kg`、`mu_est`、
  `k_est`、`fill_proxy`。
- `quality` 和 `violations` 必须透传，供下游决定是否重试、降级或停止。
- `safety_limits` 是建议边界，不是底层控制命令。

示例：

```json
{
  "schema_version": "probe_to_manip.v1",
  "scene_id": "demo_fill_000123",
  "family": "fill",
  "target": 1,
  "object_id": "obj1",
  "source_probe": {
    "primitive": "shake",
    "valid": true,
    "features": {
      "weight_proxy_N": 2.31,
      "fill_proxy": 0.84,
      "slosh_proxy": 0.19,
      "torque_peak_Nm": 0.72
    }
  },
  "estimates": {
    "fill_proxy": 0.84,
    "slosh_proxy": 0.19,
    "weight_proxy_N": 2.31
  },
  "confidence": {
    "overall": 0.81
  },
  "quality": {
    "lift_distance_m": 0.027,
    "support_contact_after_lift": 0.0
  },
  "violations": [],
  "safety_limits": {
    "max_tilt_rad": 0.35,
    "max_tcp_speed_mps": 0.08,
    "max_yaw_rate_rps": 0.5
  },
  "required_feedback": ["wrist_force", "wrist_torque", "contact_snapshot"]
}
```

### ManipulationPlan

`ManipulationPlan` 是下游规划/控制器真正消费的结构。它仍然不是高频控制信号，而
是由多个阶段组成的任务级执行规格。

建议字段：

```python
@dataclass
class PhaseSpec:
    name: str
    goal_pose: dict | None
    gripper: dict | None
    controller: str
    stop_condition: dict
    timeout_s: float


@dataclass
class ManipulationPlan:
    schema_version: str
    skill: str
    target: int
    phases: list[PhaseSpec]
    control_limits: dict
    success_criteria: dict
    failure_gates: list[str]
```

第一版建议只支持这些 skill：

| skill | 用途 | 典型 family |
|---|---|---|
| `pick` | 抓起目标物 | `mass`、`material` |
| `carry` | 稳定搬运 | `mass`、`fill` |
| `pour` | 受限倾倒 | `fill` |
| `press` | 控力压接 | `stiffness` |
| `slide_contact` | 保持 preload 的表面滑动 | `material` |

固定位置模式下，`goal_pose` 可以直接绑定到预定义模板；规划控制模式下，
`goal_pose` 是外部规划器的目标约束。

示例：

```json
{
  "schema_version": "manip_plan.v1",
  "skill": "carry",
  "target": 1,
  "phases": [
    {
      "name": "pregrasp",
      "goal_pose": {
        "frame": "object",
        "position_offset_m": [0.0, 0.0, 0.08],
        "rpy_rad": [0.0, 0.0, 0.0]
      },
      "gripper": {"mode": "open"},
      "controller": "position",
      "stop_condition": {"type": "pose_reached"},
      "timeout_s": 2.0
    },
    {
      "name": "close",
      "goal_pose": null,
      "gripper": {
        "mode": "force_closure",
        "target_normal_force_N": 14.0,
        "close_speed": 0.10
      },
      "controller": "gripper_force",
      "stop_condition": {"type": "opposing_contact"},
      "timeout_s": 1.5
    },
    {
      "name": "lift",
      "goal_pose": {
        "frame": "object",
        "position_offset_m": [0.0, 0.0, 0.05],
        "rpy_rad": [0.0, 0.0, 0.0]
      },
      "gripper": {
        "mode": "hold_force",
        "target_normal_force_N": 14.0
      },
      "controller": "position_with_force_guard",
      "stop_condition": {
        "type": "support_contact_gone",
        "min_lift_m": 0.02
      },
      "timeout_s": 2.0
    }
  ],
  "control_limits": {
    "max_tcp_speed_mps": 0.08,
    "max_accel_mps2": 0.25,
    "max_tilt_rad": 0.35
  },
  "success_criteria": {
    "object_lifted": true,
    "no_support_contact": true,
    "no_drop": true
  },
  "failure_gates": [
    "lost_opposing_contact",
    "support_contact_after_lift",
    "penetration_limit",
    "timeout"
  ]
}
```

## 四类 probe 结果到 manipulation 参数的映射

| family | 已有 probe 输出 | 下游使用方式 |
|---|---|---|
| `mass` | `m_est_kg`、`weight_signal_N`、lift quality | 初始化抓取法向力、抬升速度、加速度上限；质量越大，动作越保守 |
| `fill` | `weight_proxy_N`、`fill_proxy`、`slosh_proxy`、`torque_peak_Nm` | 限制倾角、yaw/tilt 速率和搬运速度；晃动越强，轨迹越平滑 |
| `stiffness` | `k_est`、`compliance`、force-depth curve quality | 设置最大预压力、最大压入量、接触停留时间和阻抗参数 |
| `material` | `mu_est`、`friction_ratio`、`Ft/Fn`、`slide_vibration` | 设置抓取力裕度、接触模式和滑动速度；摩擦越低，滑移 gate 越严格 |

### mass / fill

`mass` 与 `fill` 的核心不是重新估计物理量，而是把现有估计转成稳定抓取和搬运的
控制边界。

| 观测 | 控制含义 | 建议动作 |
|---|---|---|
| `m_est_kg` 或 `weight_proxy_N` 较大 | 目标更重 | 增大目标法向力，降低抬升速度和加速度 |
| `slosh_proxy` 较大 | 内容物晃动明显 | 降低搬运速度，限制倾角和 yaw/tilt 速率 |
| `support_contact_after_lift > 0` | 物体未真正脱离支撑 | plan 无效，重抓或停止 |
| `lost_opposing_contact` | 抓取不稳定 | 增力、暂停、重抓或停止 |

第一版可以采用规则化控制：开环用 probe 估计初始化参数，执行中用 wrist force、
wrist torque 和 contact snapshot 做 gate 与恢复。

### stiffness / material

`stiffness` 与 `material` 更适合输出解释性强的连续物理参数，而不是只给离散标签。

| 输出层 | `stiffness` | `material` | 消费方 |
|---|---|---|---|
| 连续参数 | `k_est`、`compliance` | `mu_est`、`friction_ratio` | 规划/控制器 |
| 置信度 | 曲线有效接触比例、最大压入量 | 有效滑动比例、preload 稳定性 | gate 与重试策略 |
| 安全边界 | `max_indentation_m`、`safe_preload_N` | `min_normal_force_N`、`max_slide_speed_mps` | 低层控制器 |

## 执行模式

### 固定位置模式

固定位置模式适合当前仿真验证。它不做通用抓取规划，而是根据目标物体的已知几何
和当前位姿，绑定一组模板位姿：

```text
pregrasp
→ approach
→ close/contact
→ lift or press/slide
→ task-specific motion
→ retreat/release
```

优点是实现简单、可复现、方便调试 probe 到 manipulation 的接口是否合理。缺点是
只适合当前规范化场景，不能声称解决任意摆放、任意 mesh 或真实机械臂规划。

### 规划控制模式

规划控制模式消费相同的 `ManipulationPlan`，但把每个阶段的 `goal_pose`、接触
条件和安全边界交给外部规划/控制系统处理。该模式要求外部系统自己解决 IK、避碰、
轨迹生成和实时控制，本仓库只负责提供条件化参数和验收 gate。

因此，接口设计上应保证：

- plan 中的 pose 使用明确 frame，例如 `world`、`object`、`tcp_local`。
- 控制器类型使用有限枚举，例如 `position`、`force`、`impedance`、
  `position_with_force_guard`。
- stop condition 和 failure gate 必须可日志化、可回放。
- probe 无效时不能默认生成执行 plan。

## 验收与评估建议

后续 manipulation 接口的验收不应只看“是否生成了命令”，而应看命令是否正确消费
probe 结果。

最小验收项：

1. `ProbeResult.valid == false` 时不生成可信 manipulation plan。
2. `mass` 估计变大时，计划中的抓取力增大、抬升速度下降。
3. `fill` 的 `slosh_proxy` 增大时，计划中的倾角和 yaw/tilt 速率上限下降。
4. `stiffness` 的 `k_est` 变小时，压接/抓取力上限下降。
5. `material` 的 `mu_est` 变小时，滑移 gate 更严格、法向力裕度增大。
6. 每个 phase 都有明确 stop condition、timeout 和 failure gate。

## 分阶段实施路线

| 阶段 | 目标 | 交付物 | 风险控制 |
|---|---|---|---|
| 第一阶段 | 定义 probe 后接口 | `ManipulationContext`、`ManipulationPlan` dataclass/JSON schema | 只做数据结构，不实现完整 manipulation |
| 第二阶段 | 打通固定位置模式 | `pick/carry/press/slide_contact` 的模板 plan 生成器 | 只支持当前规范化 demo scene |
| 第三阶段 | 加入规划控制适配 | 将 `ManipulationPlan` 转成外部控制器目标和 gate | 外部系统负责 IK、避碰和实时控制 |
| 第四阶段 | 扩展任务族 | 支持 `pour`、更复杂 fill/stiffness 行为 | 先保守轨迹，再放开动作包络 |

当前最优先的最小实现不是最终操作系统，而是把已有 probe 输出稳定地转成
`ManipulationContext` 和 `ManipulationPlan`。这样既保留现有执行层边界，也让后续
固定位置模板和规划控制后端共享同一套接口。

固定位置路线的进一步设计、对象特定模板和问题清单见
`docs/v1/sgst_mani_1_fixed_pose.md`。

从 `ProbeResult` 到 manipulation 执行结束的逐层数据流、控制 hint、phase 编译和
每步 `scene.command(...)` 信号生成过程见 `docs/v1/sgst_mani_2_dataflow.md`。

第一条已落地的 Allegro `short_can_pick_place`、关键问题处理和实跑验收见
`docs/v1/sgst_mani_3_short_can_pick_place.md`。
