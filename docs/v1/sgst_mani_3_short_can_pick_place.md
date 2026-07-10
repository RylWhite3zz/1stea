# Allegro short_can pick/place 落地方案与验证记录

> 兼容性说明（2026-07-10）：本文记录的是 `short_can_pick_place` v1 side-wrap。
> 它现在只允许在显式 `allegro_grasp_lift=0.09 / full_hand_collisions=False /
> wrist_roll_limit_rad=0.9` 的隔离 scene 中执行；安全默认 probe 已改为 support-free、
> full-collision top pinch。当前实时位姿主线见 `0710/learning_free_pose_pick_place.md`。

## 文档状态

本文记录当前已经落地的第一条 probe-conditioned manipulation 纵向切片：

```text
valid Allegro heft ProbeResult
→ short_can ManipulationContext
→ fixed ManipulationPlan
→ explicit 16-DoF Allegro hand template
→ pick / carry / place closed loop
→ ManipulationExecutionResult
```

对应实现：

- `allegro_probe/manipulation.py`
- `allegro_probe/scene.py`
- `examples/run_short_can_pick_place.py`
- `tests/test_manipulation.py`

它只解决 `mass / short_can / allegro / fixed pose`，不是通用 manipulation。

## 明确目标

在以下前提下，把上层已经选定的 short can 从源 pedestal 抓起，搬到固定目标区并
稳定放下：

- 对象为当前解析几何 `short_can`。
- 初始位姿为规范化 demo pose。
- 对象位姿来自 MuJoCo public scene sensor；当前仍属于 oracle pose。
- 腕部由理想 6-DoF carriage 执行，不包含机械臂 IK、可达性和连杆避碰。
- Allegro 使用位置 actuator，但 manipulation 直接下发 16 个关节目标。
- 输入必须包含同 target 的可信 Allegro `heft ProbeResult`。
- 上层 target selection 不在本仓库实现。

Context 使用字段名 `collision_size_m`，其数值保持 MuJoCo geom 约定：short can 为
`(radius_x, radius_y, half_height)`；不把它误写成 full-size。对象四元数沿用 MuJoCo
`wxyz` 顺序。

任务终点不是“生成 plan”或“把物体抬起来”，而是：

```text
抓取合法
∧ 脱离源支撑
∧ 搬运距离达标
∧ 在目标区落桌
∧ 物体稳定直立
∧ 手完全退出
∧ 无力、穿透和环境碰撞违例
```

## 对关键问题的处理

### 1. ProbeResult 不能单独决定动作

接口显式增加：

```python
ShortCanPickPlaceRequest(
    target=...,
    place_offset_xy_m=(0.0, 0.12),
    reset_before_execute=True,
)
```

`ProbeResult` 提供质量相关估计和 probe quality；request 提供 skill intent、target 和
固定目标位移。第一版只接受 `(0, 0.12)`，避免接口表面支持任意目标、实际却没有验证。

### 2. Probe 后状态交接

当前唯一允许的 handoff 是：

```text
reset_to_canonical_checkpoint
```

原因是现有 `heft` 结束时会 retreat、release，物体可能发生落下和微小位姿变化；如果
直接把 post-probe live state 当成固定模板起点，执行不再可重复。

因此：

- `reset_before_execute=False` 会得到 `canonical_reset_required`，不生成 plan。
- reset 只恢复仿真状态；plan 仍然只消费公开 `ProbeResult` 和 visible object spec，
  不读取 hidden mass。
- 这是一条 Sim v1 组件验证协议，不能表述为连续真机 episode。

### 3. Allegro 不能继续只有 grip_alpha

scene 新增：

```python
scene.command_allegro_joints(q_target_16)
scene.allegro_joint_targets()
scene.allegro_joint_positions()
```

`short_can_side_wrap_v1` 模板包含：

```text
q_open[16]
q_preshape[16]
q_contact[16]
q_squeeze_limit[16]
active_fingers = [mf, th]
required_contact_groups = [mf, th]
required_object_geom = obj*_geom  # waist
```

当前 q 数值仍由现有 cylinder synergy 派生，但它们已经成为显式的对象模板资产；执行器
下发 16-D joint target，而不是把所有对象硬编码成一个全局 grip 标量。后续可以替换
单个模板而不改变 plan/executor 协议。

### 4. “有接触”不等于合法抓取

contact snapshot 新增：

- `hand_force_by_group_N`
- `hand_object_geoms`
- `hand_max_penetration_m`
- `hand_table_contact / hand_table_normal_force_N`
- `hand_support_contact / hand_support_normal_force_N`

抓取建立必须满足：

```text
mf contact
∧ thumb contact
∧ 至少一个接触发生在 short_can waist geom
∧ aggregate hand normal force 达到 plan target
∧ hand penetration 未超限
∧ 手没有碰源 pedestal
```

lift/carry 期间允许接触从 waist 向 top lip 重分配，但必须持续保持 mf+thumb 对向接触；
源 pedestal、table 或手—环境碰撞都会触发失败。

### 5. target_normal_force 的语义

第一版不再使用含糊的 `target_normal_force_N`，而明确为：

```text
target_total_normal_force_N
= 所有合法 hand-object contact normal-force magnitude 的总和
```

这与 `ContactSnapshot.hand_normal_force_N` 的实际计算语义一致。它不是单指力、单侧夹紧
力，也不是真实 Allegro torque command。

### 6. Probe estimate 如何改变 manipulation

从 `ProbeResult` 读取：

```text
m_est_kg
weight_signal_N
```

腕部速度：

```python
max_wrist_speed_mps = clamp(
    0.075 / (1.0 + 0.65 * m_est_kg),
    0.040,
    0.070,
)
```

放置模式以 `weight_signal_N = 1.6 N` 为当前标定分界。

轻罐：

```python
target_total_normal_force_N = clamp(
    6.5 + 0.75 * weight_signal_N,
    6.5,
    8.0,
)
use_gravity_settle = True
post_descent_xy_correction = False
hand_table_release_guard_N = 20.0
```

普通/重罐：

```python
target_total_normal_force_N = clamp(
    8.0 + 0.45 * weight_signal_N,
    8.0,
    11.0,
)
use_gravity_settle = False
post_descent_xy_correction = True
hand_table_release_guard_N = 30.0
```

这个分支不是物体 hidden mass 的判断；它只由实际 heft signal 产生。

为什么需要分支：

- 对轻罐使用统一的高预紧和近桌面纠偏，会储存较大的横向接触能量，开指后容易翻倒。
- 对普通/重罐统一使用低刚度 gravity settle，会因惯量和接触重分配导致倾倒。
- 因此 probe 信息确实改变 target force、速度和 place controller，而不是只被写进日志。

### 7. 手指先碰桌面，物体还没到底

实跑发现 short can 下放时，Allegro 有效指节会先接触桌面。如果继续压低 wrist，会出现
很高的手—桌面力；如果立即完全张手，物体又可能横向弹出或翻倒。

最终方案分两种：

- 普通/重罐：手—桌面力达到 30 N guard 或物体底面进入 10 mm 近表面区后停止下降，
  然后执行低刚度对称释放。
- 轻罐：20 N guard 或 25 mm 近表面区停止下降；固定 wrist，把 Allegro kp 降到
  `0.35`，对称减小闭合，让物体在指间笼约束下靠重力下滑；桌面接触连续成立后再
  完全释放。

统一硬上限：

```text
max_hand_table_force_N = 40 N
```

最终 verify 还要求 `final_hand_table_contact == 0`，因此 guard 接触不能残留到成功状态。

### 8. Release 不能简单反放闭合轨迹

直接反向播放 cylinder close synergy 会在部分姿态上先增加夹持力。当前 release 使用：

```text
Allegro kp: 8.0 → 0.1
16 joints symmetric opening
stop when:
  no hand-object contact
  OR object already table-supported,
     opposing grasp gone,
     residual contact force <= 1.5 N
retreat wrist
restore kp = 8.0
```

无论正常完成还是中途失败，`_result()` 都恢复 Allegro kp，避免下一 episode 继承低刚度。

## 完整状态机

| phase | 主要命令 | 正常结束 | 关键失败 |
|---|---|---|---|
| `handoff` | canonical reset | scene settle | 非 canonical policy 不生成 plan |
| `preshape` | 16-D `q_preshape` | hand target reached | actuator/interface error |
| `approach` | fixed wrist waypoint | grasp pose reached | penetration |
| `contact_acquire` | 16-D incremental close | waist + mf/th + target force | no legal grasp、hand-support collision |
| `grip_regulate` | hold joint target | stable contact window | penetration、unstable pregrasp |
| `lift` | wrist z +35 mm command | object actual lift ≥20 mm | source/table contact、lost grasp |
| `carry` | fixed +Y motion + object-space correction | actual carry ≥80 mm | lost grasp、environment collision |
| `place_descent` | guarded wrist descent | clearance/hand-table guard | force、penetration、support collision |
| `settle_to_surface` | 轻罐专用低 kp symmetric open | table contact stable | dropped early、timeout |
| `release` | kp=0.1 symmetric opening | grasp gone / residual light touch | force、penetration、release timeout |
| `retreat` | wrist +Z | hand leaves object/table | collision |
| `final_verify` | hold 0.5 s | all success criteria true | place/tilt/drift/support/release failure |

## 最终验收条件

计划固定的主要阈值：

| 指标 | 阈值 |
|---|---:|
| actual lift distance | `>= 0.020 m` |
| actual carry distance | `>= 0.080 m` |
| place XY error | `<= 0.035 m` |
| final tilt | `<= 0.20 rad` |
| final drift over 0.5 s | `<= 0.005 m` |
| grasp/carry hand penetration | `<= 0.0052 m` |
| place/release hand penetration | `<= 0.0055 m` |
| grasp/carry aggregate hand force | `<= 20 N` |
| place/release aggregate hand force | `<= 30 N` |
| hand-table force | `<= 40 N` |
| final object-table contact | required |
| final hand-object contact | forbidden |
| final hand-table contact | forbidden |

`ManipulationExecutionResult` 单独记录 manipulation status、phase、violations、quality 和
trace；它不会修改或覆盖原始 ProbeResult。

## 当前验证结果

验证命令：

```bash
conda run -n probebench python -m pytest tests/test_manipulation.py -q
```

覆盖：

- 3 个 scene seed：`0 / 1 / 2`
- 每个 scene 的 3 个 target
- 共 9 个真实 `heft → plan → pick/place` MuJoCo 闭环
- invalid probe、非 canonical handoff、错误目标位移、reference backend 拒绝
- 16-D command shape 和 gain schedule 恢复

当前 9 个闭环全部成功。实测范围：

| 指标 | min | max |
|---|---:|---:|
| lift distance | `0.023895 m` | `0.031470 m` |
| carry distance | `0.116785 m` | `0.121072 m` |
| place error | `0.016140 m` | `0.028828 m` |
| final tilt | `1.25e-5 rad` | `1.74e-5 rad` |
| grasp/carry penetration | `0.004261 m` | `0.004743 m` |
| peak hand-table force | `7.40 N` | `30.12 N` |

这些是当前解析几何、固定位置和当前 MuJoCo contact 参数下的回归结果，不是跨对象或
真机性能结论。

## 仍然存在的问题

1. **模板仍是手工模板**：虽然执行接口已是 16-D，但 q 值仍从单一 cylinder synergy
   派生，还没有独立的 can grasp optimization 或 finger IK。
2. **carry 中仍可能使用 top lip 的 form assistance**：acquisition 必须先有 waist contact，
   但 lift 后接触可重分配到 top lip。当前结果不能证明纯摩擦抓取能力。
3. **穿透上限仍偏大**：回归最大约 4.74 mm，适合作为当前 simulation gate，但离真机
   接触可信度仍有距离。
4. **质量到控制的公式是本场景标定规则**：`1.6 N` 分界和两段公式不能直接迁移到新
   尺寸、新摩擦或真实 Allegro。
5. **使用 oracle object pose**：目标定位误差还没有进入测试；当前验证的是执行闭环，
   不是视觉定位鲁棒性。
6. **handoff 使用 reset**：保证了可重复性，但没有验证连续 probe→manipulation 状态
   继承。
7. **没有机械臂约束**：6-DoF carriage 能到达的 waypoint 不代表真实 arm-hand 可达。
8. **目标选择仍在仓库外**：这里执行明确 target，不负责比较多个 ProbeResult 后选择
   最重罐。

因此当前能力应准确描述为：

> 规范化 MuJoCo 场景中，可信 Allegro heft 结果条件化的 short_can 固定模板
> pick/place 闭环执行。

不能描述为通用灵巧手抓取、机械臂 manipulation 或完整 ProbeBench solver。

## 运行示例

```bash
conda activate probebench
python -m examples.run_short_can_pick_place \
  --seed 0 \
  --target 2 \
  --viewer
```

输出依次包含：

```text
SCENE
PROBE_RESULT
PLAN_DECISION
MANIPULATION_RESULT
```
