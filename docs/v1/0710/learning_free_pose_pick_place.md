# 无学习物体位姿抓取与固定位置放置接口

## 1. 这条路线解决什么

本接口把 manipulation 阶段收窄为一个已经在 MuJoCo 中闭环执行的任务：

```text
有效 Allegro heft ProbeResult
+ 当前 short_can 的世界系中心位姿 T_world_object
+ controller 持有的固定绝对放置目标
→ 在线几何计算 wrist 目标位姿
→ Allegro 16-DoF 手型与接触闭环
→ 抓起、搬运、放置、松手和最终验收
```

目标位置是世界系绝对位置，不再是“相对源位置平移 0.12 m”。计算过程不使用
learning、VLA、抓取网络或离线轨迹库；它使用解析几何、圆柱对称性、执行器范围、
保守障碍净空和在线接触反馈。

当前能力边界是：`mass / short_can / Allegro / 平桌 / 直立物体 / 固定放置区`。
它不是任意物体的通用抓取器，也不包含机械臂 IK 或运动规划。

## 2. manipulation 的三个输入

### 2.1 `ProbeResult`

必须满足：

- `primitive == "heft"`；
- `backend == "allegro"`；
- `valid == True` 且无 violation；
- `object_id`、`target` 与 manipulation scene 一致；
- `scene_id` 与 manipulation scene 一致，防止把另一个场景中的 `obj0` 错接进来；
- 包含正的 `m_est_kg` 和 `weight_signal_N`。

v2 按真实 probe 输出标定的准入范围是：

```text
m_est_kg       ∈ [0.025, 0.70]
weight_signal_N ∈ [0.25, 6.60]
```

规划不读取 `ObjectSpec.mass_kg` 生成控制信号。测试中的真实闭环路径由
`ProbeHarness.execute(ProbeCommand("heft", target))` 产生输入。

### 2.2 `ObjectPoseObservation`

位姿约定为：

```text
T_parent_child: 把 child 坐标表达转换到 parent 坐标表达
T_world_object: 物体几何中心坐标系到世界坐标系
quaternion:     wxyz
```

这里必须传物体几何中心，不是当前 `object_pos()` 的顶部 site。scene 新接口
`object_center_pos()` 会按物体姿态扣除旋转后的半高偏移。

short can 允许绕自身局部 z 轴连续对称，因此观测 yaw 不影响抓取语义；局部 z 轴
方向和位置会被使用。当前只接收接近直立、底部位于桌面的 can。

### 2.3 `FixedPlaceSpec`

固定目标由 controller 构造并持有，包括：

- `goal_id`；
- 世界系物体中心目标 `T_world_object_goal`；
- 支撑面高度；
- 最大三维位置误差，当前硬上限 `35 mm`；
- 最大物体轴倾角，当前硬上限 `0.20 rad`；
- short can 的 yaw 自由语义。

执行前 controller 会复验 plan 内的 `goal_id`、目标变换、支撑面和容差，另一个
controller 生成的 plan 不能被拿来绕过当前固定目标。

## 3. 两种 scene 配置为什么不同

probe 和 manipulation 使用相同 `ProbeSceneSpec` 与 `scene_id`，但使用两个 MuJoCo
scene 实例：

```python
probe_backend = AllegroHandBackend.create(spec)

manip_backend = AllegroHandBackend.create(
    spec,
    allegro_grasp_lift=0.0,
    full_hand_collisions=True,
    wrist_roll_limit_rad=np.pi,
)
```

当前 Allegro probe scene 和 manipulation scene 都让 short can 直接位于桌面，并编译
完整手部碰撞；heft 已经改为同类 top-entry 夹持，不再依赖窄托架。manipulation scene 在 XML
编译时启用 palm、base、proximal、medial、distal 和 fingertip 碰撞代理，不能在运行
后修改 `contype` 假装启用完整碰撞。

`SceneConfig` 已冻结；plan admission 和 execute 都会再次检查编译后 geom collision
mask。仅把 Python 字段改成 `full_hand_collisions=True` 不能绕过检查。

## 4. wrist 目标位姿如何实时产生

当前抓型是 `short_can_top_pinch_v1`。物体系中的标称 wrist 变换为：

```text
t_object_wrist = [0.000, -0.020, +0.130] m
R_object_wrist = Rx(pi)
```

也就是 hand 从物体上方进入，翻转手腕，让中指和拇指在 can 顶部凸缘形成夹持；
不再让掌心 mesh 穿过物体。

对圆柱局部 z 轴采样 12 个对称 yaw：

```text
T_world_wrist(theta)
  = T_world_object
  · Rz(theta)
  · T_object_wrist
```

每个候选进一步产生：

```text
staging → pregrasp → grasp → lift → carry
```

候选过滤内容包括：

- 六个 carriage actuator 的真实编译后 ctrl range；
- wrist 必须可达到 `Rx(pi)`；
- 源点与固定目标都在桌面有效边界内；
- 源点对其他候选物体保留更大的 approach 净空；
- 固定目标对其他物体保留 release 净空；
- 物体直立、底面与桌面一致、目标 z 与物体半高一致。

剩余候选按当前 wrist 到 pregrasp 的平移距离、对称 yaw 转动量和 actuator range
余量排序，选择最低分项。该计算只是少量 4×4 矩阵运算和范围检查，可在每次收到
新物体位姿时重新执行，不需要 learning 推理。

这仍不是通用碰撞运动规划。候选 admission 使用保守局部净空；分段直线路径上的
实际碰撞由执行期逐步 gate 捕获。遇到未建模障碍时返回失败，不会绕开障碍重规划。

## 5. 6-DoF wrist 信号如何变成 carriage 控制

规划使用物理 wrist frame 的世界位姿。执行器使用的 z 是 carriage 基座平移，因此：

```text
x_cmd = wrist_world_x
y_cmd = wrist_world_y
z_cmd = wrist_world_z - palm_height

R_world_wrist = Rx(roll) · Ry(tilt) · Rz(yaw)
```

`staging` 先只平移到高位，再单独旋转到 top-entry 姿态，避免位置和 180° 翻转同时
插值时长手指扫过附近物体。之后再沿分段直线下降到 pregrasp 和 grasp。

## 6. 16-DoF hand 信号如何产生

模板包含四个 16 维 Allegro actuator target：

```text
q_open          = allegro_grip_pose(0.00)
q_preshape      = allegro_grip_pose(0.10)
q_contact       = allegro_grip_pose(0.80)
q_squeeze_limit = allegro_grip_pose(0.98)
```

闭合进度 `p∈[0,1]` 使用 `q_contact` 作为真实 waypoint：

```text
p <= 0.8: q_preshape → q_contact
p >  0.8: q_contact  → q_squeeze_limit
```

执行不会无条件走到 `q_squeeze_limit`。当合法双指接触总法向力达到目标时停止继续
闭合；搬运阶段再按接触力小幅开合。目标法向力来自 heft：

```text
F_target = clip(7.0 + 0.82 × weight_signal_N, 8.1, 10.7) N
```

极轻 can 使用更窄、更缓的 force-regulation deadband，避免“微滑—快速闭合—力尖峰”
振荡。`q_squeeze_limit` 只是 regulator 的有界余量。

## 7. 完整执行状态机

```text
handoff
→ guarded preshape
→ staging translation
→ staging wrist rotation
→ pregrasp
→ grasp pose
→ contact_acquire
→ grip_regulate
→ lift 130 mm
→ high carry to fixed absolute goal
→ object-space XY correction
→ guarded fast descent
→ strict low-kp release
→ retreat
→ final_verify
```

carry 速度同样由 heft 信号有界条件化：

```text
v_wrist = clip(0.070 + 0.008 × weight_signal_N, 0.080, 0.106) m/s
```

较高 lift 避免携带物与留在桌面的其他候选相撞；有界较快的横移和下降减少重罐在
top pinch 中持续微滑的时间。到达目标上方后仍用实际物体中心 XY 对 wrist 做最多
两次小修正，不是单纯播放原始源位置对应的固定轨迹。

## 8. 每阶段的碰撞和接触 gate

成功抓持必须同时满足：

- `mf` 和 `th` 都有接触；
- 每个 required group 的法向力至少 `0.20 N`；
- 接触包含目标 can 的 `_top_lip`；
- 不允许 `ff/rf` 等非 active finger 提供隐藏支撑；
- 只允许 fingertip、thumbtip 和 distal link；
- palm/base/proximal 接触失败；
- 最大手—物 penetration 不超过 `6.8 mm`；
- 抓取/搬运总法向力不超过 `20 N`。

执行期还会立即拒绝：

- hand 接触 table；
- hand 接触任何非目标物体；
- 目标物体碰到其他候选物体；
- source/goal support fixture；
- 搬运中物体重新碰桌；
- 对向接触持续丢失；
- release 前未完全清除所有 hand-object contact。

成功结果中的 `peak_palm_object_force_N`、`peak_hand_other_object_force_N` 应为 0。

## 9. handoff 的两个语义

### 9.1 `verify_live_pose`：manipulation 正式接口

这是 `PoseConditionedPickPlaceRequest` 的默认值。executor 不 reset scene，而是比较
scene 当前物体中心/轴与请求位姿：

```text
position error <= 8 mm
axis error     <= 0.12 rad
```

不匹配时在 handoff 阶段失败。preshape 本身也有碰撞 guard，因此未知的 wrist/hand
起始状态不能先无保护闭手再进入 approach。

### 9.2 `reset_to_requested_pose`：仅仿真测试 fixture

该模式执行 `scene.reset()` 后把自由物体设置到请求位姿。它是复现实验的 canonical
fixture，本质上包含仿真“传送”，不能被描述为真实机器人已经定位并移动到了源点。

## 10. 固定目标如何验收

最终验收不是只看 XY。v2 使用物体几何中心对固定目标计算三维误差，并将物体局部
z 轴与目标轴比较：

```text
||p_final - p_goal||_2 <= max_position_error_m <= 0.035 m
axis_error             <= max_tilt_rad <= 0.20 rad
final drift            <= 0.005 m
```

此外必须满足：物体与桌面稳定接触、hand-object 接触为零、hand-table 接触为零、
完整碰撞/力/penetration 历史均未超限。short can 的最终 yaw 不验收，因为接口明确
声明绕自身轴连续对称。

## 11. manipulation 调用接口

```python
import numpy as np

from allegro_probe import (
    AllegroHandBackend,
    FixedPlaceSpec,
    ObjectPoseObservation,
    PoseConditionedPickPlaceRequest,
    PoseConditionedShortCanController,
    RigidTransform,
)

scene = AllegroHandBackend.create(
    spec,
    allegro_grasp_lift=0.0,
    full_hand_collisions=True,
    wrist_roll_limit_rad=np.pi,
).scene

observation = ObjectPoseObservation(
    target=target,
    object_id=object_id,
    T_world_object=T_world_object_center,
    confidence=1.0,
)

fixed_goal = FixedPlaceSpec(
    goal_id="short_can_drop_zone_v1",
    T_world_object_goal=T_world_object_goal,
    surface_z_m=0.0,
)

controller = PoseConditionedShortCanController(scene, fixed_goal)
request = PoseConditionedPickPlaceRequest(
    object_pose=observation,
    fixed_goal_id=fixed_goal.goal_id,
    handoff_policy="verify_live_pose",
)

decision = controller.plan(probe_result, request)
if not decision.executable:
    raise RuntimeError(decision.reason)
result = controller.execute(decision.plan)
```

`controller.execute()` 会重新验证 compiled collision、目标归属、模板、hand pose、
force/penetration/speed/tolerance 上限和完整 wrist 轨迹。调用方不能通过修改 dataclass
plan 放宽安全参数。

## 12. 可直接运行的示例

```bash
conda activate probebench
python -m examples.run_pose_conditioned_pick_place \
  --seed 0 \
  --target 2 \
  --source-x 0.11 \
  --source-y -0.09 \
  --place-x 0.0 \
  --place-y 0.12
```

默认示例使用 `reset_to_requested_pose` 便于复现。加 `--verify-live-pose` 后，示例会先
准备 live scene，再通过正式 handoff 分支执行。加 `--viewer` 可观察完整碰撞过程。

## 13. 当前明确不支持的内容

- 非 `short_can` 形状；
- 倾倒、倒置或悬空物体；
- 非平桌支撑或运行时移动托架；
- 对 short can yaw 的精确放置约束；
- 任意障碍物的绕行规划；
- Franka 等机械臂 IK、关节限位、自碰撞和轨迹规划；
- learning/VLA 产生 wrist pose 或 hand action；
- 超出已校准 heft signal、力、速度和 penetration 范围的外推。

因此这里“实时计算目标位姿”的准确含义是：在已知解析几何和已验证抓型范围内，
每次根据新的 `T_world_object` 在线重算分段 6-DoF wrist 目标；不是在线学习一个新
抓法，也不是对任意 mesh 做通用 grasp synthesis。
