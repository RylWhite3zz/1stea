# AllegroProbe 接入 Franka 机械臂时的问题与改造边界

## 1. 文档目的

本文讨论把当前 AllegroProbe 中的理想 6-DoF wrist carriage 替换成
`Franka + Allegro Hand` 时会出现的问题。这里的“接入”至少包含两种不同目标：

1. 在 MuJoCo 中把 Franka Panda/FR3 与 Allegro Hand 组装成完整机器人；
2. 让现有 probe/manipulation 计划通过真实的 7-DoF 机械臂运动学、动力学、碰撞和
   接触控制执行，并最终可以映射到实机。

第一项主要是 MJCF 装配问题，工作量相对有限。第二项会改变当前执行后端的核心假设，
不能通过“把 carriage XML 换成 Panda XML”完成。

本文只给出问题、接口边界和推荐改造顺序，不声称当前仓库已经具有 Franka IK、运动
规划或实机控制能力。

---

## 2. 当前系统的真实前提

当前 `AllegroProbeScene` 在手掌上游创建了六个彼此独立的关节：

```text
wx / wy / wz / wr / wt / wyaw
```

它们由六个 position actuator 直接驱动。调用：

```python
scene.command(x=..., y=..., z=..., roll=..., tilt=..., yaw=...)
```

相当于直接指定 wrist 的任务空间目标。当前 `_move_wrist()` 只需在起点和终点之间做
smoothstep 插值，不需要回答以下问题：

- 该 wrist pose 是否存在机械臂逆解；
- 选择哪一个肘部姿态；
- 关节是否接近限位或奇异位形；
- 从当前状态到目标状态的连杆是否撞桌、撞手或撞其他物体；
- 轨迹速度、加速度和 jerk 是否能由真实机械臂执行；
- 接触后应继续位置跟踪，还是切换阻抗/力控。

所以当前 carriage 不是“简化版 Franka”，而是一个具有理想任务空间可达性的独立执行
模型。它适合验证 wrist waypoint、手型模板和接触 gate，但不能证明机械臂可执行性。

---

## 3. 哪些现有结果可以继续复用

接入 Franka 不需要推翻整个数据流。下列结构仍可保留：

```text
ProbeResult
→ ManipulationContext / request
→ ManipulationPlan
→ phase goals + hand template + safety limits
→ device-specific execution backend
→ ManipulationExecutionResult
```

可以继续复用的内容包括：

- `ProbeResult` 的物理估计、有效性、violations、sensor profile 和 feature schema；
- `ObjectPoseObservation(T_world_object)` 的对象位姿语义；
- `FixedPlaceSpec(T_world_object_goal)` 的绝对放置目标；
- `staging/pregrasp/grasp/lift/carry` 等物理 wrist waypoint；
- short-can 的 16-DoF Allegro `q_open/q_preshape/q_contact/q_squeeze_limit`；
- 接触分组、force、penetration、支撑重接触、掉落和放置验收 gate；
- `ManipulationExecutionResult` 对执行失败的独立记录。

需要替换的是：

```text
_move_wrist()
+ scene.command(x/y/z/roll/tilt/yaw)
```

也就是从 wrist waypoint 到真实设备控制信号之间的编译与执行层，而不是改变
`ProbeResult` 的含义。

---

## 4. MJCF 机械装配问题

### 4.1 本地已有模型，但不能直接 include 后结束

本地 Menagerie 已包含：

```text
/home/enovo/robots/sim/mujoco_menagerie/franka_emika_panda/panda_nohand.xml
/home/enovo/robots/sim/mujoco_menagerie/wonik_allegro/right_hand.xml
```

`panda_nohand.xml` 在末端提供 `attachment_site`，适合作为外部手的安装入口；Allegro
模型以 `palm` 为根 body。不过仍需显式定义：

```text
T_franka_attachment_allegro_palm
```

该变换不是随意把两个原点重合。它决定：

- Allegro 掌面朝向；
- 四指相对 Franka flange 的方向；
- 当前 top-entry `Rx(pi)` 模板在机械臂语义下是否仍然向下；
- wrist F/T frame 与 palm frame 的关系；
- 所有现有 `T_object_wrist` 抓取模板是否需要整体重标定。

### 4.2 必须建模安装转接件

实机通常需要 flange-to-Allegro adapter。仿真中至少要加入：

- adapter 的固定变换；
- 质量、质心和惯量；
- 碰撞几何；
- 与线缆、手背和 Franka link7 的合理净空。

如果仿真忽略 adapter 质量，wrist wrench baseline 和机械臂动力学会与实机不一致；如果
忽略 adapter collision，则规划器可能生成实机无法通过的腕部姿态。

### 4.3 MJCF 名称和 default class 冲突

Panda 与 Allegro 都带有自己的：

- `<default>` 层级；
- mesh/material asset；
- actuator；
- compiler/option 设置；
- collision exclude。

当前 `_allegro_sections()` 是为“把 Allegro palm 放进 carriage”写的，不能原样当成
通用模型合并器。接入 Franka 时要检查：

- asset 路径是否仍相对各自模型目录有效；
- joint、actuator、site 和 geom 名是否唯一；
- `cone="elliptic"`、`impratio`、timestep、solver 参数由谁统一；
- Panda 原有 link exclude 在装手后是否仍正确；
- Allegro 的视觉 mesh 与解析碰撞 proxy 是否都被保留。

### 4.4 不应保留 carriage 与 Franka 双重自由度

正式 Franka backend 中，Allegro palm 应固定在机械臂末端，不能再同时保留
`wx/wy/wz/wr/wt/wyaw`。否则会形成：

```text
Franka 7 DoF + 理想 wrist 6 DoF + Allegro 16 DoF
```

机械臂不可达性会再次被隐藏。carriage backend 应保留为独立的 reference/regression
backend，而不是串在 Franka 后面。

### 4.5 当前中央 probe 不是 Franka+Allegro 的天然硬件

当前 carriage 在同一个 wrist 下同时挂载了 Allegro palm 和 `probe_mount`。现有 Allegro
`poke` 已改为食指指尖，但 `slide` 仍使用中央仪器化 probe。真实的 Franka+Allegro
组合并不会自动多出这根探针，必须在三种方案中明确选择：

1. 设计固定在 flange/adapter 上的侧置 probe，并加入质量、F/T 和完整碰撞模型；
2. 使用 tool changer，在 Allegro 与 probe tool 之间换工具；
3. 把 slide 重新实现为 Allegro fingertip slide，并建立新的 sensor profile、接触几何、
   force calibration 和 protocol version。

不能在仿真里继续保留一根不占空间的中央 probe，然后把结果描述成裸
`Franka + Allegro` 可以执行。若固定侧置 probe，它还会改变 IK、arm/hand collision
envelope、payload 和 wrist wrench baseline。

---

## 5. 坐标系问题

### 5.1 当前 `wrist` frame 必须重新冻结

当前代码中的 `T_world_wrist` 对应 carriage 内部的 `wrist_pose_site`。位置控制还包含：

```python
carriage_z = physical_wrist_z - scene.config.palm_height
```

接入 Franka 后，不应继续使用这个换算。至少要区分：

```text
world
franka_base
link7 / flange
attachment_site
allegro_palm
protocol_wrist
object
```

推荐保留一个与现有计划兼容的抽象 `protocol_wrist`，然后显式标定：

```text
T_franka_base_protocol_wrist(q)
  = FK_franka(q)
  × T_flange_attachment
  × T_attachment_protocol_wrist
```

规划器消费的仍是 `T_world_protocol_wrist`，机械臂后端负责转换到 flange/link frame。

### 5.2 `world` 与 Franka base 不一定重合

当前对象、桌面和放置目标都在 MuJoCo `world` 中。机械臂接入后要显式给出：

```text
T_world_franka_base
```

实机还需要相机、桌面、机械臂基座之间的外参。若把 base 与 world 暗中视为同一坐标系，
仿真中可行的固定目标换到实机后会产生系统性偏差。

### 5.3 右手安装方向会改变 RPY 分解

当前 carriage 用 XYZ roll/tilt/yaw 三个串联 hinge。Franka IK 通常直接消费旋转矩阵或
四元数。不能把现有 RPY actuator 数值直接当作机械臂末端 RPY；正确输入应是完整的
`RigidTransform.rotation`。否则在接近 ±π 或不同 Euler 分支时会产生不连续轨迹。

### 5.4 圆柱 yaw 自由度应变成 IK 冗余资源

short can 对 local-z 轴具有连续 yaw 对称性。当前代码通过多个
`symmetry_yaw_samples_rad` 选 wrist candidate。接入 7-DoF Franka 后，这个自由度很有
价值，可用于：

- 远离关节限位；
- 选择更好的肘部姿态；
- 增加桌面和邻物净空；
- 降低奇异性；
- 缩短关节路径。

但它只能增加候选，不等于已经解决 IK 或避碰。

---

## 6. IK 与可达性问题

### 6.1 一个 wrist pose 可能有零个或多个逆解

carriage 能直接到达范围内的任意 6-D pose；Franka 的目标 pose 可能：

- 完全不可达；
- 只能以接近关节限位的姿态到达；
- 存在多个 elbow configuration；
- 单点可达，但 approach 路径不可达；
- grasp 可达，但 lift/carry/place 不可同时连续可达。

所以 plan admission 不能只检查每个 waypoint 的 XYZ/RPY 范围，而应检查整个 waypoint
序列是否存在连续的关节空间解。

### 6.2 7-DoF 冗余需要明确选择准则

对于同一个末端 pose，Franka 的第七自由度允许不同肘部姿态。至少需要综合：

```text
joint-limit margin
manipulability / singularity margin
self-collision margin
environment clearance
distance from current q
trajectory continuity
future waypoint feasibility
```

只对每个 waypoint 独立调用 IK，容易在相邻 phase 之间跳到不同逆解分支，造成不连续或
大幅甩动。

### 6.3 当前 `carriage_margin` 需要替换

`GraspCandidate.carriage_margin` 只描述理想 carriage actuator 离 ctrl range 的距离。
Franka backend 至少需要输出：

```text
ik_solution_count
minimum_joint_limit_margin
minimum_manipulability
minimum_self_collision_distance
minimum_environment_clearance
joint_path_length
planning_status
```

这些量应进入 candidate 排序和 plan admission，不能继续把 carriage margin 当作机械臂
可达性证据。

---

## 7. 运动规划问题

### 7.1 Cartesian waypoint 插值不等于无碰撞轨迹

当前 `_move_wrist()` 只在 wrist target 之间平滑插值。换成 Franka 后，即使末端直线看似
安全，肘部或前臂也可能扫到：

- 桌面；
- 邻近候选物；
- 物体托架；
- 机械臂自身；
- Allegro 手背或手指；
- 被抓取物。

需要 joint-space collision checking，以及必要的 path search，而不只是 endpoint IK。

### 7.2 每个 phase 的规划语义不同

建议区分：

| phase | 主要规划问题 |
|---|---|
| staging | 从当前 q 到安全高位，完整 arm/scene 避碰 |
| pregrasp | 受约束接近，保持手掌朝向和对象净空 |
| grasp | 短距离 Cartesian/速度受限接近，准备进入接触控制 |
| lift | 保持抓取与对象姿态，避免对象扫桌 |
| carry | 机械臂、手和 attached object 一起避碰 |
| place | 受约束下降，切换接触/力控 |
| retreat | 确认松手后再安全退离 |

自由空间 phase 可以用采样式或优化式规划；接触附近通常要用短程 Cartesian 约束轨迹和
阻抗控制，不能把所有 phase 交给同一种 planner。

### 7.3 抓住物体后要更新 planning scene

通过 grasp gate 后，规划器必须把对象视为 attached collision object。否则 carry 时只
检查机械臂和手，可能让罐子本身撞桌或撞邻物。release 被确认后才能 detach，并恢复为
环境物体。

### 7.4 需要时间参数化

Franka 轨迹至少要满足关节速度、加速度、jerk 和力矩约束。当前由 `min_steps` 产生的
仿真时长不能直接转换为实机轨迹时长。`max_wrist_speed_mps` 也不足以约束全部七个关节。

---

## 8. 控制问题

### 8.1 理想 position carriage 会掩盖机械臂动力学

当前 carriage 使用较高位置增益，可以在拿着物体时强制跟踪任务空间目标。Franka 中会
出现：

- 重力、科氏力和惯性耦合；
- 末端载荷变化；
- 关节柔顺和跟踪误差；
- 碰撞 reflex；
- 位置增益过高导致接触冲击；
- 增益过低导致 probe/抓取姿态漂移。

因此“MuJoCo 中 wrist 到了”不能直接等价为“Franka 控制器能稳定到达”。

### 8.2 自由空间与接触 phase 应使用不同控制模式

推荐至少分成：

```text
free-space:
  joint trajectory tracking 或 Cartesian trajectory tracking

guarded approach / poke / slide / place:
  Cartesian impedance、admittance 或显式 force guard

heft / shake / carry:
  稳定 pose tracking + payload compensation + grasp/contact supervision
```

如果全过程都使用刚性位置控制，poke/slide/place 很容易超力；如果全过程都使用很软的
阻抗，heft/shake 的动态信号又可能被机械臂柔顺性污染。

### 8.3 arm 与 hand 控制必须同步

现有执行器能在同一仿真循环里同时修改 wrist 和 16 个 Allegro actuator。真实系统需要
明确：

- arm command rate；
- hand command rate；
- 两个控制器的时钟和 timestamp；
- 哪个控制器拥有 phase transition；
- arm stop 后 hand 是否立即停止；
- hand force gate 触发后如何中止正在执行的 arm trajectory。

不能让 MoveIt 轨迹在后台继续运行，而手部控制器已经因超力判定失败。

### 8.4 停止语义必须下沉到机械臂控制器

当前 `run.step()` 每个 MuJoCo step 都能审计碰撞并停止后续命令。实机中，如果安全 gate
只在 Python 高层以低频轮询，机械臂可能在发现超力后继续运动数十毫秒。需要把硬限制、
reflex 或 trajectory cancel 放到足够低延迟的控制层。

---

## 9. wrist F/T 与 probe 信号语义会变化

### 9.1 当前 wrist wrench 是理想仿真 site 读数

当前 `wrist_force/wrist_torque` 位于 carriage 内的 `wrist_ft_site`。接入 Franka 后必须
决定信号来自：

- 安装在 flange 与 Allegro adapter 之间的真实 6-axis F/T；
- Franka 关节力矩估计出的 external wrench；
- MuJoCo 中在 attachment 处新增的 force/torque sensor；
- 上述信号的融合。

这些信号的噪声、带宽、bias、坐标系和动态响应都不同，不能继续共用当前 calibration。

### 9.2 工具重力和 payload 补偿不可省略

Allegro 手、adapter、线缆和被抓物都会影响 wrist wrench。baseline 至少依赖：

```text
arm q / wrist orientation
tool mass and CoM
adapter inertia
hand joint pose
object mass estimate
sensor bias and temperature
```

当前 heft 使用“支撑中 baseline → 脱离支撑后静态力差”。该差分仍有价值，但机械臂姿态
变化、加速度和重力补偿误差可能混进重量信号。

### 9.3 `shake` 尤其容易被机械臂本体污染

当前 shake 的主输入是实际 wrist tilt，输出是 baseline-corrected wrist torque。接入
Franka 后，3 Hz 动态响应还会包含：

- 七个 arm joint 的跟踪与柔顺性；
- link/adapter/hand 的惯性；
- 控制器相位延迟；
- 姿态相关的 Jacobian 和 torque estimation 误差；
- 机械臂结构振动。

因此 `dynamic_torque_gain_Nm_per_rad` 的 calibration key 至少还要加入：

```text
arm model/profile
arm controller profile
mount/tool profile
wrench source
arm posture or posture bin
```

不能直接复用 carriage/allegro backend 的动态标定。

---

## 10. 四个 probe 分别会遇到什么问题

### 10.1 `poke`

Allegro poke 已使用食指 `ff_tip_touch` 闭环，但 Franka 接入后还需解决：

- 从任意 arm q 到 fingertip pre-contact pose 的 IK 和避碰；
- 手指接触点与 arm wrist frame 的精确外参；
- 机械臂刚度对法向力—压入曲线的影响；
- guarded descent 的低延迟停止；
- 法向方向不再必然等于 world z；
- 目标物轻微移动后，是否重规划 wrist pose。

若 arm compliance 没有标定，测到的 compliance 会同时包含对象、手指、adapter 和机械臂，
不能仍解释成对象刚度。

### 10.2 `slide`

当前 Allegro slide 使用 wrist 上的中央 probe；首先必须解决 4.5 节的真实工具来源。
在此之后，slide 还需要同时保持法向 preload 和切向速度。机械臂后端必须：

- 将 surface normal/tangent 转成 Cartesian path constraint；
- 在切向运动时用 impedance/force loop 保持法向力；
- 处理 Jacobian 随姿态变化造成的方向耦合；
- 防止 arm link 或手背在往返路径中碰撞环境；
- 区分表面 stick-slip 与机械臂控制振动。

### 10.3 `heft`

heft 的核心仍是物体真实脱离支撑，但新增问题包括：

- arm lift trajectory 的实际末端速度与加速度；
- 加速阶段惯性力不能进入静态重量窗口；
- arm 姿态变化导致的 wrench gravity compensation；
- 机械臂跟踪误差造成底缘重新接触；
- carry 前是否存在连续、无碰撞的关节解；
- 抓持失败时如何在不掉落的情况下安全回放。

需要在 lift 后增加 arm/hand/object 都稳定的 dwell，再采重量信号。

### 10.4 `shake`

shake 是最难迁移的一项：

- ±3°、3 Hz 对真实 arm 的 joint velocity/acceleration/torque 是否可行要重新验证；
- wrist tilt sinusoid 必须编译成连续 7-D joint trajectory；
- 不能在相邻 sample 间跳换 IK branch；
- 动态窗口要记录实际 FK wrist angle，而不是命令角；
- arm/controller 传递函数会进入 torque gain/phase；
- 手—物相对旋转和真实底缘净空仍必须逐周期检查；
- 若 arm 不能安全执行 3 Hz，应定义新的 versioned Franka protocol，而不是静默降频后
  复用原 calibration。

---

## 11. 碰撞与安全问题

### 11.1 当前 collision audit 只覆盖手部执行层

当前代码已经审计：

- 手—目标；
- 手—桌面/托架；
- 手—其他候选物；
- 目标—其他物体；
- probe—环境。

接入 Franka 后还要增加：

- arm link—table；
- arm link—object/support；
- arm self-collision；
- arm—Allegro 非法碰撞；
- adapter/cable volume—environment；
- attached object—environment。

### 11.2 Allowed collision 必须按 phase 管理

接触任务不能简单要求“全机器人零碰撞”。需要区分：

```text
pregrasp:       机器人不得接触目标
grasp:          指定手指 link 可接触目标
lift/carry:     指定抓持接触持续，其他接触禁止
place:          目标可接触目标支撑面，手/arm 不可撞桌
release:        手—目标接触应逐步消失
```

这与当前 hand contact whitelist 一致，但需要扩展到完整 planning scene 与 arm link。

### 11.3 规划碰撞与运行时碰撞都需要

planner 的离散碰撞检查不能替代执行期 contact audit；执行期传感器也不能替代规划。
正确关系是：

```text
plan-time collision-free
+ runtime distance/contact/force guard
+ hardware reflex / emergency stop
```

---

## 12. manipulation 接口需要怎样变化

### 12.1 不应让 `ManipulationPlan` 直接保存 7-D 轨迹

`ManipulationPlan` 继续保存 skill 级目标和约束较合理。设备相关轨迹应由新的编译结果
承载，例如：

```python
@dataclass(frozen=True)
class ArmPlanResult:
    executable: bool
    reason: str
    robot_profile: str
    controller_profile: str
    q_start: tuple[float, ...]
    phase_trajectories: dict[str, "JointTrajectory"]
    ik_quality: dict[str, float]
    collision_quality: dict[str, float]
```

建议数据流为：

```text
ManipulationPlan
→ ArmPlanCompiler
→ waypoint constraint + IK candidate sets
→ collision-aware continuous joint path
→ time parameterization
→ ArmPlanResult
→ FrankaAllegroExecutionBackend
```

### 12.2 需要设备状态结构

执行前不能只读取 wrist pose，还应有：

```text
q / dq
estimated joint torque
FK wrist pose
Jacobian
controller mode
robot error/reflex state
hand q / dq / actuator state
wrench + timestamp
planning-scene revision
```

所有 pose 和传感信号都应带 frame、timestamp 和 profile provenance。

### 12.3 phase transition 应由统一 executor 管理

arm planner、Franka controller 和 Allegro hand controller 可以是不同模块，但应由一个
phase executor 统一处理：

- 发送/取消 arm trajectory；
- 发送 hand target；
- 同步等待条件；
- 读取 arm/hand/contact 状态；
- 触发 safety stop；
- 生成一个统一的 `ManipulationExecutionResult`。

---

## 13. 仿真与实机并不是同一个后端

建议至少区分：

```text
carriage_allegro_sim
franka_allegro_mujoco
franka_allegro_real
```

三者不能共用一个模糊的 `backend="allegro"` calibration key。它们的：

- dynamics；
- controller；
- sensor profile；
- mount；
- latency；
- collision model；
- safety threshold

都不同。

MuJoCo Franka backend 可以直接控制 Panda 模型的七个 actuator；实机后端通常还需要
ROS 2/Franka control/MoveIt 等桥接。但对当前仓库而言，最好保持内部协议独立于具体
ROS message：外部适配器负责把 `ArmPlanResult` 映射成 joint trajectory、Cartesian
impedance target 和 stop/cancel 信号。

---

## 14. 推荐的实现顺序

### 阶段 0：冻结 frame 和 profile

先定义并测试：

- `T_world_franka_base`；
- `T_attachment_allegro_palm`；
- `protocol_wrist` 的准确位置和方向；
- arm/mount/controller/wrench sensor profile ID；
- Panda 还是 FR3，不能混用。

验收：静态 FK 与 MuJoCo site pose 一致，frame round-trip 数值误差有明确上限。

### 阶段 1：建立独立 Franka+Allegro MuJoCo scene

只完成模型装配、7+16 actuator 索引、传感器和碰撞分类，不执行 probe。

验收：

- canonical q 下无初始自碰撞；
- link、adapter、palm 和全部手指碰撞体存在；
- arm/hand 分别可独立小幅动作；
- wrench frame 和 payload 参数可审计；
- 不存在隐藏的 6-DoF carriage。

### 阶段 2：只做 FK/IK 与 plan admission

将现有 wrist waypoint 编译成连续 IK 候选，但暂不执行接触任务。

验收：

- 每个 waypoint 输出所有可接受逆解及拒绝原因；
- 整条 phase 序列保持同一连续 IK branch；
- joint-limit 和 singularity margin 可测；
- 不可达 plan 在动作前拒绝。

### 阶段 3：加入完整 arm collision planning

先验证 staging/pregrasp/retreat 等自由空间 phase，再加入 attached-object carry。

验收：

- 连杆、自碰撞、手、adapter、桌面、邻物和 attached object 全覆盖；
- 离散轨迹之外还有连续段碰撞检查或足够密的保守检查；
- trajectory 满足速度、加速度和 jerk 上限。

### 阶段 4：迁移 `poke` 和 `slide`

这两项不需要抬起物体，适合作为 arm 接触控制的第一阶段。

验收：

- guarded approach 能在超力前停止；
- `poke` 刚度排序在多个 arm posture 下保持；
- `slide` preload 和往返路径完成率达标；
- arm compliance 不会被错误计入对象 feature，或已有独立校准。

### 阶段 5：迁移 `heft`

加入真实 lift、payload/gravity compensation、支撑脱离和安全回放。

验收：

- 支撑脱离由真实 contact/geometry 确认；
- 静态测量窗口排除 arm 加速段；
- 质量排序跨多个 q/posture 成立；
- 失败时不会空中松手；
- carry 前后 arm/hand/object 均无非法碰撞。

### 阶段 6：最后迁移 `shake`

重新标定 Franka 专属频率、幅值和 dynamic feature，不默认继承 3 Hz。

验收：

- joint velocity/acceleration/torque 均在限值内；
- 实际 wrist input 的幅值、相位和 SNR 达标；
- arm-only/locked-content baseline 可重复；
- 内容物响应在相同 arm posture/profile 下可分；
- 真实物体底缘始终满足净空硬门；
- 动态窗口无额外未建模控制输入。

### 阶段 7：再接实机

MuJoCo 通过不代表实机可直接运行。实机前还需要：

- robot/hand homing；
- base、camera、table 和 mount 标定；
- 实际 wrench bias/payload 标定；
- 低速、低力 dry-run；
- 硬件 reflex、急停和人工监护；
- sim/real profile 分离。

---

## 15. 必须新增的测试

至少应新增以下测试族：

### 模型与 frame

- flange→adapter→palm 固定变换；
- FK site pose 与独立 FK 一致；
- world/base/object/wrist 变换 round-trip；
- right-hand 安装方向和手指命名不变。

### IK 与规划

- 可达、不可达、关节限位和奇异位形；
- 相邻 waypoint 不跳 IK branch；
- arm/table、自碰撞、adapter、hand 和邻物负例；
- attached object carry collision；
- trajectory velocity/acceleration/jerk gate。

### 控制与安全

- trajectory cancel 延迟；
- arm/hand 同步 stop；
- guarded contact 超力；
- controller mode 切换失败；
- Franka reflex/error 状态传播；
- 失联、掉落和支撑重接触后的 guarded cleanup。

### 信号与 calibration

- wrench frame 旋转与重力补偿；
- 不同 arm posture 的 baseline；
- 工具/对象 payload 变化；
- arm-only shake response；
- locked/damped/mobile 在相同完整 profile 下的区分。

---

## 16. 接入前必须明确的外部条件

开始实现前至少要确定：

1. 使用 Panda、FR3 还是其他 Franka 型号；
2. 只做 MuJoCo，还是目标包含真实 Franka；
3. Allegro 与 flange 的真实安装变换、adapter CAD/惯量；
4. wrist wrench 来自独立 F/T 还是 Franka external-wrench estimate；
5. 自由空间与接触阶段分别使用什么控制器；
6. 使用哪套 IK/规划栈，以及由哪个仓库负责；
7. arm base、桌面、相机与对象 pose provider 的 frame 标定；
8. 是否允许重新定义 Franka 专属 probe protocol 和 calibration。

这些条件会改变接口和安全结论，不能由执行器静默猜测。

---

## 17. 最终判断

本地资源足以开始构建 `Franka Panda + Allegro` 的 MuJoCo 组合模型，但当前代码不能直接
连接后就运行现有 probe/manipulation：

```text
当前：wrist pose → 6 个理想 actuator

Franka：wrist pose
      → frame conversion
      → continuous collision-aware IK
      → time-parameterized 7-D trajectory
      → phase-specific arm controller
      → arm/hand/contact feedback
      → safety-supervised execution
```

最现实的架构是：

- 保留 carriage backend 作为动作语义和回归基线；
- 新建独立 `franka_allegro_mujoco` backend，不修改既有 carriage scene 的含义；
- 继续让 `ManipulationPlan` 保存设备无关的 wrist/hand 目标和 gate；
- 增加 arm plan compiler 与 Franka-specific execution result；
- 先做自由空间和 `poke/slide`，再做 `heft`，最后重新标定 `shake`；
- 真实 Franka 作为第三个独立 profile，不与 MuJoCo 或 carriage calibration 混用。

因此，最大的风险不是“Franka XML 能不能加载”，而是如果只做模型拼接，现有理想
carriage 所隐藏的不可达性、连杆碰撞、动力学和接触控制问题会全部重新出现，却仍被错误
描述成已经完成机械臂规划。接入工作应被视为新增一个完整执行后端，而不是给当前 wrist
多加七个关节。
