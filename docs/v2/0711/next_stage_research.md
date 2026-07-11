# 阶段 2 前需要提前完成的调研

## 1. 调研目标和范围

阶段 1 只建立 `franka_allegro_mujoco` 的 23-DoF 可信模型。下一阶段要回答的是：

```text
现有 probe/manipulation 的物理 phase 目标
如何在不借助理想 carriage 的前提下
编译成连续、无碰撞、满足动态限制并可被接触 gate 中止的 Panda+Allegro 控制信号？
```

目标位置可以来自确定的对象位姿与固定规则；不要求 learning。仓库继续只承担执行层，
不新增 VLA/VLM 策略、任务评分或 manipulation 决策模型。

调研结论应形成可实现的接口、候选方案对比和小规模数值实验，不以“找到一个相关库”或
“一个 endpoint IK 有解”作为完成。

---

## 2. P0：phase IR，而不是 VLA action chunk

### 2.1 为什么需要 IR

当前 primitive 直接调用 carriage `scene.command(x/y/z/roll/tilt/yaw)`。Panda 后端需要先
保留动作物理语义，再由设备编译器决定 joint path 和控制模式。建议研究一个小型 typed
phase IR：

```text
JointMove
CartesianMove / CartesianConstraint
GuardedApproach
HandTarget / HandClosure
HoldAndMeasure
AttachObject / DetachObject
PlaceUntilSupport
Retreat
Recovery
```

每个 phase 至少携带：

```text
target frame / pose or constraint
arm and hand goals
allowed contact roles
force/distance/time guards
controller mode requirement
completion and failure conditions
sensor/profile provenance
```

### 2.2 “chunk”在这里不采用的语义

若把一段连续轨迹称为 segment/chunk，它只表示执行器可取消、可验收的一段确定性 phase，
不表示：

- VLA 一次预测若干 token/action；
- learned policy 的 horizon；
- 以固定长度数组替代 phase transition；
- 绕过在线 contact/safety gate 的开环播放。

当前更准确的术语是 `phase IR` 或 `executable segment`。

### 2.3 调研交付物

- 能表达当前 `poke/slide/manipulation/heft/shake` 的最小字段集；
- carriage 与 Panda 两个 compiler 的输入/输出草案；
- admission、compile、execute、cancel、result 的失败传播；
- 一段 phase 不携带设备专属 7-D trajectory，编译结果才携带；
- 证明没有把 `ProbeResult` 变成控制信号或策略输出。

### 2.4 P0：MuJoCo world/base/table/workspace frame

阶段 1 是无桌面、无对象的独立机器人模型，当前 `world` 与固定 Panda base 重合。这只证明
robot self-collision，不证明 canonical 在任务场景中不撞桌或能覆盖对象。进入 IK 回归集前
必须冻结：

```text
T_world_franka_base
table top pose / extent / collision profile
candidate object workspace
fixed place region
T_attachment_protocol_wrist / T_protocol_wrist_palm
```

先只定义 MuJoCo 场景 frame；相机外参和实机 base/table 标定仍不在本轮。验收要同时包含
canonical arm/table clearance、四动作目标区的 FK reachable envelope，以及同一对象 pose
在 carriage/Panda compiler 中 frame 语义一致。

---

## 3. P0：连续 7-DoF IK 与整段 admission

### 3.1 必须回答的问题

单个 wrist pose 有解不足以执行动作。需要对完整 phase 序列研究：

- 相邻 waypoint 是否保持同一连续 IK branch；
- 7-DoF 冗余怎样选择肘部姿态；
- short can 的 yaw 对称自由度怎样用于避限位、奇异和碰撞；
- staging、approach、contact、lift、carry、place 是否存在共同可延续的解；
- 当前 q 改变后是否仍能稳定选择近邻分支；
- 无解、近限位或近奇异时怎样在动作前拒绝。

候选排序至少考虑：

```text
joint-limit margin
manipulability / singularity margin
distance from current q
self/environment clearance
path continuity
future phase feasibility
```

### 3.2 候选技术路线

首轮冻结两个可对照的 spike：

```text
A. 只依赖 MuJoCo/NumPy/SciPy 的 Jacobian damped least squares 基线
B. Mink 轻量候选（需要新增依赖后再验证，不预设它一定胜出）
```

二者必须使用同一 MJCF、同一 frame、同一 waypoint 回归集，比较 residual、分支连续性、
joint/singularity margin、约束表达和单步耗时；不应仅因生态完整就先引入 MoveIt。调研需要
用本仓库当前 waypoint 做实测，包括：

```text
poke approach/contact/retreat
slide preload + 全往返路径
short-can pregrasp/grasp/lift/carry/place
heft lift/hold/place-back
shake 的静态姿态和小幅驱动包络
```

### 3.3 完成条件

- 输出整段连续 q path 或具体拒绝原因；
- 不出现相邻 sample 的 branch jump；
- 对初始 q、对象 yaw 和对象位置的小扰动有统计报告；
- 记录最小 joint margin、最小 manipulability 和迭代/耗时；
- 明确 solver 依赖、许可证和本仓库的最小集成面。

这一路线是确定性运动学，不要求 learning。

---

## 4. P0：路径生成——确定性模板优先，必要时才全局规划

### 4.1 首选路线

当前场景和动作受限，先研究由确定 phase 产生的路径：

```text
safe joint staging
→ constrained Cartesian approach
→ short contact segment
→ constrained lift/slide
→ safe joint carry/retreat
```

每段都对**完整离散/连续路径**检查碰撞，而不是只检查 endpoint。若固定 staging、高位和
直线 approach 能覆盖当前 v1 对象，就不需要一开始引入通用规划框架。

### 4.2 何时才启用全局 planner

只有出现以下证据时再研究 RRT/OMPL/MoveIt 等：

- endpoint 可达但确定性自由空间路径被桌面/邻物/自身阻断；
- 需要明显绕障，而不是调整一个安全 staging pose 即可；
- 多个 IK branch 中只有非局部路径可连接；
- attached object carry 不能由受限模板覆盖。

全局 planner 只负责自由空间段；接触 approach、poke、slide、place 仍需受约束局部轨迹和
在线 guard。planner 失败必须返回 `no_plan`，不能回退到未检查的直线插值。

### 4.3 attached object

manipulation/heft 抓持 gate 通过后，规划场景要把对象作为 attached collision object；
确认 release 后才 detach。调研需要验证 object-table、object-neighbor、object-arm/hand 的
碰撞，而不是只看 Panda links。

---

## 5. P0：规划 collision proxy 与运行时碰撞模型

MuJoCo 的 detailed contact geom 适合运行时物理，但连续规划可能需要更简单、稳定和保守的
proxy。需要明确：

```text
runtime collision geom
planner collision proxy
distance-check proxy
phase-specific allowed collision matrix
```

调研问题：

- Panda mesh collision 是否需要 capsule/convex 简化；
- Allegro palm/base/proximal/fingertip 怎样建立保守 proxy；
- synthetic mount 与未来真实 adapter 的包络怎样版本化；
- proxy inflation 如何覆盖离散采样间隙和建模误差；
- grasp/slide/place 不同 phase 如何只允许指定 contact role；
- planner 与 MuJoCo runtime audit 结果不一致时以何种规则拒绝。

阶段 1 已经发现 MuJoCo 3.10 对某些分离 box pair 调用 `mj_geomDistance` 时可能返回 raw
zero、但 `fromto` 端点仍给出正 separation；当前 runtime audit 已对这一具体情况做了回归
修正。下一阶段不能因此假设 detailed mesh/convex 的所有较大正距离都适合作为连续规划
margin，仍需与简化 proxy 逐 pair 对照。

交付物应包含 proxy 可视化、关键 pair 距离对照、保守性测试和 phase allowlist，不只是
一份 geom 名单。

---

## 6. P0：时间参数化

当前 carriage 的 smoothstep 和 `min_steps` 不能转换成 Panda 轨迹。研究输入是连续 q path，
输出是带时间戳的：

```text
q(t), dq(t), ddq(t)（以及需要时的 jerk/effort envelope）
```

需要：

- 每关节 velocity/acceleration/jerk profile；
- 起止速度与加速度边界；
- 相邻 phase 拼接连续性；
- 接触附近单独的低速/低 jerk 限制；
- 执行时使用实际 q/FK，而不是命令轨迹作为测量输入；
- 仿真 profile 与未来 real profile 分开。

首轮比较：

```text
A. dependency-free quintic segment + iterative time scaling
B. Ruckig 候选
```

先调研离线确定性 parameterizer 即可；low-pass/clip 只能做最后 safety guard，不能替代轨迹
生成。

### 6.1 当前依赖事实与决策门

本地 `probebench` 环境当前已有 MuJoCo/NumPy/SciPy；`mink`、`qpsolvers`、`ompl`、
`ruckig`、`pinocchio` 均未安装。项目运行依赖也只声明 MuJoCo 和 NumPy。因此每个候选都要
先通过独立 spike，再决定是否成为正式依赖，并记录版本、许可证、CPU 开销和无该依赖时
是否仍能运行现有两个 carriage backend。阶段 1 不为这些候选提前下载仓库或扩大依赖面。

---

## 7. P0：free-space 与 contact control 分层

至少需要两种执行语义：

```text
free-space:
  time-parameterized joint tracking（或经验证的 Cartesian tracking）

contact:
  Cartesian/joint impedance + force/touch/distance guard
```

重点不是阶段 2 立刻实现通用力控，而是先回答：

- Panda Menagerie position actuator 能否用于 free-space 仿真基线；
- task-space impedance 在 MuJoCo 中怎样映射到 7 个 joint actuator；
- arm 和 16-DoF hand 的命令率、timestamp 和 phase owner；
- fingertip/wrist 超力后怎样在同一 step 停止 arm 并冻结/松开 hand；
- controller mode 切换的状态与失败语义；
- recovery 是退回、放回支撑还是保持抓持。

不能用高增益 position tracking 把穿透/超力变成“轨迹完成”，也不能用过软控制让 arm
compliance 被误解释成对象刚度或摩擦。

---

## 8. P0：wrist F/T 的来源和语义

当前 carriage 的 `wrist_force/wrist_torque` 不能直接迁移。MuJoCo Panda 阶段要先冻结：

```text
sensor site/body
reported frame and sign
force/torque units
sampling rate and filter
baseline window
hand/mount payload compensation
gravity and inertial compensation
sensor_profile_id
```

需对比两种仿真信号：安装处显式 force/torque sensor，与通过 joint force/Jacobian 推算的
external wrench。两者不能无标识混用，也不能复用当前 carriage calibration。

显式 sensor spike 要使用 attachment/adapter 链上的 welded dummy sensor body，验证 MuJoCo
force/torque 所测 child-parent 相互作用的表达 frame 和符号；不能只是把 site 画在 palm 上
便假设已经得到腕部载荷。

建议顺序：先在静态多姿态验证空载与挂手 baseline，再做已知 payload，最后才将其用于
heft/shake feature。`shake` 还需要 arm-only、tool-only 和 locked-content 动态 baseline，
否则机械臂和控制器传递函数会被当成内容物响应。

真实独立 F/T 或 Franka external-wrench estimate 的选择属于实机阶段，本轮只需让接口和
profile 不阻断以后替换。

---

## 9. 动作迁移顺序与各自的研究 gate

冻结顺序：

```text
poke
→ slide
→ manipulation/heft
→ shake
```

### 9.1 `poke`

先验证最短链路：IK → 无碰撞 approach → 食指 guarded contact → retreat。重点研究 surface
normal frame、低延迟停止，以及怎样分离 arm/hand compliance 与对象 stiffness。

### 9.2 `slide`

在 poke 可靠后增加切向路径和法向 preload 闭环。重点是 Jacobian 方向耦合、实际 fingertip
path completion、stick-slip 与 arm controller 振动的区分。

### 9.3 manipulation / `heft`

二者共享 pregrasp、grasp gate、attached object、lift、support-clearance 和安全 place-back。
先验证确定性 short-can/fixed-place 纵向切片，再把 lift/hold 信号用于 heft。目标 pose 可以
实时由对象 pose 几何计算，也可以使用明确的固定位置；两者都不要求 learning。

本阶段继续保留当前 v1 `heft` 的“真实脱离支撑后测量”语义，不采纳旧调研中先改成
`quasi-heft` 的建议。如真实 Panda 路径证明该动作在目标场景不可安全执行，再通过独立
协议版本讨论，而不是在迁移时静默改义。

### 9.4 `shake`

最后迁移。必须重新求得 Panda profile 可执行的幅值/频率，检查 joint
velocity/acceleration/effort、实际 wrist input 的幅相、抓持相对运动和底缘净空。不能直接
继承 carriage 的 `3° / 3 Hz` calibration，也不能用命令 tilt 代替实际 FK tilt。

仅从正弦输入看，`3° / 3 Hz` 已对应约 `0.99 rad/s` 的腕部角速度峰值和
`18.6 rad/s²` 的角加速度峰值；映射到冗余 7-D joint trajectory 后还可能更苛刻。因此调研
必须扫 arm posture/yaw，并以实际 FK、inverse dynamics 和 arm-only baseline 决定新 profile。

当前 v1 仍保留真实脱离支撑后的 micro-shake；旧报告中的 `constrained-shake` 不作为本轮
默认语义。液体/内部动力学继续使用当前简化 proxy，不在机械臂接入时升级成 CFD。

---

## 10. 调研结果应怎样进入执行层

建议形成以下设备相关编译结果，而不是把 7-D trajectory 写回 skill-level plan：

```text
PhasePlan / ManipulationPlan
→ FrankaPlanCompiler
→ IK candidate sequence
→ collision-checked path
→ time parameterization
→ FrankaExecutablePlan
   ├── phase trajectories
   ├── hand targets
   ├── controller modes
   ├── contact/safety guards
   ├── expected frames/profiles
   └── compile diagnostics
→ PhaseExecutor
→ execution result
```

compile diagnostics 至少包括：

```text
executable / reason
IK solution and branch information
minimum joint/singularity/collision margins
trajectory duration and dynamic maxima
planning-scene revision
robot/mount/controller/sensor profile IDs
```

仓库仍只负责从确定的 skill 请求到控制与执行结果；对象选择、belief update、VLM/VLA、
leaderboard 和上层 manipulation policy 继续在仓库外。

---

## 11. 当前明确不调研的内容

- VLA、VLM action 生成、learned action chunk；
- learning-based grasp pose、端到端策略或在线 imitation/RL；
- 横向平台、通用 benchmark adapter 或多机器人统一接口；
- 高保真液体 CFD/SPH/VOF；
- 任意 mesh、任意障碍、任意机械臂的通用抓取规划；
- ROS 2、libfranka、1 kHz 实机实时回路与真实急停认证；
- 相机/基座/桌面的实机标定；
- 真实 adapter payload、线缆模型和硬件寿命；
- ProbeBench split、评分、belief model、probe 选择与停止策略；
- 在没有本地失败证据前引入完整 MoveIt 系统；
- 重新定义当前四个 probe 的 v1 物理语义。

这些不是永久否定，而是不会阻塞阶段 1 和紧接着的 MuJoCo 规划/控制纵向切片。

---

## 12. 对 v1/0711 旧材料的取舍

| 旧材料中的说法 | 当前决定 |
| --- | --- |
| Franka 接入不能只换 XML | 保留；第三 backend 需要 frame、IK、规划和控制编译层 |
| 先建立独立 Panda+Allegro MJCF | 保留并作为阶段 1 当前任务 |
| Panda 与 Allegro 的 asset/default 冲突需处理 | 保留；使用确定性 namespace 和路径重写 |
| 可把 `franka_allegro_real` 与 MuJoCo 混在第三 backend | 不采用；本轮第三 backend 仅为 `franka_allegro_mujoco`，real 以后另建 profile/backend |
| 尽快使用 MoveIt/全局 planner | 调整；确定性 staging/approach path 优先，有失败证据再引入 |
| 先冻结 `quasi-heft` / `constrained-shake` | 不采用；当前 v1 heft/shake 语义保留，按动作迁移结果再决定是否另起协议版本 |
| 首先搭建实机 1 kHz FCI/control | 延后；阶段 1 和下一纵向切片只做 MuJoCo，但接口避免阻断实机 |
| 用完整液体仿真增强 shake | 不采用；继续使用当前等质量内部动力学 proxy |
| 用 VLA/learning 生成 manipulation | 不在范围；确定性规划或固定位置均可 |
| `ManipulationPlan` 直接保存 7-D 轨迹 | 不采用；设备轨迹属于 compiler result |

因此旧文档仍可作为问题清单和背景材料，但实施顺序、backend 数量、heft/shake 语义、
planner 依赖和实机优先级以本文件为准。
