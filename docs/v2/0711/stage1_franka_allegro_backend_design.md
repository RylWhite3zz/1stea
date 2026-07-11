# 阶段 1：第三执行后端与 Panda+Allegro 模型设计

## 1. 本文冻结的结论

本阶段新增第三个执行后端：

```text
franka_allegro_mujoco
```

三个后端的关系是并列而不是替换：

| backend | 上游载体 | 当前能力 |
| --- | --- | --- |
| `reference` | 理想 6-DoF carriage + 参考探针/夹爪 | 现有四个 probe 回归 |
| `allegro` | 理想 6-DoF carriage + Allegro | 现有四个 probe 与收窄的 manipulation 纵向切片 |
| `franka_allegro_mujoco` | Panda 7-DoF + Allegro 16-DoF | 阶段 1 只验证模型、frame、碰撞和 actuator 接线 |

阶段 1 **不执行** `poke/heft/shake/slide` 或 manipulation，也不把理想 carriage 串在
Panda 末端。通过阶段 1 只能说明“一个 23-DoF Panda+Allegro 仿真实体可信地装配并可被
分别驱动”，不能说明 wrist waypoint 已经可达、路径无碰撞，或者接触动作已经可执行。

阶段 1 的硬目标是：

```text
独立 MJCF 可编译
+ nq/nv/nu = 23/23/23
+ canonical q 合法且无未声明的自碰撞
+ attachment / protocol_wrist / palm / fingertip FK site 正确
+ Panda 7 actuator 和 Allegro 16 actuator 可逐个独立小幅动作
+ 不含 wx/wy/wz/wr/wt/wyaw 等隐藏 carriage 自由度
```

详细验收见 [stage1_acceptance_and_usage.md](stage1_acceptance_and_usage.md)。阶段 2 之前要做
的调研见 [next_stage_research.md](next_stage_research.md)。

---

## 2. 为什么不能只把第三个字符串加入现有 `BACKENDS`

当前 `reference` 和 `allegro` 虽然手部结构不同，但都由 `AllegroProbeScene` 提供同一套
理想任务空间接口：

```text
scene.command(x, y, z, roll, tilt, yaw)
scene.wrist_pos()
act_wx / act_wy / act_wz / act_wr / act_wt / act_wyaw
```

现有 primitive 直接调用这些接口。Panda 后端没有、也不应伪造这些 actuator。若仅把
`franka_allegro_mujoco` 放进原先用于 probe 参数化的 `BACKENDS`：

1. 现有 probe 测试会误以为第三后端已经支持四个动作；
2. `as_backend()` 的二分逻辑可能把未知后端当成 `allegro`；
3. `ProbeBackend.scene` 仍被具体绑定到 `AllegroProbeScene`；
4. primitive 会继续绕过机械臂运动学，直接发 task-space carriage 命令。

因此这次重构必须先把“后端身份”和“已支持能力”分开。建议边界如下：

```text
ExecutionBackend
  identity/profile/reset/state/step
  ├── CarriageProbeBackend          # reference, allegro
  │     supports: poke/heft/shake/slide
  └── FrankaAllegroMujocoBackend    # 阶段 1
        scene API: reset, joint command, frame query, collision audit
```

Python 类型名可以随实现调整，但以下语义必须保留：

- `ProbeHarness` 只接受声明了相应 primitive capability 的后端；
- 不支持的动作在 admission 时明确拒绝，不能进入 primitive 后才因缺少属性崩溃；
- `SceneConfig` 继续只描述现有 carriage scene，Panda scene 使用独立配置；
- backend 适配使用显式注册或显式类型分派，不能用“非 reference 都是 allegro”的默认分支；
- 现有两个后端、命令和结果结构保持兼容，第三后端不会扩大 `ProbeResult` 的含义。

代码中的 `BackendCapabilities` 目前只声明可执行的 probe primitive。第三后端在阶段 1 的
声明应为：

```text
supported_primitives = frozenset()
```

模型编译、canonical reset、joint command、frame query 和 collision audit 由独立 scene
API 与测试暴露；它们是阶段 1 的**验收能力**，不是另外一组 capability 字符串。以后若要
让统一 registry 同时描述低层模型能力，应另行版本化，不能和 primitive capability 混用。

当前代码尚无 manipulation capability registry；即使以后新增，阶段 1 也不能给第三后端
声明任何 manipulation skill。

---

## 3. 独立 scene 和模型装配边界

### 3.1 模型来源

阶段 1 使用本地 MuJoCo Menagerie：

```text
/home/enovo/robots/sim/mujoco_menagerie/franka_emika_panda/panda_nohand.xml
/home/enovo/robots/sim/mujoco_menagerie/wonik_allegro/right_hand.xml
```

本次验收时 Menagerie 为 clean revision
`71f066ad0be9cd271f7ed58c030243ef157af9f4`。运行时 provenance 另外记录两份源 XML 的
SHA256；以后 revision 或 XML hash 改变必须重新跑阶段 1 gate。

装配结果当前由独立的 `FrankaAllegroScene` 持有，而不是把 Panda joint 塞进
`AllegroProbeScene`。后者仍是 carriage probe 的来源，不应因机械臂接入改变物理语义。

模型构建必须记录：

```text
panda/allegro source XML path + SHA256
mount_profile_id
Menagerie root / commit（或等价内容 hash）
MuJoCo version
solver / cone / timestep profile
```

这样上游 Menagerie 更新后，不会在不知情的情况下改变碰撞、惯量或 actuator。

### 3.2 MJCF 合并不能依赖偶然的名称覆盖

两个源模型都使用相对的 `meshdir="assets"`，也都有 `white` 等 asset 名。合并层必须做
确定性的路径重写与命名空间隔离，例如给手部资源统一增加 `hand/` 前缀，并同步更新所有
引用。还需检查：

- default class、mesh、material、body、joint、geom、site、actuator 是否唯一；
- 顶层 `<compiler>` 和 `<option>` 由哪个 profile 统一；
- Panda 的 collision exclude 是否在装手后仍只排除了预期相邻结构；
- Allegro visual mesh 与解析 collision geom 是否都保留；
- 组合 keyframe 是否具有完整 23 维 `qpos` 和 `ctrl`。

不允许通过“后加载的同名 material 覆盖前一个”或依赖当前工作目录解析 mesh。

当前实现使用 `MjSpec` attachment API，因此项目最低依赖已冻结为 MuJoCo `>=3.3.1`；本地
验收环境为 `3.10.0`。模型测试还会在改变当前工作目录后重新编译，证明 asset 路径不依赖
仓库 cwd。

### 3.3 模型自由度和 actuator 顺序

固定基座、固定 adapter 的组合模型应满足：

```text
q = [q_panda(7), q_allegro(16)]
u = [u_panda(7), u_allegro(16)]
nq = nv = nu = 23
```

代码不能仅依赖“恰好先 arm 后 hand”的 XML 顺序；应按冻结名称查询并保存索引，同时在
初始化时验证索引集合不重复且完整。阶段 1 使用 Menagerie 的 position-like actuator 做
接线 smoke test，不在这里设计最终自由空间或接触控制器。

---

## 4. synthetic mount 的边界

Panda 的 `attachment_site` 只定义了机械臂模型中的安装入口，不能自动给出真实的：

```text
T_attachment_adapter
T_adapter_allegro_palm
adapter mass / CoM / inertia
adapter collision geometry
```

阶段 1 尚没有真实 adapter CAD、实测外参和惯量，因此采用显式版本化的仿真安装件：

```text
mount_profile_id = sim.synthetic_panda_allegro_mount.v1
```

这个 profile 必须在代码中集中保存固定变换、几何与惯量；日志和测试引用 profile ID，
不要在多个文件复制一组匿名 `pos/quat`。它的承诺只有：

- Allegro 掌面方向明确；
- canonical q 下 Panda、mount、palm 和手指不存在未声明穿透；
- 有可用于规划和运行时接触的 conservative collision geom；
- frame 组合可重复、可测试。

它**不承诺**与任何真实 flange-to-Allegro adapter 一致，也不能用于推断实机 payload、
wrench baseline、可达空间或线缆净空。以后取得实物资料时应新建
`hardware.<adapter>.vN`，而不是静默修改 synthetic v1。

DexJoCo 中的 Panda+Allegro 变换可以作为“模型怎样拼装”的参考，但其注释本身允许继续
调整，且没有形成可审计的真实 adapter profile，因此不作为本项目的机械真值。

---

## 5. canonical 状态

23 维全零不能作为 canonical：Panda `joint4=0` 超出其负值区间，Allegro
`thj0=0` 也低于 Menagerie 的下限。

阶段 1 的初始候选由两部分组成：

```text
q_panda = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]

q_allegro = [
  0.00, 0.10, 0.05, 0.05,
  0.00, 0.10, 0.05, 0.05,
  0.00, 0.10, 0.05, 0.05,
  0.45, 0.10, 0.08, 0.08,
]
```

前者来自 Panda Menagerie `home` keyframe，后者与当前 carriage Allegro 的 open pose
一致。当前 synthetic mount 的 contact 与 signed-distance gate 已通过，因此这组值就是
`sim.synthetic_panda_allegro_mount.v1` 冻结的 canonical。以后若更换 mount profile，任何
调整仍必须：

1. 保持所有 joint/ctrl 在限位内并留有可报告的 margin；
2. 说明调整的是 mount 还是 q，不能用任意弯手指掩盖错误安装；
3. 将最终 23 维 q 和 ctrl 固化为组合模型 keyframe；
4. `reset()` 后运行 `mj_forward`，不得依赖先执行一段控制轨迹才能进入合法状态。

---

## 6. frame 设计与 FK 真值

阶段 1 至少区分：

```text
world
franka_base
link7
attachment
adapter
allegro_palm
protocol_wrist
ff_tip / mf_tip / rf_tip / th_tip
```

其中：

- `attachment`：Panda Menagerie 的原始安装 site；
- `allegro_palm`：Allegro 刚体根 frame；
- `protocol_wrist`：以后 phase 计划使用的稳定抽象 frame；
- fingertip site：以后接触 waypoint 和传感器语义使用的物理 site。

`protocol_wrist` 不能默认为 `attachment`、`palm` 或某个视觉 mesh 原点；它们恰好重合
也必须通过显式固定变换表达：

```text
T_world_protocol_wrist(q)
  = T_world_attachment(q)
  × T_attachment_adapter
  × T_adapter_protocol_wrist

T_world_palm(q)
  = T_world_attachment(q)
  × T_attachment_adapter
  × T_adapter_palm
```

FK 验收包含两个独立方向：

1. 相同 Panda `q7` 下，合并模型的原始 `attachment` pose 与独立 Panda 模型一致；
2. 由固定变换手工组合得到的 wrist/palm/tip pose 与 MuJoCo site pose 一致。

只打印一个 site 坐标，或者让两个 site 相互比较，不足以证明安装方向正确。

---

## 7. 碰撞语义

“canonical q 无自碰撞”不能简化为 `data.ncon == 0`。MuJoCo 可能因 welded body、父子
关系、contype/conaffinity 或 `<exclude>` 不生成某些接触，即使两个 collision geom 在
几何上相交。

阶段 1 需要同时维护：

```text
structural adjacency / allowed fixed-interface overlap
runtime collision exclusions
forbidden robot self-collision pairs
pairwise geometric clearance audit
```

原则是：

- 只允许 attachment-adapter 等真实固定接口存在有理由的结构重合；
- 不得用大范围 `exclude` 把 link7-palm、link7-finger 或非相邻 arm collision 隐藏掉；
- 每个 exclude 都要有 body/geom 名、理由和对应测试；
- canonical reset 与短时 settle 后都检查接触、penetration、force 和有限状态；
- 碰撞报告使用稳定 geom 名，不输出难以追溯的匿名 ID；
- planner proxy 与运行时 MuJoCo contact geom 的差异留到下一阶段明确，但阶段 1 必须保留
  完整运行时碰撞体。

---

## 8. 数据与控制信号边界

阶段 1 的数据流是：

```text
FrankaSceneConfig
  ├── Menagerie model provenance
  ├── synthetic mount profile
  ├── canonical q23
  └── simulation/actuator profile
        ↓
MJCF composition + compile-time validation
        ↓
FrankaAllegroMujocoBackend.reset()
        ↓
FrankaAllegroState（概念性诊断集合；当前由 scene 属性和查询方法返回）
  ├── q23 / dq23 / ctrl23
  ├── named frame poses
  ├── joint/ctrl limit margins
  ├── contact pairs / distances / penetration
  └── model/profile provenance
        ↓
arm-only or hand-only small position target
        ↓
MuJoCo actuator → dynamics step → updated state
        ↓
smoke-test metrics / diagnostic result
```

这里的 23 维 control target 来自测试或人工 viewer 命令，不来自 `ProbeResult`、
`ManipulationPlan`、IK 或 motion planner。阶段 1 不生成 `ProbeResult`，因为没有执行任何
probe，也不能把 actuator smoke 的成功包装成动作成功。

后续的目标数据流才是：

```text
skill-level phase goal
→ continuous IK / collision path / time parameterization
→ arm trajectory + hand target + guard conditions
→ phase-specific executor
→ state/contact/wrench feedback
```

这一层在阶段 1 中只保留接口落点，不提前用固定轨迹假装已经实现。

---

## 9. 本阶段明确不做

- 不做 IK、冗余肘位选择或运动规划；
- 不把现有 wrist smoothstep 改名后当作 7-DoF 轨迹；
- 不执行 `poke/heft/shake/slide`；
- 不执行 short-can pick/place；
- 不接 ROS 2、libfranka 或真实机械臂；
- 不设计 VLA、VLM、learning grasp 或 action chunk；
- 不改变当前四个 probe 的 v1 物理语义；
- 不实现高保真液体；
- 不把 synthetic mount 的 wrench/dynamics 结果外推到实机。

这仍符合仓库“只保持执行层”的边界：阶段 1 增加的是一个新的仿真实体和执行入口，
不是上层策略、benchmark 评分或 manipulation policy。
