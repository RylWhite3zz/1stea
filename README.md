# AllegroProbe

AllegroProbe 是 ProbeBench 的 MuJoCo probe 执行层。它接收一个明确的
`ProbeCommand`，执行接近、接触、有效性检查和 probe 原语，并返回带诊断信息的
`ProbeResult`。它不读取隐藏属性来选择答案。

仓库另外包含两条刻意收窄的 manipulation 纵向切片：兼容原有 canonical scene 的
`short_can_pick_place`，以及根据当前物体中心位姿实时计算 wrist 目标、放到固定绝对
位置的 `pose_conditioned_short_can_pick_place`。它们用于验证
`ProbeResult → plan → 6-DoF wrist/16-DoF hand command → closed-loop result`，
不代表通用 manipulation 已经实现。

v1 限定为四个 family/primitive：

| hidden family | primitive | 主要可信信号 |
| --- | --- | --- |
| stiffness | `poke` | 法向力—压入量曲线、估计刚度 |
| mass | `heft` | 脱离支撑后的 baseline-corrected 腕部力 |
| fill | `shake` | 通过 heft gate 后的腕部力矩动态响应 |
| material | `slide` | preload 闭环下的切向/法向力比 |

## 执行后端

两个后端共享相同命令、状态机、传感语义和结果结构：

| backend | `poke/slide` | `heft/shake` |
| --- | --- | --- |
| `reference` | `poke` 用中央仪器化探针；`slide` 用左侧专用指腹 pad | 带底缘承托钩的双指参考夹爪 |
| `allegro` | `poke/slide` 均用食指 `ff_tip` 与指尖触觉 | 完整碰撞 Menagerie Allegro 的 top-entry 中指—拇指夹持 |

创建后端：

```python
from allegro_probe import (
    AllegroHandBackend,
    ProbeCommand,
    ProbeHarness,
    ReferenceProbeBackend,
    make_demo_scene,
)

task = make_demo_scene("mass", n_candidates=3, seed=0)
backend = ReferenceProbeBackend.create(task)
result = ProbeHarness(backend).execute(ProbeCommand("heft", target=1))
```

旧的 `ProbeHarness(AllegroProbeScene(...))` 调用仍然支持，scene 会根据
`SceneConfig.backend` 自动适配为对应 backend。

## 控制与有效性

四种原语统一采用显式分阶段控制：

```text
approach
→ guarded contact/descent
→ contact establishment
→ contact quality gate
→ primitive execution
→ post-check
→ retreat
```

这里的 wrist 是 MuJoCo 中的 6-DoF task-space carriage：`x/y/z + roll/tilt/yaw`。
各阶段在预先定义的目标位姿之间平滑插值，并根据接触、力和超时条件转移。它不是
机械臂、IK、关节空间避碰或 MPC 规划。

关键 gate：

- `poke`：reference 以中央 `probe_touch/probe_force` 闭环；Allegro 将手在高位翻转，
  只允许 `ff_tip_fingertip_collision` 接触指定 target，并以 `ff_tip_touch` 闭环。
  Allegro 默认目标/上限为 0.8/1.0 N，目标穿透上限 0.5 mm；非目标手指、指节、
  掌面、桌面、邻物或中央 probe 接触均立即无效。
- `slide`：两套后端都先用单个实体指腹建立稳定 preload，再以 wrist-z PI 维持法向
  触觉读数，并由 wrist-x 执行 20 mm、10 mm/s 的往返；路径完成率使用实际指腹位移。
  Allegro 只允许 `ff_tip_fingertip_collision`，reference 只允许
  `ref_left_slide_pad_geom`。切向力来自 baseline-corrected wrist F/T，摩擦统计排除
  起步、换向和端点瞬态。持续失联、持续超力、物体平移超过 3 mm 或有效接触占比不足
  都会使结果无效。
- `heft`：reference 使用左右夹爪；Allegro 在高位翻腕 `Rx(pi)`，从上方以中指—拇指
  夹住 top lip。腕部最多允许移动 35 mm，但停止条件看物体几何中心的实际抬升：
  默认 8 mm、20 mm/s、连续脱离支撑 120 ms、目标带稳定 80 ms。测量是 200 ms
  静态 hold，重量沿世界重力轴从“支撑中 baseline → 抬升后受力”得到。
- `shake`：必须先通过同一 unsupported lift gate；3°/3 Hz 单轴 micro-shake 前根据
  容器半径和半高补偿底面扫掠净空，再采 200 ms 抬升后动态 baseline。分析窗口使用
  实际 wrist tilt 的整周期 lock-in。额外高度补偿只能发生在 baseline 前的独立稳定
  阶段；baseline/drive/return 固定 wrist z，避免把双输入响应伪装成单轴 feature。
  每步以物体所有外部碰撞 geom 的实时最低点检查相对桌面/托架至少 1.5 mm 净空，
  不用 wrist 角度代替容器自身姿态。动作结束必须精确回零并复核支撑、抓持和相对位姿。

Allegro 的 mass/fill 默认让物体直接落在桌面，不使用托架；reference 为暴露底缘仍
保留小型中央 pedestal。两条路线都必须在 heft/shake 测量前连续确认 target 已脱离
table/pedestal。测量结束后不再空中松手：控制器先把物体放回原支撑面、完全张手、
确认支撑稳定，再垂直退场。

碰撞角色在 scene 编译时固定：

- 只有 reference stiffness scene 启用中央 probe 碰撞。
- Allegro stiffness 以及两个 backend 的 mass/fill/material scene 都隐藏并禁用中央
  probe；material 仅启用该 backend 的单个 slide 指腹。
- 每个 material target 都显式建立“指腹—表面”接触 pair，pair 的切向摩擦系数取该
  target 的 `friction_mu`，避免 Allegro 指尖的通用高摩擦参数覆盖被测物性。
- Allegro probe scene 默认编译 palm/base/proximal 在内的全部解析碰撞 proxy；视觉
  mesh 仍是 Menagerie 的无碰撞渲染层，但不再存在“看得见却没有对应刚体 proxy”的手部。
- primitive 运行期间不通过切换 `contype/conaffinity` 制造穿模捷径。
- 每个仿真 step 都审计 target/非 target 接触、最深 penetration 和接触 pair；掌心、
  手—桌、手—托架、手/target—邻物以及 probe—非 target 接触立即使结果无效。

根因、几何定义、状态机 gate 和验收矩阵见
[`docs/v1/0710/probe_collision_integrity_fix.md`](docs/v1/0710/probe_collision_integrity_fix.md)。

## ProbeResult

`ProbeResult` 将执行有效性和属性 feature 分开：

```text
scene_id                     probe/manipulation 场景 provenance
protocol_id / mode            版本化动作语义，不允许 supported fallback 冒充
feature_schema                feature key 的版本
sensor_profile_id             实际 effector/sensor 组合
status / controller_status   控制结果
valid                        feature 是否可作为可信 probe 信号
phase_reached                最后到达的状态机阶段
violations                   超力、失联、穿透、支撑接触、滑移等
quality                      路径完成率、接触组、漂移、抬升距离等
features                     属性相关结构化特征
raw_summary                  baseline、逐阶段 collision maxima 和最深非法 pair
trace                        可选完整时序
```

控制失败时，质量、刚度、填充或摩擦估计不会被伪装成成功 feature。`to_dict()`
默认不展开时序；使用 `to_dict(include_trace=True)` 可包含 trace。

## 传感器

统一传感包括：

- `probe_touch`、`probe_force`、`probe_framepos`
- `wrist_force`、`wrist_torque`、wrist pose
- wrist 六轴 joint position/velocity
- 物体 position/quaternion
- Allegro fingertip touch/position、actuator force、`jointactuatorfrc`
- reference 左右夹爪 touch、slide pad touch/position 和 `jointactuatorfrc`
- 直接从 MuJoCo contact buffer 得到的手指分组、pedestal/table 接触、法向力和
  penetration
- manipulation 额外区分每指法向力、手接触到的物体 geom，以及手—桌面/手—托架
  接触，避免把环境碰撞误当成有效抓取或放置接触

fill 默认 demo 是 `track="content_mobility"`：fixed/damped/mobile 三个不透明密封容器
具有相同外壳、总质量、静态填充率、内部质量、静态质心和 joint range，只改变内部质量
是否可动及阻尼。旧的 fill-ratio 对照仍可用 `make_demo_scene(..., track="fill_ratio")`
显式生成，但也保持总质量一致，避免 `heft` 重量捷径。

完整 v2 动作、gate、字段与 ProbeBench 适配边界见
[`docs/v1/0710/probe_protocol_v2.md`](docs/v1/0710/probe_protocol_v2.md)。

## Allegro short_can pick/place

旧的 `short_can_pick_place` v1 纵向切片只面向
`mass / short_can / allegro`。它仍使用历史 side-wrap 模板，因此只能在显式隔离的
`allegro_grasp_lift=0.09, full_hand_collisions=False, wrist_roll_limit_rad=0.9`
兼容 scene 中执行；安全默认 scene 会在 plan admission 阶段拒绝它。新代码不把这条
兼容路径包装成 full-collision 抓取。

```text
valid Allegro heft ProbeResult
→ canonical reset handoff
→ object-specific 16-DoF preshape/contact/squeeze template
→ waist contact + mf/th opposing-contact gate
→ lift ≥ 20 mm
→ carry ≥ 80 mm
→ object-space XY correction
→ guarded near-table descent
→ optional low-stiffness gravity settle
→ low-stiffness symmetric opening
→ retreat and final placement verification
```

这里的目标法向力语义固定为所有合法手—物接触法向力幅值之和；它由
heft 的质量/重量信号条件化生成。校准为轻罐的对象采用更低预紧、跳过
近桌面二次纠偏，并在固定腕部下用低刚度指间笼约束物体靠重力下滑到桌面；普通/重罐
使用更高法向力和二次纠偏后直接近表面释放。放置全程监控手—桌面力，轻/重分支分别
在 20 N/30 N guard 处停止继续下压，40 N 为硬失败上限。最终仍要求物体直立、落在
目标区、稳定受桌面支撑且手—物、手—桌面接触完全消失。

这条路径仍使用理想 6-DoF carriage，不做机械臂规划。`reference` backend 保留为
probe 回归基线，不执行这个 Allegro 专属动作。

## 无学习 pose-conditioned pick/place

新的 manipulation 接口接收有效 `heft ProbeResult` 和调用方给出的
`ObjectPoseObservation(T_world_object)`；固定世界系目标由
`PoseConditionedShortCanController` 持有。它按圆柱 z 轴对称性生成并筛选
`staging/pregrasp/grasp/lift/carry` wrist pose，执行 top-entry 中指—拇指夹取，
不再依赖掌心穿过物体或托架支撑。

该路径要求独立的 manipulation scene 配置：

```python
scene = AllegroHandBackend.create(
    spec,
    allegro_grasp_lift=0.0,        # 物体直接在桌面
    full_hand_collisions=True,     # 编译时启用 palm/base/proximal 等
    wrist_roll_limit_rad=np.pi,    # top-entry Rx(pi)
).scene
```

正式 handoff 使用 `verify_live_pose`：不 reset，执行前复核 scene 当前物体中心与请求
位姿。`reset_to_requested_pose` 只用于可复现的仿真 fixture，它会 reset 并设置自由物体
位姿，不能等同于真实定位执行。

规划和执行均检查 table workspace、其他候选物体净空、编译后的完整碰撞 mask、
actuator range、mf/th 双指分组力、合法 link、palm/桌面/其他物体接触、force 和
penetration。固定目标按三维中心误差、目标轴倾角、稳定性和完全松手验收。

完整接口、变换约定、控制信号来源和限制见
[`docs/v1/0710/learning_free_pose_pick_place.md`](docs/v1/0710/learning_free_pose_pick_place.md)。

## 运行

依赖：

- Python 3.10+
- MuJoCo 3.1+
- NumPy
- Allegro 后端需要 MuJoCo Menagerie 的 `wonik_allegro/right_hand.xml`

默认 Menagerie 路径：

```text
/home/enovo/robots/sim/mujoco_menagerie/wonik_allegro
```

示例：

```bash
conda activate probebench
python -m pip install -e .

python -m examples.run_probe_demo \
  --backend reference \
  --family mass \
  --candidates 3 \
  --reset-between-probes

python -m examples.run_probe_demo \
  --backend allegro \
  --family fill \
  --candidates 3 \
  --reset-between-probes \
  --viewer

python -m examples.run_short_can_pick_place \
  --seed 0 \
  --target 2 \
  --viewer

python -m examples.run_pose_conditioned_pick_place \
  --seed 0 \
  --target 2 \
  --source-x 0.11 \
  --source-y -0.09 \
  --place-x 0.0 \
  --place-y 0.12 \
  --viewer
```

添加 `--include-trace` 会在 JSON 中输出完整时序。

运行测试：

```bash
python -m pytest -q
```

probe 回归覆盖两个 backend、五个随机 seed、四种 primitive、三个 target，即 120 次
有效执行和物理排序；并逐次检查穿透、非法接触、无效抓取、未完成 slide、固定碰撞
角色、真实 tip 坐标、6-DoF wrist、脱离支撑、错误 collision model 和拥挤邻物负例。
`short_can_pick_place` 另外覆盖 Allegro 的 3 seed × 3 target 全网格、无效 plan
准入、16-DoF 模板、质量条件化参数、放置稳定性和 gain 恢复。
pose-conditioned 路径另外覆盖 SE(3) frame 约定、full-collision 编译复验、绝对固定
目标归属、障碍净空、plan 防篡改、真实 `ProbeHarness heft` 的轻/中/重罐闭环，以及
`verify_live_pose` 成功和 mismatch 分支。

## 边界

本仓库仍然只是执行层：

- 不包含 ProbeBench split、评分、leaderboard 或 belief model。
- 不设计 VLM 图像/历史编码、probe 选择和停止策略。
- 除上述两个 short-can 纵向切片外，不包含最终 manipulation 动作空间或通用成功
  判定。
- 不包含机械臂、IK、运动规划或任意 mesh 的通用抓取。
- v1 对象是为可重复 probe 设计的解析几何和 stiffness/slosh proxy。

DexJoCo 只用于参考 task-space pose/hand action 的接口分层；DexGraspBench 只用于
参考分阶段抓取和接触质量检查。本项目不依赖或导入这两个仓库。
