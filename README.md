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
| `reference` | 中央仪器化探针 | 带底缘承托钩的双指参考夹爪 |
| `allegro` | 中央仪器化探针 | Menagerie Wonik Allegro 真正的关节和碰撞体 |

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

- `poke`：以 `probe_force` 法向分量闭环，touch 作为接触 guard；达到目标力或
  最大安全压入量后才形成有效曲线。
- `slide`：PI 维持法向 preload，允许短时 touch 失联恢复；路径完成率和有效接触
  占比都达标后才有效。
- `heft`：`pregrasp → grasp → bounded squeeze → lift`；要求 Allegro 的拇指与
  至少一个对向手指（或 reference 左右夹爪）形成接触，物体脱离 pedestal/table，
  相对腕部稳定且穿透受限。
- `shake`：必须先通过与 heft 相同的抓取和脱离支撑 gate；shake 过程中重新接触
  支撑、持续丢失对向接触或掉落都会使结果无效。

mass/fill 物体初始放在小型中央 pedestal 上。pedestal 比物体底面窄，不带四周
挡墙，因此腰部、凸缘和底面外圈对侧向手指开放。物体在 reset/抓取阶段可以受
pedestal 支撑，但进入 heft/shake 测量前必须连续确认 pedestal/table 接触消失。

碰撞角色在 scene 编译时固定：

- stiffness/material scene 启用中央 probe 碰撞。
- mass/fill scene 禁用中央 probe 碰撞并启用对应 gripper/hand。
- primitive 运行期间不通过切换 `contype/conaffinity` 制造穿模捷径。

## ProbeResult

`ProbeResult` 将执行有效性和属性 feature 分开：

```text
scene_id                     probe/manipulation 场景 provenance
status / controller_status   控制结果
valid                        feature 是否可作为可信 probe 信号
phase_reached                最后到达的状态机阶段
violations                   超力、失联、穿透、支撑接触、滑移等
quality                      路径完成率、接触组、漂移、抬升距离等
features                     属性相关结构化特征
raw_summary                  baseline 和简要诊断
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
- reference 左右夹爪 touch 和 `jointactuatorfrc`
- 直接从 MuJoCo contact buffer 得到的手指分组、pedestal/table 接触、法向力和
  penetration
- manipulation 额外区分每指法向力、手接触到的物体 geom，以及手—桌面/手—托架
  接触，避免把环境碰撞误当成有效抓取或放置接触

## Allegro short_can pick/place

当前唯一落地的下游动作只面向 `mass / short_can / allegro`：

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
`weight_signal_N` 条件化生成。`weight_signal_N < 1.6` 的轻罐采用更低预紧、跳过
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

测试覆盖两个 backend、三个随机 seed、四种 primitive 的有效执行和物理排序，
并包含无效抓取、未完成 slide、固定碰撞角色、6-DoF wrist 和脱离支撑检查。
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
