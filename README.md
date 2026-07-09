# AllegroProbe

AllegroProbe 是 ProbeBench 的 MuJoCo probe 执行层。它接收一个明确的
`ProbeCommand`，执行接近、接触、有效性检查和 probe 原语，并返回带诊断信息的
`ProbeResult`。它不读取隐藏属性来选择答案。

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
```

添加 `--include-trace` 会在 JSON 中输出完整时序。

运行测试：

```bash
python -m pytest -q
```

测试覆盖两个 backend、三个随机 seed、四种 primitive 的有效执行和物理排序，
并包含无效抓取、未完成 slide、固定碰撞角色、6-DoF wrist 和脱离支撑检查。

## 边界

本仓库仍然只是执行层：

- 不包含 ProbeBench split、评分、leaderboard 或 belief model。
- 不设计 VLM 图像/历史编码、probe 选择和停止策略。
- 不包含最终 manipulation 动作空间或成功判定。
- 不包含机械臂、IK、运动规划或任意 mesh 的通用抓取。
- v1 对象是为可重复 probe 设计的解析几何和 stiffness/slosh proxy。

DexJoCo 只用于参考 task-space pose/hand action 的接口分层；DexGraspBench 只用于
参考分阶段抓取和接触质量检查。本项目不依赖或导入这两个仓库。
