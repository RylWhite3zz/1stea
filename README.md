# AllegroProbe

这个目录从 ProbeBench 中独立出来，只负责一件事：在 MuJoCo 中控制
Wonik Allegro 灵巧手执行 probe，并把仿真传感器数据整理成结构化特征。

当前实现的 primitive：

| hidden family | primitive | 主要信号 |
| --- | --- | --- |
| stiffness | `poke` | 探针法向力、压入量、估计刚度 |
| mass | `heft` | 腕部 z 向力、抬升状态、估计质量 |
| fill | `shake` | 腕部力矩、晃动响应、填充代理特征 |
| material | `slide` | 法向力、切向力、估计摩擦系数 |

`poke` 和 `slide` 使用腕部中央的仪器化探针；`heft` 和 `shake` 使用
Menagerie Allegro 的真实关节、碰撞体和位置执行器完成抓取。传感器包括
指尖触碰、腕部 force/torque、探针 force/touch、关节位置/速度和物体位姿。

## 边界

本项目目前是 probe 执行层，不是完整 benchmark，也不是完整机器人 agent。

- 已实现：MuJoCo 场景、Allegro 低层位置控制、四种 probe 状态机、feature
  提取、被动 viewer。
- 仅有协议示意：`VLMPolicy`、`ProbeCommand`、`ManipulationCommand` 和
  `ManipulationController`，见 `allegro_probe/interfaces.py`。
- 尚未设计：图像/历史如何编码给 VLM、VLM 如何选择下一次 probe、终止
  probe 的策略、OpenAI 多模态消息格式、最终 manipulation 动作空间与成功判定。
- 不包含：ProbeBench split、数据集、评分、leaderboard、belief model 或
  根据隐藏真值自动选答案的 reference policy。

因此当前可调用边界只有：

```python
result = ProbeHarness(scene).execute(
    ProbeCommand(primitive="heft", target=1)
)
```

未来 VLM 应输出 `ProbeCommand`，harness 执行后再把 `ProbeResult` 和视觉观测
交还给 VLM。最终 manipulation 需要另一套控制器和评测定义，现在不做假设。

## 依赖

- Python 3.10+
- MuJoCo 3.1+
- NumPy
- MuJoCo Menagerie 的 `wonik_allegro/right_hand.xml`

本机默认模型路径：

```text
/home/enovo/robots/sim/mujoco_menagerie/wonik_allegro
```

也可以通过 `--menagerie-root` 指定。

## 运行

在本目录执行：

```bash
conda activate probebench
python -m pip install -e .
python -m examples.run_probe_demo \
  --family mass \
  --candidates 3 \
  --reset-between-probes
```

显示 MuJoCo 画面：

```bash
python -m examples.run_probe_demo \
  --family mass \
  --candidates 3 \
  --reset-between-probes \
  --viewer \
  --hold-open
```

四类任务可分别使用 `stiffness`、`mass`、`fill`、`material`。演示程序会对
所有候选物体执行对应 probe 并打印 feature，但不会替 VLM 做选择，也不会
执行后续 manipulation。

## 目录

```text
allegro_probe/
  scene.py          MuJoCo XML 装配、Allegro 控制、传感器读取
  primitives.py     poke/heft/shake/slide
  models.py         场景、物体和 probe 结果数据结构
  interfaces.py     尚未落地的 VLM/manipulation 宏观协议
  demo_scenes.py    仅用于运行 probe 的简单几何体
examples/
  run_probe_demo.py
tests/
```

当前手腕轨迹和抓取姿态是针对这些简单几何体调出的固定控制序列，还不是
通用 IK、运动规划或鲁棒抓取系统。换物体尺寸、位置和随机种子后需要重新验证。
当前 feature 也没有做跨场景物理标定，例如 `m_est_kg` 会混入手部和腕部
动力学影响，应先视为用于同场景比较的质量代理，而不是可靠的绝对测量值。


当前配置：
任务	主要碰撞体	读取信号
stiffness / poke	中央 probe_tip_geom 与物体	probe_touch、probe_force
material / slide	中央 probe_tip_geom 与物体	法向 touch、探针三轴 force
mass / heft	Allegro 中段、远端、指尖碰撞体	腕部 force、指尖 touch、物体位姿
fill / shake	Allegro 中段、远端、指尖碰撞体	腕部 force/torque、指尖 touch、物体位姿

另外：
手掌、基座和近端指节碰撞被关闭，但视觉 mesh 仍显示。
poke/slide 时 Allegro 的有效碰撞体没有关闭，只是主要接触由中央探针完成。
heft/shake 时会关闭中央探针碰撞，避免它干扰抓取。
隐藏液体球的碰撞关闭，液体效果仅通过内部滑动关节和惯性模拟。
桌面、托架和物体几何体也都参与碰撞。
所以目前不是“每个任务只启用指定传感器”，而是按任务切换主要执行碰撞体，但场景中仍有其他碰撞体存在。
