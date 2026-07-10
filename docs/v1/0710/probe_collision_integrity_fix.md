# 四个 probe 动作的穿模修复与验收

## 1. 修复目标

本次只处理当前 v1 的 `poke / heft / shake / slide`。目标不是让画面“看起来更顺”，
而是让成功结果能由 MuJoCo contact buffer 证明：合法工具接触了指定目标，所有可见手部
都有对应刚体碰撞 proxy，掌心、环境和邻近候选没有被穿过。

## 2. 已确认的旧问题

旧 Allegro 默认只启用 medial/distal/fingertip 碰撞，palm/base/proximal 的视觉 mesh
仍显示但解析 collision proxy 被关闭。旧 `heft/shake` 又使用从物体侧下方穿入的固定
under-wrap 轨迹，因此返回 `ok` 时掌心视觉几何实际可进入罐体约 26–29 mm。

旧中央 probe 的接触端几乎位于 wrist，掌心最低点反而比所谓 probe tip 更低；
`poke/slide` 到达物体时，手掌也可能先进入目标。`probe_tip_pos()` 还表示 capsule 端点
球心而非最下方接触表面，额外有 5 mm 坐标误差。

最后，旧状态机主要读取 touch/总力，没有逐步证明接触对象身份，也没有统一拒绝
hand-other-object、hand-table、palm-object 等接触；retreat 发生在有效性判定之后。

## 3. 当前几何与控制

### 3.1 `poke / slide`

- 中央 probe 有效长度为 100 mm、半径为 5 mm；`probe_tip_pos()` 是 capsule 最下方
  物理表面，`wz_for_tip_z()` 与这个 frame 精确互逆。
- probe position joint 使用 `damping=40`、actuator 使用 `kp=40`；material 接触面使用
  `solref="0.017 1"`，降低 position servo 把工具硬压入表面的数值穿透。
- `probe_contact_snapshot(target)` 必须证明 probe 接触的是指定 target；probe—桌面、
  托架、其他候选或其他 geom 均失败。
- 手/掌不能接触 target；真实 probe 穿透硬上限为 1 mm。
- slide 在横移前建立 30 步稳定 preload；完成率来自实际 tip x 位移，而非 command
  插值进度。末端允许一个有界 servo settling 窗口，但它不混入摩擦统计窗口。

### 3.2 Allegro `heft / shake`

- 默认 `full_hand_collisions=True`，并复验 palm、三个手指 base/proximal 与 thumb
  base/proximal 的 compiled `contype/conaffinity`。
- mass/fill 的中央 probe 隐藏且禁碰；对象直接在桌面，不使用 pedestal。
- wrist 在高位先平移，再单独翻到 `Rx(pi)`；之后沿 z 到 pregrasp/grasp，避免旋转
  扫过候选物体。
- grasp wrist 位于 `object_top + 94 mm`，y offset 为 `-20 mm`。
- 16-DoF 手目标按 `synergy(0.10) → synergy(0.80) → synergy(0.98)` 分段插值。
- 只有 `mf + th`、目标 `*_top_lip` 和 fingertip/thumbtip/distal link 白名单构成合法
  top pinch；总法向力目标为 7 N，硬上限 20 N，穿透上限 5.5 mm。
- 默认 lift 为 130 mm；lift、heft 正弦保持和 shake tilt/yaw 全程继续调节 closure。
- 测量结束后先归零 tilt/yaw、下降到原桌面接触稳定、低刚度完全张手、垂直退到
  高位，最后才将 roll 转回 0。安全验收包含 place/release/retreat。

reference 后端仍使用真实可见且可碰撞的左右 jaw/hook 和窄 pedestal，但也进入同一套
逐步 contact audit，并使用“放回—释放—退场”，不再在空中松手。

## 4. 统一逐步接触策略

每个 `scene.step(1)` 后同时读取 `ContactSnapshot` 和 `ProbeContactSnapshot`。以下事件
不会被解释成成功接触：

- palm—target；
- hand—table、hand—pedestal；
- hand—其他候选、target—其他候选；
- probe—非 target；
- `poke/slide` 中任何 hand—target；
- `heft/shake` 中任何 probe—target；
- Allegro grasp 中 ff/rf、base/proximal 或非白名单 link 接触。

`ProbeResult.quality` 给出全过程最大合法/非法穿透与峰值力；
`raw_summary.collision_audit.phase_maxima` 按 phase 保存 maxima，并记录最深非法 contact
pair。violation 同时记录发生 phase。失败结果不输出可信质量/填充/摩擦 feature。

## 5. 兼容边界

旧 `short_can_pick_place` v1 manipulation 仍依赖 side-wrap 和 distal-only collision，
不能在安全默认 scene 中伪装成 full-collision 路线。builder/executor 会拒绝错误 model；
如需历史兼容，必须显式构建：

```python
AllegroHandBackend.create(
    spec,
    allegro_grasp_lift=0.090,
    full_hand_collisions=False,
    wrist_roll_limit_rad=0.9,
)
```

新的 pose-conditioned manipulation 与当前 Allegro probe 都使用 support-free、
full-collision top-entry 路线。

## 6. 验收

自动回归覆盖：

```text
2 backends × 5 seeds × 4 primitives × 3 targets = 120 runs
```

每次成功都要求物理量排序正确、非法接触峰值为零、穿透不越界、grasp 脱离支撑且
最终安全放回。另有两个关键负例：partial-collision Allegro heft 必须拒绝；将候选间距
压到 70 mm 时，邻物接触必须导致 `other_object_collision`，不能仍返回 `ok`。

对应测试入口：

```bash
conda run -n probebench python -m pytest -q tests/test_simulation.py
```
