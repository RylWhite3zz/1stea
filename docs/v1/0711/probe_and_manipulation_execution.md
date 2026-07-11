# Probe 与 Manipulation 如何运行：先得到 wrist 位姿，再执行手指动作

## 1. 先给出最重要的结论

当前仓库里的一个动作可以统一拆成两层：

```text
对象/表面几何 + 动作模板 + 当前传感反馈
→ 得到下一段 wrist 目标（x, y, z, roll, tilt, yaw）
→ 得到 Allegro 16 维关节目标 q_hand
→ MuJoCo 执行一步并检查接触、力、穿透、支撑和邻物碰撞
→ 用新反馈修正下一步 wrist 或手指命令
```

但这里的 `wrist` 不是 Franka 等机械臂算出来的末端位姿。当前
`AllegroProbeScene` 在 Allegro 手掌上游直接放置了一个理想 6-DoF carriage，六个
position actuator 分别控制：

```text
wx, wy, wz, wr, wt, wyaw
```

所以代码可以直接命令 wrist 的任务空间位置和姿态，再在起终点间做 smoothstep
插值；当前没有机械臂 IK、肘部选择、关节空间轨迹规划或机械臂连杆避碰。

手指侧也不是一个学习策略。当前使用预先定义的 16 维 Allegro 手型，并在合法接触
建立后，根据接触法向力小幅增加或减小闭合进度。

四个 probe 原语和 manipulation 的核心区别是：

| 动作 | wrist 主要负责 | 手指主要负责 |
|---|---|---|
| `poke` | 把食指指腹对准表面，并沿 z 建立/维持压入力 | 保持食指伸出、其他手指避让的固定 preshape |
| `slide` | 沿 z 维持 preload，同时沿 x 往返滑动 | 保持同一食指 preshape，不执行抓握闭合 |
| `heft` | 从上方对准、翻腕、下降，并按物体实际抬升高度向上移动 | 中指和拇指闭合成 top pinch，并按接触力持续微调 |
| `shake` | 先完成 heft 式抬升，再固定 z、沿 tilt 轴做正弦激励 | 保持中指—拇指 top pinch，并在晃动中补偿夹持力 |
| pick/place | 由物体位姿和固定目标产生 staging/grasp/lift/carry 等 wrist 位姿 | 中指—拇指闭合、搬运中调力、落桌后低刚度张开 |

本文以当前 Allegro 路径为主。reference backend 的协议语义相同，但 `poke` 使用中央
probe、`slide` 使用专用左指腹 pad、`heft/shake` 使用双指参考夹爪，不是 Allegro
16-DoF 手指控制。

---

## 2. wrist 命令和手指命令在代码里是什么

### 2.1 wrist：物理 wrist pose 与 carriage control

probe 原语中的 `_move_wrist()` 接收：

```python
x, y, z, roll, tilt, yaw
```

函数读取六个 actuator 的当前 target，在当前值和目标值之间做 smoothstep 插值，每个
插值点执行 MuJoCo step 和碰撞审计。它只是理想 carriage 的分段插值，不是 arm
planning。

pose-conditioned manipulation 规划的是物理变换 `T_world_wrist`。执行前通过：

```text
x_cmd = wrist_world_x
y_cmd = wrist_world_y
z_cmd = wrist_world_z - palm_height
roll, tilt, yaw = R_world_wrist 的 XYZ RPY 分解
```

转换成 carriage actuator target。减去 `palm_height` 是当前 MJCF 中 carriage 基座与
物理 wrist site 的建模偏移，不应带到未来 Franka backend。

### 2.2 手指：16 维 position target

Allegro actuator 顺序固定为：

```text
ff[0:4], mf[4:8], rf[8:12], th[12:16]
```

即食指、中指、无名指、拇指各四个关节。基础圆柱闭合手型由：

```python
allegro_grip_pose(alpha)
  = (1 - alpha) * q_open + alpha * q_closed
```

得到。`command_allegro_joints(q)` 把一个 16 维目标分别写入 16 个 position
actuator。

top-pinch 路径不是盲目闭到终点，而是经过：

```text
q_preshape = grip_pose(0.10)
q_contact  = grip_pose(0.80)
q_squeeze  = grip_pose(0.98)
```

接触后根据总法向力调节闭合进度：力太小则再闭一点，力太大则退一点。`0.98` 只是
调节器允许到达的上限，不是每次动作的必达手型。

---

## 3. `poke`：wrist 对准并下压，手指只保持食指探测手型

`poke` 对应 stiffness family。Allegro 使用食指指腹
`ff_tip_fingertip_collision`，不是场景中的中央 probe。

### 3.1 wrist 位姿如何得到

1. 从对象 pose sensor 读取目标顶部世界坐标 `target_top`。
2. 先把 wrist 平移到目标上方的安全高度，并在高位翻腕到 `Rx(pi)`，让指尖朝下。
3. 不使用写死的“wrist 到食指指尖”偏移。代码实时读取
   `fingertip_positions()["ff"]`，设期望指尖位置为：

   ```text
   desired_tip = target_top + [0, 0, 40 mm]
   wrist_cmd_next = wrist_cmd_now + (desired_tip - measured_ff_tip)
   ```

   该修正执行两次，因此 Menagerie 手部运动学或 preshape 有小变化时，仍以实际
   `ff_tip` site 对齐目标。
4. guarded descent 每次把 wrist-z target 下移 `0.15 mm`，随后仿真 3 step，直到食指
   touch 超过接触阈值且接触 geom 身份合法。
5. 接触后 PI-like 法向力环仍然修改 wrist-z，而不是修改食指关节：

   ```text
   error = target_force - ff_tip_touch
   dz = clip(kp * error + ki * integral)
   z_cmd = z_cmd - dz
   ```

   默认目标力为 `0.8 N`，硬上限为 `1.0 N`。

### 3.2 手指如何操作

进入 approach 前，16 个关节移动到专用 `_ALLEGRO_POKE_PRESHAPE`：食指伸出并弯成适合
指腹向下接触的姿态，另外三指保持避让。随后整个加载和 hold 阶段都不做抓握闭合，
主要靠 wrist-z 改变指腹压入量。

合法接触必须满足：

```text
hand group 只有 ff
hand contact geom 只有 ff_tip_fingertip_collision
无 palm、其他手指、其他指节、桌面、邻物或中央 probe 接触
```

动作结束时先保持 `Rx(pi)` 垂直退至少 60 mm，确认食指无接触，再把 wrist 回正并把
手张开。

---

## 4. `slide`：wrist-z 管法向力，wrist-x 管切向路径

`slide` 对应 material family。Allegro 仍只使用食指指腹，手指操作与 `poke` 类似，
区别在于 wrist 同时执行法向 preload 闭环和切向往返轨迹。

### 4.1 wrist 位姿如何得到

设目标中心 x 为 `x0`、单程距离为 `d=20 mm`：

```text
start_x = x0 - d / 2
desired_tip = [start_x, 0, target_top_z + 35 mm]
```

与 `poke` 一样，wrist 先在高位平移、翻转到 `Rx(pi)`，再读取实际 `ff_tip` site，使用
`desired_tip - measured_tip` 两次校正 wrist xyz。这里也没有依赖固定 fingertip offset。

之后分两套同时运行的控制：

1. 法向方向：guarded descent 每次下降 `0.04 mm`；首次接触后用 PI-like wrist-z
   闭环把 `ff_tip_touch` 稳定在 `0.6 N` preload。
2. 切向方向：保持上述 z 闭环，同时令 wrist-x 执行：

   ```text
   start → start + 20 mm → start
   ```

   每个单程默认 2 s，即约 `10 mm/s`。端点是否到达使用实测指尖 x 位移判断，不使用
   actuator 命令值冒充实际路径。

### 4.2 手指如何操作

手仍保持 `_ALLEGRO_POKE_PRESHAPE`，没有 contact/squeeze 闭合过程。换句话说，slide
中的“手指动作”是选择食指指腹作为唯一 effector 并保持其关节姿态；法向 preload 和
切向滑动都由 wrist carriage 完成。

切向力取接触前 wrist F/T baseline 的差值，法向力取 `ff_tip_touch`。只使用往返两腿
各自 10%–90% 的中段估计摩擦比，排除启动、换向和端点伺服瞬态。目标对象平面位移
超过 3 mm、合法接触占比低于 80%、路径完成不足 95% 或持续失联都会使结果无效。

---

## 5. `heft`：先用对象几何算 top-entry wrist，再用中指—拇指夹住

`heft` 对应 mass family。它是第一个真正同时使用 wrist 运动和动态手指闭合的原语。

### 5.1 抓取 wrist 位姿如何得到

先读取目标的几何中心 `center`、顶部半高 `obj.size[2]`，构造：

```text
x_goal = center_x
y_goal = center_y - 20 mm
physical_wrist_z = center_z + half_height + 94 mm
z_grasp_cmd = physical_wrist_z - palm_height
z_pregrasp_cmd = z_grasp_cmd + 75 mm
R_world_wrist = Rx(pi)
```

动作顺序不是同时平移和翻腕：

```text
手指到 preshape
→ wrist 在高位平移到目标 xy
→ 高位单独翻腕 Rx(pi)
→ 下降到 pregrasp
→ guarded descent 到 grasp z
```

这里的几何偏移是当前 short-can/cup top-pinch 的手工模板，不是 IK、抓取网络或在线
碰撞规划结果。

### 5.2 手指如何操作

wrist 到达抓取位姿后，闭合进度从 0 到 1 扫描，16 维手型依次经过：

```text
q_preshape(0.10) → q_contact(0.80) → q_squeeze_limit(0.98)
```

只有以下接触才算合法抓取：

```text
mf 和 th 都接触
接触对象 geom 中必须包含 target 的 *_top_lip
接触 hand link 只来自 fingertip/thumbtip/distal 白名单
无 palm、ff、rf、桌面或邻物提供隐藏支撑
```

合法接触总法向力达到默认 `7 N` 后停止大步闭合，并进入 100 step 稳定窗口。后续每个
lift/measurement step 都会调用夹持调节器：

```text
F_hand < 0.70 F_target  → closure_progress += 0.0015
F_hand > 1.35 F_target  → closure_progress -= 0.0010
F_hand > 20 N           → 立即失败
```

### 5.3 抬升 wrist 位姿如何得到

heft 不把“wrist 上移固定距离”当成成功，而是读对象几何中心的实际高度：

```text
object_lift = current_object_center_z - lift_start_center_z
```

若 `object_lift < 8 mm - 1.5 mm`，才以不超过 `20 mm/s` 的速度继续增加 wrist-z；进入
目标带后保持，不会为了追踪高度主动向下压。wrist 总行程上限为 35 mm，物体抬升硬
上限为 15 mm。

成功获取 measurement 条件还包括：对象连续脱离桌面/托架 120 ms、实际抬升在目标带
稳定 80 ms、mf/th 合法夹持持续存在。静态 hold 中 hand 调节器继续工作，wrist-z 只在
物体下沉到目标带以下时做小幅向上补偿。

质量信号来自“抓住但仍受支撑时的 wrist F/T baseline”与“脱离支撑后的 wrist force”
之差，并旋转到世界坐标后投影到重力轴。测量结束后 wrist 下降到原支撑面，确认物体
重新受支撑，手指低刚度张开，再垂直退场；不会在空中直接松手。

---

## 6. `shake`：沿用 heft 抓持，wrist 做单轴倾转，手指持续保夹

`shake` 对应 fill/content-mobility family。它的抓取 wrist 位姿和手指 top pinch 与
`heft` 完全共享，区别发生在抬升高度规划和动态激励阶段。

### 6.1 shake 的 wrist 高度如何得到

容器倾转时底缘会扫过更大空间，因此不能只抬 8 mm。设容器半径为 `r`、半高为 `h`、
峰值倾角为 `A`，先计算：

```text
nominal_sweep = r sin(|A|) + h (1 - cos(A))
nominal_center_lift = 8 mm + nominal_sweep + 0.5 mm
control_target = nominal_center_lift + 1.5 mm tolerance + 1.5 mm sag reserve
```

默认 `A=3°`。完成初始 lift 后，`height_stabilization` 继续读取对象实际中心高度，只允许
向上补偿，直到进入更窄的 0.25 mm 带并稳定 80 ms。之后冻结 wrist-z。

运行时安全不依赖上述公式，而是逐 step 遍历对象外部碰撞 geom，计算实际最低点相对
源支撑面的净空，要求始终至少 1.5 mm。

### 6.2 wrist 动态动作与手指动作

高度稳定后先在 `tilt=0` 采 200 ms 动态 baseline，然后 wrist 执行：

```text
0.25 cycle smooth ramp-in
→ 2 个完整 analysis cycles
→ 0.25 cycle smooth ramp-out
→ tilt 精确回零并保持 120 ms
```

默认输入为：

```text
tilt_cmd(t) = 3° × envelope(t) × sin(2π × 3 Hz × t)
yaw_cmd = 0
z_cmd = frozen_shake_z
```

分析使用实际 `wt` 角度，不使用命令角度。整个波形期间，mf/th 的 16 维 hand target
继续由同一个法向力调节器更新；如果夹持丢失、物体碰回支撑面、相对 wrist 位姿漂移
过大或实时底缘净空不足，动作立即无效。

因此 shake 的两层可以简写为：

```text
wrist：保持高度 + 单轴小角度正弦倾转
hand：不生成晃动轨迹，只负责持续夹紧且不过力
```

---

## 7. ProbeResult 如何进入 manipulation

当前示例使用两个独立的 MuJoCo scene：一个执行 probe，一个执行 manipulation。
两者可以共享 `ProbeSceneSpec/scene_id`，但 probe 后的物理 qpos/contact state 不直接
延续到 manipulation。

```text
数据连续：ProbeResult 被 plan builder 使用
物理状态不连续：manipulation 单独 reset 或验证自己的 live pose
```

对 pose-conditioned short-can pick/place，输入分别负责：

| 输入 | 决定什么 |
|---|---|
| `ObjectPoseObservation.T_world_object` | 源对象在哪里、朝向如何 |
| `FixedPlaceSpec.T_world_object_goal` | 对象最终应放到哪个绝对世界位姿 |
| top-pinch 几何模板 | wrist 相对对象应该在哪里 |
| 圆柱连续 yaw 对称性 | 可选的 wrist 绕物体轴角度 |
| `ProbeResult.weight_signal_N` | 目标夹持力和有界 wrist 速度 |
| 实时接触/对象 pose | 闭合量、携带纠偏、下降终止和最终验收 |

所以必须避免一个常见误解：

> probe 并不直接输出 manipulation 的 wrist pose。heft 提供对象重量相关信号；
> manipulation 的 wrist pose 由对象 pose、固定目标和抓取几何模板算出。

---

## 8. Pose-conditioned manipulation：每个动作的 wrist → 手指两步

这是当前更安全、应优先理解的 manipulation 路径：
`pose_conditioned_short_can_pick_place`。它面向
`mass / short_can / Allegro / 直立圆柱 / 平桌 / 固定绝对放置区`。

### 8.1 抓取 wrist 候选如何生成

short can 的物体系抓取模板为：

```text
t_object_wrist = [0, -20 mm, +130 mm]
R_object_wrist = Rx(pi)
```

圆柱绕局部 z 轴 yaw 对称，因此采样 12 个 `theta`：

```text
T_world_wrist_grasp(theta)
  = T_world_object · Rz(theta) · T_object_wrist
```

每个候选派生出：

```text
pregrasp = grasp 沿对象局部 z 再上移 75 mm
staging  = 保持 pregrasp xy/姿态，把 wrist 提到高位
lift     = grasp 的世界 z 增加 130 mm
carry    = 固定目标下的同一抓取模板，z 保持在 lift 高度
```

候选先检查 carriage ctrl range、桌面边界和其他对象净空，再按当前 wrist 到 pregrasp
的距离、对称 yaw 转动量和执行器余量排序。它是解析变换和有限候选筛选，不是通用
motion planner。

### 8.2 phase-by-phase：wrist 如何走，手指如何动

| phase | wrist 位姿/动作从哪里来 | 手指操作 |
|---|---|---|
| `handoff` | reset 到请求 pose，或验证 live scene 与请求 pose 的误差 | 不生成新 hand 命令；随后进入受保护的 preshape |
| `preshape` | wrist 不动 | 16 维目标移动到 `q_preshape=grip_pose(0.10)`，全过程要求无预接触 |
| `staging translation` | 只取候选 staging 的 xyz，先在高位平移 | 保持 preshape |
| `staging rotation` | 再单独转到候选的 `Rx(pi)+yaw` | 保持 preshape，避免长手指在低位扫过物体 |
| `pregrasp` | 移到候选 `T_world_wrist_pregrasp` | 保持 preshape |
| `grasp pose` | 直线下降到 `T_world_wrist_grasp` | 仍保持 preshape，且禁止提前接触 |
| `contact_acquire` | wrist 保持 grasp pose | 沿 `q_preshape → q_contact → q_squeeze_limit` 闭合，达到合法 mf/th 接触和目标总法向力即停止 |
| `grip_regulate` | wrist 保持 | 连续 100 step 调力；至少 80 step 满足合法双指接触和力阈值 |
| `lift` | 移到候选 `T_world_wrist_lift` | 每 step 根据接触力微调 closure progress |
| `carry` | 移到固定目标上方 `T_world_wrist_carry` | 保持并调节夹持；丢失 mf/th 或过力即失败 |
| XY correction | 根据 `p_goal_xy - p_object_xy` 最多修正 wrist 两次 | 持续夹持调力 |
| `place_descent` | wrist-z 逐步下降；对象碰桌或底部进入 release-height gate 后停止，并回抬 2 mm 卸载位置误差 | 持续夹持，但允许目标物体接触桌面 |
| `release` | wrist 保持 | 把 Allegro kp 降到 0.1，16 指关节对称、缓慢地向 `q_open` 插值，直到连续确认所有 hand-object 接触消失 |
| `retreat` | wrist-z 最多增加 100 mm，并受 carriage `z_cmd<=0.10` 限制 | 完全保持 `q_open` |
| `final_verify` | wrist/hand 不再接触对象 | 检查三维位置误差、对象轴倾角、漂移、桌面支撑和完全松手 |

### 8.3 probe 如何改变手指和 wrist 控制

pose-conditioned top pinch 的目标总法向力为：

```text
F_target = clip(7.0 + 0.82 * weight_signal_N, 8.1, 10.7) N
```

wrist 最大平移速度为：

```text
v_wrist = clip(0.070 + 0.008 * weight_signal_N, 0.080, 0.106) m/s
```

这里的 `F_target` 是所有合法 hand-object contact 法向力幅值之和，不是每根手指各自
达到该值，也不是 wrist F/T 的目标。

对极轻对象，手指调节使用更窄、更缓的 deadband，减少“微滑 → 快速闭合 → 力尖峰”
振荡。执行不会因为 probe 估计更重就改变抓取几何模板；它主要提高夹持力目标并改变
有界的运动速度。

---

## 9. 旧 manipulation 路径与新路径不要混用

仓库仍保留 `short_can_pick_place` v1，但它是兼容路径：

```text
allegro_grasp_lift=0.09
full_hand_collisions=False
wrist_roll_limit_rad=0.9
```

它使用 elevated fixture 上的 side-wrap 模板。其 grasp wrist 不是由完整
`T_world_object` 候选计算，而是用当前对象位置和固定偏移：

```text
grasp_x = object_x
grasp_y = object_y + 20 mm
grasp_z = object_center_z - 414 mm
```

然后 wrist 抬升 35 mm、相对源位置沿 +y 搬运 120 mm。手指仍按 mf/th 模板闭合，并
由 heft 的质量信号决定抓力、速度及轻/重放置分支。

这条路径不能说明完整手部碰撞下的真实桌面抓取。新设计或对外解释应优先使用
pose-conditioned top-entry 路径；v1 只用于保留旧回归与对照。

---

## 10. 从入口看完整调用链

### 10.1 Probe

```text
ProbeHarness.execute(ProbeCommand(primitive, target))
→ run_probe()
→ poke() / heft() / shake() / slide()
→ 分阶段生成 wrist 与 hand 命令
→ 每 step 接触和碰撞 gate
→ ProbeResult
```

运行示例：

```bash
python -m examples.run_probe_demo --family stiffness --backend allegro --viewer
python -m examples.run_probe_demo --family mass --backend allegro --viewer
python -m examples.run_probe_demo --family fill --backend allegro --viewer
python -m examples.run_probe_demo --family material --backend allegro --viewer
```

### 10.2 Probe → pose-conditioned manipulation

```text
Allegro heft ProbeResult
+ ObjectPoseObservation(T_world_object)
+ FixedPlaceSpec(T_world_object_goal)
→ PoseConditionedShortCanController.plan()
→ 生成并筛选 wrist pose candidates
→ ManipulationPlan
→ controller.execute()
→ wrist 分段轨迹 + 16-DoF hand 接触闭环
→ ManipulationExecutionResult
```

运行示例：

```bash
python -m examples.run_pose_conditioned_pick_place --target 0 --viewer
```

默认示例的 `reset_to_requested_pose` 是仿真 fixture：它会 reset 并把对象设置到请求
pose。`--verify-live-pose` 才是不搬动物体、只核对当前 scene pose 的 handoff 语义。

---

## 11. 当前实现边界

1. wrist waypoint 已有明确物理语义，但执行它的是理想 carriage，不是机械臂。
2. `poke/slide` 的手指关节基本固定，主要控制自由度是 wrist；`heft/shake/manipulation`
   才使用在线手指闭合调节。
3. 当前 top pinch、偏移和 16 维 synergy 都是针对 demo short can/cup 手工确定的模板，
   不是任意物体抓取规划。
4. pose-conditioned planner 只做圆柱对称候选、执行器范围和保守净空筛选；直线路径遇到
   障碍会失败，不会自动绕障。
5. probe 与 manipulation 当前通常使用独立 scene，传递的是可信结果数据，不是 probe
   结束后的连续物理状态。
6. 从 wrist pose 接入 Franka 时，需要新增 FK/IK、连续关节解、机械臂碰撞、时间参数化
   和接触控制后端；不能把 carriage actuator 数值直接当作 Franka 关节命令。

## 12. 对应代码

- `allegro_probe/primitives.py`：四个 probe 原语、wrist 插值、top-pinch 和指尖闭环。
- `allegro_probe/pose_manipulation.py`：物体 pose、固定目标、抓取模板和 wrist 候选生成。
- `allegro_probe/manipulation.py`：16-DoF hand 模板、plan builder 和 pick/place executor。
- `allegro_probe/scene.py`：理想 6-DoF carriage、Allegro 16 个 actuator、传感器与接触分类。
- `examples/run_probe_demo.py`：独立运行四类 probe。
- `examples/run_pose_conditioned_pick_place.py`：heft 数据到固定绝对位置 pick/place 的完整示例。

协议细节见 [probe_protocol_v2.md](../0710/probe_protocol_v2.md)；ProbeResult 到
manipulation 的字段和旧 v1 链路见
[probe_to_manipulation_dataflow.md](../0710/probe_to_manipulation_dataflow.md)；
pose-conditioned 路径见
[learning_free_pose_pick_place.md](../0710/learning_free_pose_pick_place.md)。
