# Probe execution protocol v2（本地实现）

本文冻结当前 `AllegroProbe` 的 v1 四原语执行协议。命名中的 `v2` 指 feature/schema
和控制协议已经从旧的固定腕部轨迹升级，并不扩大 benchmark family：仍只有
`poke / heft / shake / slide`。本仓库仍是执行层；任务生成、belief、标定、评分、
TSR/PAU 和 leaderboard 属于 `ProbeBench`。

## 1. 版本化入口与结果

公共入口仍是：

```python
result = ProbeHarness(backend).execute(
    ProbeCommand(
        primitive="heft",
        target=1,
        mode="unsupported_micro_lift",  # 省略时取该 primitive 的唯一默认 mode
    )
)
```

固定标识为：

```text
protocol_id   = probebench.probe.v1
feature_schema = allegro_probe.features.v2
```

四个 canonical mode：

| primitive | mode | 物理语义 |
|---|---|---|
| `poke` | `normal_force_ramp` | 指定 target 上的受控法向压入 |
| `heft` | `unsupported_micro_lift` | 物体真实脱离外部支撑后的重量测量 |
| `shake` | `unsupported_micro_shake` | 脱离支撑后的单轴小幅动态响应 |
| `slide` | `round_trip_force_control` | 恒定 preload 下往返切向滑动 |

`supported_nudge` 不能返回 `m_est_kg`，`supported_tilt` 也没有当前协议所需的 pivot
几何和支撑 baseline，因此两者会在命令 admission 阶段被拒绝，不会自动 fallback。

`ProbeResult` 额外携带 `mode`、`protocol_id`、`feature_schema` 和
`sensor_profile_id`。数值 `features` 只放可供下游消费的数值；字符串身份不混入
`Dict[str, float]`。控制失败时，权威物理 feature 置零，失败原因留在
`valid/controller_status/violations/quality/raw_summary`。

## 2. `poke`：Allegro 真正用食指指腹

两个 backend 共享“法向压入”语义，但 embodiment 和标定不同：

| backend | effector | 控制传感器 | 默认目标/硬上限 |
|---|---|---|---|
| reference | `central_probe` | `probe_touch + probe_force` | 3 N / 10 N |
| allegro | `ff_tip` | `ff_tip_touch + wrist F/T` | 0.8 N / 1.0 N |

Allegro 路径为：

```text
高位设置食指 preshape
→ 高位翻腕 Rx(pi)
→ 用实时 ff_tip site 相对 wrist 的位置对齐 target 顶部上方 40 mm
→ guarded descent（每次 0.15 mm，随后仿真 3 step）
→ 连续 20 step 接触身份 gate
→ ff_tip_touch PI-like 小步法向闭环
→ 200 ms hold 诊断
→ 保持翻腕姿态垂直退 60 mm，确认 touch=0
→ 高位回正腕部并张手
```

唯一合法 target 接触 geom 是：

```text
ff_tip_fingertip_collision
```

其他手指、食指 distal/medial/proximal、掌面、桌面、邻物和中央 probe 接触均无效。
Allegro stiffness scene 的红色中央 probe 在 XML 编译时即透明且关闭碰撞；运行时不
切换 collision mask。为避免 Menagerie 默认软接触产生毫米级假穿透，stiffness target
和食指 tip 使用显式 `<pair>`：`solref="0.004 1"`、
`solimp="0.95 0.995 0.001"`；协议穿透上限为 0.5 mm。

当前 stiffness 对象只有一个 spring-damper 压缩自由度。因此 `hold_force_ratio` 只是
控制稳定性诊断，不能声称是材料 stress relaxation；在增加独立粘弹真值与正式加载—
卸载协议之前，不输出伪造的 relaxation/hysteresis 标签。

## 3. `heft`：实际物体高度闭环

旧实现把 `wrist z += 25/130 mm` 当作 lift；不同抓持间隙会让物体实际只升几毫米或
升得过高。当前停止量改为 `object_center_pos(target)[2]`：

```text
target object lift      8 mm
tolerance               ±1.5 mm
max wrist travel        35 mm（安全/超时上限，不是目标高度）
max upward speed        20 mm/s
support-free dwell      120 ms
target-band dwell       80 ms
static measurement      200 ms
```

闭环只允许继续向上或保持，从不为追踪目标主动下压：

```python
object_lift = object_center_z - lift_start_center_z
if object_lift < target - tolerance:
    wrist_z_cmd += max_speed * dt

support_free = not (table_contact or pedestal_contact)
acquired = (
    support_free 持续达到 120 ms
    and object_lift 在目标带内持续达到 80 ms
    and opposing grasp 持续合法
)
```

最终还要求：无 table/pedestal 重接触、合法对向接触仍在、相对 wrist 平移漂移不超过
3.5 mm、旋转漂移不超过 5°、穿透和所有全局 collision gate 通过。

重量 baseline 有明确的物理时机：抓住但物体仍由桌面/窄 pedestal 支撑时采
`supported_baseline_wrist_force`；抬升静态段的受力减去它，再把 wrist-frame force
旋转到世界系并投影到重力轴：

```text
weight_signal_N = |median((R_world_wrist · ΔF_wrist)_z)|
m_est_kg        = weight_signal_N / 9.81
```

绝对质量仍有 backend bias，必须分别标定。

### Cleanup 不再依赖 10 mm 阈值

8 mm micro-lift 天然小于旧的 `lift_distance >= 10 mm` cleanup 判断。当前只要满足任一
状态：`lift_started`、`ever_support_free`、当前物体不受支撑，就必须执行 guarded
place：回零姿态、下降到源支撑、连续确认支撑、张手、垂直退场。即使 lift gate 中途
失败，也不能在空中直接松手。

## 4. `shake`：几何净空、双 baseline 和实际角度 lock-in

`shake` 继承全部 heft gate，并只允许 `container_sealed=True` 的 track。canonical
激励为 tilt 单轴 3°、3 Hz；yaw 必须作为未来独立窗口，不能与 tilt 同频混合后仍声称
存在唯一 gain/phase。

`container_sealed` 在当前低阶 proxy 中是协议准入 metadata；仿真没有开口、泄漏或
自由液面物理，不能把这个布尔值解释成已经模拟并验证了密封性能。

### 4.1 净空

静态 8 mm 是物体几何中心的 nominal micro-lift，不等于倾转后的实际底缘净空。
规划阶段先按 commanded 峰值 `A` 计算半径 `r`、半高 `h` 的保守中心抬升：

```text
nominal_sweep_bound = r sin(|A|) + h (1 - cos(A))
nominal_center_lift = 8 mm + nominal_sweep_bound + 0.5 mm
```

当前 cup 和 3° 对应约 10.34 mm。控制目标再加一份 1.5 mm tolerance 和 1.5 mm
动态下沉预留，使 canonical 中心目标约为 13.34 mm；物体中心抬升硬上限仍为
15 mm，超过时拒绝参数或使执行 invalid，不静默截断。
该 planning 值写入 `planned_shake_center_lift_m`；旧的
`required_shake_clearance_m` 仅作为兼容 alias，不能解释成实时底缘净空。

上式只用于 nominal planning，不能作为运行时安全证据。top-pinch 中容器会相对 wrist
旋转，因此 `wt_pos` 不是容器姿态。逐 step 的权威 gate 直接遍历 target 的外部碰撞
geom，用实时 `geom_xpos/geom_xmat` 求各 box/cylinder 在世界 z 方向的支撑函数：

```text
z_lowest = min_g (geom_center_z - vertical_extent(g))
bottom_clearance = z_lowest - source_table_or_pedestal_top_z
require bottom_clearance >= 1.5 mm
```

1.5 mm 是该 protocol_id 的硬下限；调用方可以提高门限以构造更保守的执行/负例，
不能在同一协议下调低它。

同时从 `object_quat` 记录容器自身 z 轴倾角及保守中心高度需求，但它只是诊断；真正的
validity 由实际最低碰撞几何体决定。当前五个 seed、三个 Allegro 内容候选的实测全程
最低净空为约 2.02–3.53 mm。quality 分别保存
`shake_min_bottom_clearance_m`、`shake_minimum_bottom_clearance_gate_m` 和两者差值
`shake_min_geometric_margin_m`，不再报告基于腕角的假裕量。

lift 后先进入独立的 `height_stabilization`：继续读取物体中心高度，以不超过
20 mm/s 的速度仅向上补偿，并要求在 0.25 mm 带内稳定 80 ms；累计腕部位移仍受
35 mm 行程硬上限约束。随后冻结 wrist z，再采 dynamic baseline、执行完整正弦窗口并
回零。这样 lock-in 仍是单一 commanded tilt 输入；动态相对旋转 gate 为 6.1°（在
6° 设计界限上保留 0.1° 求解器余量）；动态窗口内若实时净空不足则直接
invalid，不允许边晃边调 z 后仍把结果标成 `tilt_to_wrist_torque_y`。
`shake_extra_wrist_correction_m` 仅记录 stabilization 阶段的追加补偿。

### 4.2 动态 baseline 和波形

lift 后静止 200 ms，另采：

```text
lifted_dynamic_baseline_wrist_force
lifted_dynamic_baseline_wrist_torque
```

它只用于动态差分，不能替换 heft 的支撑中 baseline。波形为：

```text
0.25 cycle smooth ramp-in
→ 2 个完整 analysis cycles
→ 0.25 cycle smooth ramp-out
→ command exact zero
→ 120 ms post-zero check
```

`duration` 用来推导整数 analysis cycle 数；结果同时记录 resolved cycle 和真实 analysis
duration。

### 4.3 相对位姿 gate

世界坐标差 `object_pos - wrist_pos` 会随 wrist 倾转而变化，不能当 slip。实际计算：

```text
p_rel = R_world_wrist^T (p_world_object_center - p_world_wrist)
R_rel = R_world_wrist^T R_world_object
```

动态全程和回零后要求相对平移漂移不超过 5 mm、旋转漂移不超过 6°。5 mm 是当前
full-collision Allegro fixed-content 回归的明确 gate（观测最大约 4.52 mm），所有实测值
仍写入 `quality`，并没有从审计中隐藏。

### 4.4 Lock-in feature

只对完整 analysis cycles，使用实际 `wt_pos`，而不是命令角度。令：

```text
Cθ = 2/N Σ (θ - mean(θ)) exp(-j 2πft)
Cτ = 2/N Σ (τy - mean(τy)) exp(-j 2πft)
H  = Cτ / Cθ
```

输出：

```text
dynamic_torque_gain_Nm_per_rad = |H|
dynamic_phase_lag_rad          = angle(H)
dynamic_lockin_snr_db
angle_tracking_ratio           = |Cθ| / commanded_amplitude
post_zero_ringdown_rms_y_Nm
```

实际角度 tracking ratio 必须至少 0.75，角度 SNR 至少 15 dB。低 torque SNR 本身不使
probe 无效，因为 fixed content 的真实动态响应可以很低。

这里刻意叫 `dynamic_torque_gain`，不叫 `slosh_gain`：单次读数仍含手、夹爪、容器
刚体和 carriage 的传递响应。只有 ProbeBench 用相同 backend/mode/schema 的 locked
content 标定作复数差分后，才有资格命名纯 slosh gain。旧 `fill_proxy/slosh_proxy`
仅为兼容 alias。

## 5. `slide`：往返而非单程

`round_trip_force_control` 不再使用中央 probe。Allegro 以
`ff_tip/ff_tip_touch`、reference 以专用左指腹 pad/touch 建立 0.6 N preload；两者都
通过 wrist-z 闭环维持法向触觉读数，再由 wrist-x 以 10 mm/s 执行
`start → end → start` 两段 20 mm 路径。验收使用实际 fingertip site 位移而非命令
插值。material scene 中中央 probe 透明、禁碰且 `wp=0`。有效条件包括：

```text
outbound completion >= 0.95
return completion   >= 0.95
return error        <= one-way distance 的 10%
target contact fraction >= 0.80
无超力、持续失联、邻物/环境碰撞
目标平面位移 <= 3 mm
```

法向力来自指腹 touch，切向力来自接触前 baseline-corrected wrist F/T。摩擦估计仅使用
每条腿 10%–90% 的准匀速中段，排除起步、换向与端点伺服瞬态。每个 target 的显式
`condim=3` 指腹接触 pair 使用其 `friction_mu`；结果保留合并 `friction_ratio`，同时
给出 outbound/return 比值和方向不对称诊断。两后端 sensor profile 分别为：

```text
sim.allegro_ff_tip_touch+wrist_ft.v1
sim.reference_left_slide_touch+wrist_ft.v1
```

## 6. 等质量内容物 proxy

默认 `make_demo_scene("fill")` 生成 `track="content_mobility"`。三个对象的外壳、
总质量、fill ratio、内部质量、静态质心和 `slosh_range_m` 相同：

| class | 内部 body | f_n | damping ratio |
|---|---|---:|---:|
| fixed | 保留同质量 child body，但无 joint（真正 welded） | 2.4 Hz metadata | 1.20 |
| damped | 2D slide mass | 2.4 Hz | 1.20 |
| mobile | 2D slide mass | 2.4 Hz | 0.12 |

移动质量的 MuJoCo 参数由物理参数生成：

```text
k = m (2π f_n)^2
c = 2 ζ m (2π f_n)
```

`fixed` 不再用旧的“最小 1 mm joint range”近似；其隐藏质量真实焊接，因此 fixed 和
mobile 仍保持相同静态质量与 CoM。`track="fill_ratio"` 仍可显式生成，但候选总质量也
保持一致，避免仅凭 heft 解题。

## 7. ProbeBench adapter 边界

adapter 位于 sibling `ProbeBench/probebench/execution/`，AllegroProbe 不反向依赖
benchmark。它负责：

- ProbeBench 的 g→kg、N/mm→N/m、shape 和 track 转换；
- `reference/allegro` backend capability；
- 只声明等质量三候选 `content_mobility/shake`；尚未通过可分性校准的
  `fill_ratio` 不进入 capability，也不把 fill/heft 偷换 family；
- Allegro fingertip-touch 与 reference probe-touch 两种 `poke` 都只声明 T-Full；
  T-Force 仅含 wrist/actuator F/T 与 proprioception，不得读取触觉皮肤；
- stiffness 仅接收 80–1400 N/m 的 box，mass 仅接收 0.08–0.62 kg 的
  cylinder/short_can；标准任务越界或 shape 不匹配时在创建 backend 前拒绝，不缩放；
- invalid result 不向 belief 暴露零值/半成品 feature；
- calibration key 包含
  `track/tier/primitive/mode/protocol/backend/schema/sensor_profile/content_proxy`；
- 未版本化的数值参数 override 明确拒绝，不能复用 canonical calibration；
- `contact_energy=None`，PAU 暂时只使用显式的 time proxy，不伪造零机械功。

原来的 `PRIMITIVE_FEATURE` 仅是 native legacy fallback；content mobility 的权威 route
是 `dynamic_torque_gain_Nm_per_rad`。adapter 尚未替换 ProbeBench 的 episode env；它是
可注入执行器边界，完整 env/belief/cache migration 需在 benchmark 仓库另行完成。

## 8. 当前验收矩阵

回归至少覆盖：

```text
2 backend × 5 seed × 4 primitive × 3 target
Allegro fingertip poke 仅 ff_tip 接触及 <0.5 mm penetration
8 mm 实际高度 micro-lift + support-loss dwell
等质量 fixed/damped/mobile 动态可分
shake 回零、支撑重接触和相对位姿 gate
round-trip slide 两腿完成
material 中央 probe 零接触，且仅声明的 slide 指腹 geom 可接触 target
failed grasp / max wrist travel / partial collision / neighbour collision 负例
invalid feature 不进入 ProbeBench trusted observation
T-Force 拒绝 Allegro fingertip poke
```

seed 当前主要改变物性排列，不等价于位姿、摩擦、传感 bias、solver 或关节误差的 domain
randomization；这些仍是后续验收扩展，而不是本文声称已经解决的部分。
