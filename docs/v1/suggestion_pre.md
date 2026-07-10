MuJoCo 灵巧手 Probe 控制调研与 1stea 改造建议
核心判断
基于你给出的 1stea 行为复核摘要，我对这个仓库的定位判断是：它现在更像一个“把 benchmark 任务先跑通的 v0 演示脚手架”，而不是一个已经具备物理可信接近、接触、夹持、探询和后续操作闭环的 probe 平台。问题不只是在 Allegro 本身难控，而是在 机器人本体、碰撞建模、传感器布置、低层控制、任务判定 这五层目前还没有被解耦；于是 grasp failure、contact spoofing、固定轨迹捷径、以及属性估计误差会混在一起。MuJoCo 的接触是严格按 geom、contact filter、solver 参数和传感器定义来算的，所以只要视觉几何和碰撞几何严重脱钩，或者 palm / base 长期“不参与碰撞”，就会天然出现你看到的“先穿过去、再突然抓住”的视觉效果。

如果目标是 先把 ProbeBench 的“主动触诊 benchmark”做对，我不建议现在把“完整 Allegro 单手完成所有 probe + manipulation”当成唯一主线。更稳妥的路线是把系统拆成两层：上层是 benchmark 与 probe 原语定义，下层是 可替换的执行器。你的设计文档本来就已经把任务族按隐藏物理属性组织，并且把 probe 原语明确成 poke / pinch / heft / shake / slide / tap / twist，又把传感配置分成 T-Full / T-Force / T-Acoustic 三档；这非常适合采用“先做参考 probe rig，再做 Allegro/Shadow 高阶轨道”的方案。
 

如果你现在必须在 MuJoCo 里选一个更方便的灵巧手基线，纯仿真 v1 更推荐 Shadow Hand MJCF；面向未来硬件一致性则继续保留 Allegro 作为长期目标。理由很直接：MuJoCo Menagerie 里的 Allegro 和 Shadow 都被归在 “Grippers & Hands” 类，而不是机械臂类，说明它们本质上都是手部资产，不自带完整的 reaching / arm-planning 栈；但 Shadow 的 Menagerie 版本额外加入了 forearm body、impratio=10 以改善 no-slip、并且显式强化了 hand geoms 的接触，而 Allegro 版本只做到“从 URDF 派生、加 base body、加 exclude、加 position actuators”。这意味着 Shadow 的 MJCF 对接触丰富任务略更友好，但两者都不是开箱即用的“完整操作机器人”。

对 1stea 当前实现的定位
按你给出的代码审计结论，1stea 在 mass / fill 上的抓取更像“固定腕部轨迹下降 + 关闭部分不需要的碰撞 + 分阶段闭合有效指节”的捷径，而不是“先可达地接近、再真实接触、再依据接触闭合、最后验证 grasp quality 后抬升”的物理流程。这个差异对 benchmark 非常关键，因为 MuJoCo 的碰撞选择与接触生成是先做 broad-phase / mid-phase / near-phase，再经过同体过滤、父子体过滤和 contype/conaffinity 过滤；一旦 hand 上大量视觉可见但不参与接触的 geom 被拿来“穿过”物体，后面再由少数 distal/fingertip geom 接管接触，仿真从视觉上就会表现成“虚化穿模后再实体化”。

这类实现为什么会直接污染 benchmark 结果？因为你的 benchmark 关心的不是“最终有没有抬起来”，而是“通过哪些 probe 读到了哪些有意义的物理信号，并据此决定操作参数”。你的文档已经把 episode 成功定义成 判别正确 ∧ 操作成功 ∧ 无安全违例，又把信息效率、过度触诊惩罚和属性估计精度单列为核心指标；这意味着 controller failure 不能伪装成 property estimation failure，反过来也不能把 accidental lift 记成成功。换句话说，1stea 这种 v0 实现可以保留为“动作脚本雏形”，但不能直接当作 benchmark 的官方 reference policy。

从这个角度说，你前面发现的那几个现象——mass/fill 里“没有有效抓取接触也能抬起”、slide 里“主要是下压不是水平摩擦”、以及 Allegro 本体“没有 wrist carriage / arm / IK / 现成 tactile arrays”——不是边缘 bug，而是同一个根问题的不同表现：当前环境把“动作发生了”误当成“probe 信息产生了”。这也是为什么后面我会强烈建议你把 probe controller 的有效性校验，和属性估计 / manipulation success 的评测逻辑彻底拆开。

公开仓库与工具链给出的启示
我把与你最相关的 MuJoCo 公开资产分成了六类。结论不是“哪个仓库最好”，而是“它们分别解决了哪一层问题”。

资源	它真正提供的东西	对你最有用的启示
MuJoCo Menagerie wonik_allegro	一个从公开 URDF 派生出的 Allegro Hand 简化 MJCF，README 写明它加了 base body、exclude 和 position-controlled actuators。
它是 hand asset，不是完整机器人；可以当几何与关节基线，但不能直接拿来做“到第 i 个物体上方的接近—预抓取—抬升”系统。
MuJoCo Menagerie shadow_hand	Shadow E3M5 的 MJCF，额外加入 forearm body、impratio=10、并“hardened the contacts”。
同样不是完整 arm-hand 栈，但更像一个“为接触任务调过参的 hand model”。如果你想先做纯 MuJoCo 参考基线，它比 Allegro 少踩一些接触坑。
Gymnasium-Robotics shadow_dexterous_hand	目录里同时有 manipulate_block.py / egg.py / pen.py 和对应的 *_touch_sensors.py 版本。
社区的常见做法是：模型层提供手，环境层决定是否暴露 touch sensors、如何定义任务和观测。也就是说，sensor/task variant 不该硬编码在 hand XML 里。
BODex	在 MuJoCo 上对 Shadow Hand 和 Allegro Hand 做双层优化式 grasp synthesis，报告了超过 75% 的仿真成功率，同时 penetration depth 和 contact distance 都压到 1 mm 以内。
稳定抓取不是靠“固定关节闭合曲线”碰运气，而是靠 预抓取姿态 + 受约束闭合 / QP / 优化。这对 heft、shake 的可靠性尤其关键。
MuJoCo MPC	一个基于 MuJoCo 的开源实时 MPC 框架，支持 iLQG、梯度下降和 Predictive Sampling。
如果你给 hand 增加 6-DoF wrist carriage，这个工具很适合做 预抓取位姿、受约束接近轨迹、微调末端姿态，而不是全靠手写轨迹。
DexJoCo	一个 2026 年的 MuJoCo 灵巧操作 benchmark/toolkit，包含 11 个更功能化的任务，并显式覆盖 tool-use、bimanual coordination、long-horizon。
它提醒你：双手协作本身就是一个独立难度轴。所以 twist 在你这里最好不要从 v1 开始就把“密封判断”和“双手协调”绑死。

这些仓库放在一起看，有一个很强的共同点：高质量项目通常分层。Menagerie 负责模型层，Gymnasium-Robotics 负责环境层，BODex 负责 grasp acquisition / contact quality，MJPC 负责轨迹与控制，DexJoCo 负责任务和评测层。你现在的 1stea 最大的问题恰恰是这几层被揉在一起了：碰撞遮罩在动作逻辑里改，接触有效性没有独立 gate，任务成功又和“抬起来了没有”混在一起。

你现在真正需要补的工程层
本体与接近层
Allegro 和 Shadow 在 Menagerie 里都只是手部资产，不是“完整机械臂 + 手腕 + 规划器”的系统；Model Gallery 里手和机械臂就是分栏展示的。Allegro README 也只说它是简化 MJCF、从 URDF 导出、加了 base body 和 position actuators，没有任何“到目标物体上方”或 IK 相关承诺。你的第一步，不是调 16 个手指关节，而是先给 hand 补一个 6-DoF wrist carriage。最简单的方法不是接真机械臂，而是在 MuJoCo 里给手掌前面加 3 个 slide + 3 个 hinge 的 wrist/probe carriage，或者用 mocap/weld 方式提供 task-space 入口；这样 approach / retreat / guarded descent / lift / tilt / yaw 都先变成腕部控制问题。

这一步补上之后，许多你现在在 Allegro 上遇到的困难会自动下降一个层级。因为 heft 的本质不是“16 个关节稳定接触控制”，而是先解决 末端姿态可达、目标上方预抓取、沿无碰路径接近、到接触点时触发闭合。这也正是我不建议你一开始就把所有 probe 都绑死在完整 Allegro anthropomorphic skill 上的原因：benchmark 问题的核心是“属性可辨”，不是“先把世界上最难的 dexterous grasp 不带 arm 地一次性解掉”。这个判断也和你文档里“按属性族组织 benchmark、把原语和 tier 分开”的设计是吻合的。
 

碰撞、接触与几何层
MuJoCo 的碰撞是 geom 级的，用户提供的三角网格在碰撞时默认会替换成 convex hull；只有少数特殊机制如 SDF plugin 才能绕开这个限制。MuJoCo 还支持多个接触点，但一般性的凸碰撞算法默认仍更偏向近似接触。对你的 benchmark 来说，这意味着一个非常现实的建模策略：除非任务真的需要细致几何，否则尽量用解析几何或“视觉 mesh + 简洁碰撞 proxy geom”的混合建模。像 mass、material、auth 大量样本完全可以用 box / capsule / cylinder + 参数变化完成；而像瓶盖螺纹、杯把、深凹槽、细颈瓶身这类对 concavity 敏感的东西，就不要指望“外观 mesh 直接拿来碰撞”会自然真实。

这里我特别不建议你继续沿用“视觉层保留大块 hand geom，碰撞层只让少数 distal links 生效”的做法。MuJoCo 文档明确写了 contype / conaffinity 决定接触是否生成；margin/gap 决定什么时候记为 inactive / active contact；solref/solimp 和 impratio 决定接触有多硬、摩擦方向约束有多强。Shadow 的 Menagerie 版本之所以专门加 impratio=10，就是为了让摩擦方向约束更硬、减少 slip。也就是说，你可以有视觉 mesh 和碰撞 proxy 的分离，但不能有“视觉上会挡住物体、物理上却长期透明”的分离。否则 benchmark 评到的是脚本漏洞，不是 probe policy。

如果你后面要做 slide/material，还有一个常被忽略的点：MuJoCo 对 geom 级 friction 不支持各向异性切向摩擦；如果你真想模拟“顺纹易滑、逆纹难滑”这种面内各向异性，文档建议用 explicit contact pair 而不是单 geom friction。对于 benchmark v1，这意味着多数 material 任务可先只做 各向同性摩擦系数 μ，不要过早上方向纹理。

传感器与信号层
你列的那组传感器在 MuJoCo 里是能对应上的，而且定义方式很清楚。touch 传感器挂在 site 上，输出是该 site 范围内所有接触的 法向力标量和；这非常适合你要的 probe_touch 或 fingertip touch，但它本身不含切向分量。force 和 torque 都是 3 轴 传感器，定义在 site 所在 child body 与 parent 之间的相互作用上，MuJoCo 文档还特别强调它们通常需要“一个 welded 到 parent 的 dummy body”来承载。framepos、jointpos、jointvel、actuatorfrc、jointactuatorfrc 也都有现成传感器定义。特别是 jointactuatorfrc，当一个 joint 上有多个 actuator 或存在重力补偿时，它比简单的 actuatorfrc 更接近你真正想要的“该 joint 上的总 actuator 广义力”。

这直接带来两个改造建议。第一，slide 必须是 同一个 probe site 上同时读 touch 和 force：前者做法向 preload 闭环，后者算切向/法向力比，否则你现在看到的就只会是“压下去几牛，然后刚开始水平动就 lost_contact”。第二，heft / shake / twist 这些基于 wrist F/T 的原语，必须做 baseline subtraction。因为 MuJoCo 的 force/torque 传感器测到的是 child-parent 间的总相互作用，里面会混进手自身重力、加速度项、外部扰动、桌面反力等效应。你前面说“重物没夹起来时 wrist_force 读到的不是重量而是动力学和桌面反力”，这正是物理定义决定的，不是偶然现象。

社区里成熟一些的做法，也确实倾向于把“有没有触觉”当作环境层配置，而不是让模型层同时承担所有语义。Gymnasium-Robotics 的 Shadow dexterous hand 目录里，把 block、egg、pen 操作都分别做了 touch-sensor 版和非 touch-sensor 版；这对你非常有启发：sensor stack 应该是可插拔的。你的 benchmark 完全可以保留一套统一任务定义，再提供 T-Full、T-Force、T-Acoustic 三条赛道。
 

控制与状态机层
你现在最缺的不是一个“更强策略网络”，而是一套 probe-aware 低层状态机。高质量 dexterous grasp 的公开工作，通常不会直接把“手张开—下降—闭合—抬起”写成单条固定曲线；像 BODex 这类 MuJoCo 抓取系统，更接近“先给定或优化预抓取，再在接触和 penetration 约束下做闭合”，于是既能把 penetration depth 压小，又能把 grasp success 顶上去。这个思想对 probe 比对普通抓取更重要，因为你的目标不是“总之先抓起来”，而是“用一条尽可能可重复、可比较、可校验的接触链路去产生信息”。

我建议你把所有 probe 和 manipulation 统一改写成同一个有限状态机框架：

approach → guarded descent → contact establish → contact quality gate → primitive execution → post-check → retreat / handover

这里最关键的是 contact quality gate。它不是一个装饰，而是 benchmark 有效性的生命线。比如 heft 必须要求“至少若干有效 fingertip 接触、相对位姿在短窗口内稳定、抬升前物体已脱离桌面支撑、且 palm 没有依赖被禁用碰撞体穿透维持姿态”；slide 必须要求“法向 preload 落在区间内、水平位移真的执行到目标长度，允许有限时间短暂失联但不能一失联就提前 exit”。只有满足这些 gate，后面读到的 force / torque / touch 才算 probe signal；否则应该直接记成 controller_fail，而不是算 attribute_estimation_fail。这个分层，也和你文档里“TSR、OPP、AEA、SR 分开记”的指标体系完全一致。

按 probe 原语给出的具体实现建议
适合直接用默认刚体几何和参数变化的 family
mass 和 material 最适合先做成 MuJoCo benchmark v1。原因是它们都不要求对象必须发生真实体变形。mass 可以用同外形、同外观的 rigid body，直接改 inertial 参数、质量和质心；material 可以先用同形体的 rigid geoms 改切向 friction、rolling / torsional friction，必要时再上 explicit contact pair 做更细定向摩擦。对这两类任务，默认 box / cylinder / capsule 加参数变化，已经能提供高信噪比 probe 信号，而且不会强迫你先解决 soft-body 或 thread-contact 难题。

auth 也可以在 v1 里先做成“外观几乎一致、内部参数不同”的刚体族。比如同样的“水果外壳”，真实体用较高密度、不同质心分布和更软的 pinch proxy，假体用更轻、更硬、tap 响应不同的代理。这样 pinch / tap / heft 都能给出可区分信号，但你不必一开始就去追求照片级材质或复杂软组织建模。这个思路与“通过机器人本体感受估计物体质量与柔软度”以及“通过 fingertip tactile 触发滑移来估计摩擦 / 动力学”的公开工作是一致的。

需要额外建模而不是仅改默认 geom 参数的 family
stiffness 不适合只靠 rigid geom 的 solref/solimp 伪装。MuJoCo 的 solref/solimp 决定的是接触约束的软硬和阻尼，不是物体的体材料弹性；如果你真想 benchmark “对象更软”，应使用 flexcomp / flex 之类的 deformable 建模，或者至少用一个 compliant proxy，例如“外壳刚体 + 内部 slider-spring-damper 压缩自由度”的 lumped model。MuJoCo 的 flexcomp 文档明确把它定义成用于 deformable entity 的宏，支持 young、poisson、damping 等弹性参数；它当然更慢，但对于你这种以物理属性为核心的 benchmark，stiffness 家族如果不用形变代理，物理意义会很弱。

seal 同样不适合只靠默认几何体凑。因为“密封/拧紧程度”本质是 相对转动约束 + 扭矩—转角曲线 问题，而不是静态接触问题。MuJoCo 的 mesh 碰撞默认会走 convex hull，这对螺纹、卡扣、盖沿这些细节都不友好；所以我更建议 v1 先做 fixture-assisted twist：瓶身固定在桌面 socket / clamp 里，probe 手只负责接触瓶盖并施扭矩，seal state 由一个显式 hinge / screw-like proxy joint 和扭矩阈值来编码。这样 twist family 仍然成立，但不会从第一天起就把任务难度抬成“双手高精协作 + 细几何接触 + 属性辨识”三重耦合。

fill 最容易把 benchmark 拖进“为了模拟流体而模拟流体”的泥潭。我建议 v1 不做 CFD 风格液体，而用 内部质量—质心代理：同样的杯子几何，内部放一个随姿态变化而产生力矩响应的 proxy mass / pendulum / slosh state，保证 heft 和小幅 shake/tilt 能分出“满 / 半满 / 近空”，但不要求你先把真实自由液面仿到很细。公开研究已经说明，仅用腕部或内在传感就能有效估计液体重量 / 倒出量；对 benchmark 来说，关键是信号可稳定辨识，而不是把液体 Navier–Stokes 也一并变成门槛。

各个 probe 原语应该怎么落地
poke 最简单，也最应该先做成 gold standard 原语。中央 probe site 下压，采用 法向力闭环，目标是达到某个 F_n 区间而不是某个位移；随后用 probe_framepos 计算压入量，用 probe_touch 读法向 aggregate，用 probe_force 区分是不是斜向受力。MuJoCo 的 touch 天生就是法向标量和，所以它非常适合作为刚度 / 接触建立的第一性信号；而 framepos 与 jointpos/jointvel 可以让你做位移—力曲线，而不是只看瞬时峰值。

slide 的关键不是“压到 2N 后横向挪一下”，而是 在法向 preload 闭环下完整执行切向轨迹。你当前版本通常 5–7 mm 后 lost_contact 提前结束，根因通常不是摩擦估计不会做，而是 preload 控制、接触容忍和轨迹终止条件太脆。实际实现时，建议用 probe_force 的法向分量做 PI 闭环维持 F_n，把 touch 作为“还在接触窗口内”的 binary/analog guard，再把切向位移按固定路径执行完；即便中间有短时失联，也不要立刻判 fail，而应给一个恢复窗口。最后用 F_t / F_n、滑移距离和 path completion ratio 共同定义 slide 有效性。

heft 不应从桌面上“直接用单手接管一个圆柱”，而应通过 物体设计 + 预抓取位姿 把难度改成可重复的单手任务。我的建议是：v1 的所有 mass / fill 物体都做成带平底、可侧向插指、或放在小 pedestal / shallow pocket 上的单手友好模型；先用腕部到预抓取位姿，再基于接触闭合，最后只抬离桌面 1–2 cm 做短时间重量估计，不要一开始就要求完整搬运。公开的 proprioception / tactile 研究和 BODex 都说明，只要 grasp acquisition 足够稳，物体的质量、惯性和摩擦参数是可以通过机器人内在信号辨识出来的；真正把信号毁掉的，往往不是估计器，而是前级 grasp controller 自己没抓稳。

shake 应建立在 heft-valid 之上，而不是一个独立的“拿起来就晃”。换句话说，shake 必须复用 heft 的 grasp quality gate：只有已经确认有足够 fingertip contact、物体脱离桌面且相对位姿稳定，才允许进入倾斜、微 yaw、低幅振荡。这样你后面用 wrist_force / wrist_torque 去读 fill-level 时，测到的才更可能是内容物重心移动，而不是“手指重新整理接触”和“腕部突然补偿”的混合信号。

pinch 则最适合作为真假和刚度的低风险探针：不要一上来做完整抓取，而是限定在标准 pinch pad 区域内，要求两指或三指以受限闭合量和受限力阈值接触，读 ftip{i}_touch、ftip{i}_force 和 jointactuatorfrc。这里我建议把“超过安全 pinch force 阈值”直接算 violation，而不是让 policy 自己在训练时慢慢摸索。你的文档已经把安全违例定义为任务失败的一部分，这样做也更契合 benchmark 的精神。
 

twist 是最不适合在 v1 里强上“纯双手协作”的原语。我更推荐你拆成两个 track：core track 用 fixture 固定瓶身、单手拧盖；advanced track 再做双手 body-cap 协同。这样做不是偷懒，而是为了保证 benchmark 的主变量仍然是“识别 seal state 并选择合适扭矩”，而不是“先把 bimanual dexterity 做出来”。DexJoCo 把 bimanual coordination 单列成能力维度，本身就说明双手协作值得单独评。

我给你的推荐架构与实施路线
最稳的方案，不是直接把 1stea 从 v0 修到“完美 Allegro”，而是按下面的顺序做系统重构。

首先，保留你现有 benchmark 的 任务族、probe 原语、tier 设计和信息效率指标，因为这些部分方向是对的。你的文档已经把 family、primitive、tier、TSR / IE / OPP / AEA 的关系说得很清楚，这恰好给了你一个“控制层和评测层分离”的天然接口。
 

其次，把执行器改成三层：

层级	你现在应该怎么做	为什么
Benchmark 层	保留 family、primitive、tier、指标定义不变	这是你的研究问题本体，不该被 Allegro 的当前难度拖垮。
Reference controller 层	先做 6-DoF wrist carriage + 可换 probe end-effector，其中 end-effector 可以是单中央探针、双指 pinch probe、简化三指或四指夹爪	这样 poke / slide / pinch / 部分 heft 能先形成高可信 reference policy。
Dexterous hand 层	把 Allegro 或 Shadow 作为高级轨道接入，要求共享同一套 task API、同一套传感器命名、同一套 probe validity gate	这样 benchmark 不会被某个 hand model 的局部 hack 绑死。

这个分层背后的逻辑，与公开工具链的分工是对齐的：Menagerie 负责 hand 资产，Gymnasium-Robotics 负责 tactile 与非 tactile 任务变体，MJPC 负责轨迹优化，DexJoCo 负责 task toolkit，BODex 负责 grasp acquisition。

最后，如果你问我“现在这一阶段最该做哪三件事”，我的答案会非常具体：

先补 wrist carriage 和 guarded approach。不把 hand 变成可在任务空间里接近目标的系统，后面所有 Allegro probe 都会变成“手指动作替代腕部运动”的畸形问题。
把传感器和有效性 gate 一次性补齐。至少要有中央 touch + force + framepos、wrist force + torque、fingertip touch、joint pos/vel、以及 jointactuatorfrc；并且每种 primitive 都有 valid / invalid 判定。MuJoCo 已经给了这些传感器原语，工程重点不在“能不能加”，而在“怎么把它们和 benchmark 逻辑切开”。
重写 object family，使 probe 与 manipulation 兼容。mass/material/auth 先上；stiffness 用 compliant proxy；fill 用 slosh proxy；seal 用 fixture-assisted cap；所有对象都做单手友好接近和稳定抓取，不要把“物理属性辨识”和“极限 dexterous picking from flat table”绑死。
如果只给一个最终建议，我会这么下结论：ProbeBench 的 v1 不应该以“把 Allegro 全面控好”为起点，而应该以“把 probe controller 做成可验证、可替换、可分层评测的 reference stack”为起点；Allegro 则作为与未来硬件一致的高级执行器，在这个 stack 上逐步接入。 对你眼下的进度，这比继续在 1stea 现有捷径实现上打补丁，成功率更高，也更接近一篇真正能立住的 benchmark 论文。
