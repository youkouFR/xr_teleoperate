# Phase 2 规划与工作量评估：Quest3 → 智元 G2 + O10

> **状态**：G2_Robot 摸底完成，本文是**规划 + 工作量评估 + 决策点**。待你拍板关键决策后，我再把选定路线展开成 Phase 1 那样的**分步执行手顺**。
> **前置**：Phase 1（Quest3 → xr_teleoperate → G1+Dex3 → Isaac Sim）已全链路跑通 ✅。
> **分工不变**：我写手顺 + 只读诊断，你执行（含 sudo / 环境变更）。

---

## 0. 摸底结论：G2_Robot 家底

### 0.1 模型资产（`genie_robot_description/`）
- **型号矩阵**：`body`(t2/t2v2/t0/lg) × `arm`(crs 天机谐波 / crsB 洛石 / acs / crsP) × `gripper`(o10_t2 / o12_t2 / omnipicker / dahuan 全系 / ctek90d)。`predefined.yml` 是唯一数据源。
- **URDF 已生成**（IK 可直接加载）：如 `urdf/G2_t2v2_crs/G2_t2v2_crs_o10_t2.urdf`、`urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf` 等（G2+O10 组合齐全，具体机型选定后确认对应文件）。
- **MJCF 已生成**（MuJoCo 可直接跑）：`mjcf/G2_t2_crs_o10_t2_flat.xml`、`G2_t2v2_crs_o10_t2_flat.xml` 等。
- **USD**：⚠️ 只有 `FromJunPu` 的 `G2_t2_crs + 军普夹爪` 版有 `model.usd`；**G2+O10 没有现成 USD**（走 Isaac Sim 需自己转）。
- **O10 手 xacro**：`xacro/gripper.o10_t2/{hand,thumb,finger,const}.xacro`。

### 0.2 G2 + O10 自由度（以 `G2_t2_crs_o10_t2` 为例，MJCF 共 76 关节）
| 部位 | 关节名 | DoF | 遥操作是否关注 |
|---|---|---|---|
| 底盘 | 4 轮 × 2 | 8 | 否（可选） |
| 躯干 | `idx01~05_body_joint1~5` | 5 | 可选（腰） |
| 头 | `idx11~13_head_joint1~3` | 3 | 可选（云台） |
| **左臂** | `idx21~27_arm_l_joint1~7` | **7** | ✅ 核心 |
| **右臂** | `idx61~67_arm_r_joint1~7` | **7** | ✅ 核心 |
| **左 O10 手** | `idx31~46_hand_l_*` | 16（6 个 equality 耦合 → 约 10 主动） | ✅ 核心 |
| **右 O10 手** | `idx71~86_hand_r_*` | 16（同上） | ✅ 核心 |

> O10 每手 16 关节：拇指 5(roll/abad/mcp/pip/dip) + 食指 3 + 中指 2 + 无名指 3 + 小指 3；其中 6 个 dip/pip 由 equality 耦合（欠驱动），**主动 DoF ≈ 10/手**（"O10"之名）。
> **遥操作核心 = 双臂 14 DoF + 双手 ~20 主动 DoF**（躯干/头/底盘可选纳入）。

### 0.3 GDK SDK（`app/gdk/` v2.6.3；文档 `G2_SDK文档_v2.3.4.md` 13528 行）
- **Python SDK**：`agibot_gdk`（编译扩展 `.cpython-310-x86_64-linux-gnu.so`）+ `dds.py`。⚠️ **绑定 Python 3.10 / x86_64**——正好与 teleop 的 `tv` 环境（py3.10）一致。
- **运行依赖 `app/env.sh`**：注入 `LD_LIBRARY_PATH`(733 个 .so)、`PYTHONPATH+=gdk/lib`、`AORTA_DISCOVERY_URI`；**真机在 `10.42.1.x` 网段**（与本机 wifi `192.168.10.x` 不同段）。
- **控制 API（实测自 `servo_control.py` / `servo_control_gripper.py`）**：
  ```python
  import agibot_gdk
  agibot_gdk.gdk_init(); robot = agibot_gdk.Robot()
  # 全身命名关节初始定位
  req = agibot_gdk.JointControlReq(); req.life_time=0.1
  req.joint_names=[...22个...]; req.joint_positions=[...]; req.joint_velocities=[0.3]*22
  robot.joint_control_request(req)
  # 手臂高频流式伺服（14=左7+右7，~100-200Hz）
  robot.servo_control_arm_pos([14个目标角], 2)
  # 末端/手
  cmd = agibot_gdk.JointStates(); cmd.group="left_tool"  # / "right_tool"
  cmd.target_type="omnipicker"  # O10 待确认对应字符串
  cmd.states=[agibot_gdk.JointState(position=...)]; cmd.nums=1
  robot.move_ee_pos(cmd)  # 返回 GDKRes.kSuccess
  # 状态回读
  robot.get_joint_states()  # {'states':[{'motor_position':...}]}
  robot.get_end_state()     # {'left_end_state':{'names','end_states':[{'position','velocity','err_code','enable'}]}, ...}
  ```
- **无对外的原生 VR 遥操作/重定向 API**（`rh_msgs_v4` 的 `TeleopPairing/RetargetConfig`、`genie_msgs` 的 `remote/VR手柄` 是内部消息，SDK 文档未开放）→ **重定向需自建，但可完全复用 xr_teleoperate 的 IK + dex-retargeting**。

---

## 1. 架构映射：Phase 1（G1）→ Phase 2（G2）

| 层 | Phase 1（现成，几乎没改） | Phase 2（要做） | 改动量 |
|---|---|---|---|
| Quest3 采集 | `televuer`（位姿 + 手部骨架） | **不变** | 0 |
| 手臂 IK | `robot_arm_ik.py`（G1_29） | **G2_ArmIK**：换 G2 URDF + 关节/EE 帧 + 限位 | 中 |
| 手重定向 | `dex-retargeting` + `unitree_dex3.yml` | 新增 **`o10.yml`** + O10 HandType | 中 |
| 手臂控制器 | `robot_arm.py` → unitree DDS | **G2 臂控制器** → GDK `servo_control_arm_pos` | 中 |
| 手控制器 | `robot_hand_unitree.py` → DDS | **O10 控制器** → GDK `move_ee_pos` | 中 |
| 通信层 | `unitree_sdk2py`（CycloneDDS channel） | **GDK**（`agibot_gdk` + `dds.py`，`env.sh`，10.42.1.x） | 中 |
| 仿真 / 本体 | `unitree_sim_isaaclab`（Isaac，含图像/状态/DDS 桥） | **无现成**：真机 或 自建 sim 后端 | **高** |

> 一句话：**上层（Quest3→IK→重定向）基本复用，下层（通信 + 本体/仿真）整套换成 G2/GDK。** 最重的是"仿真/本体"这一层，因为 G2 没有 unitree_sim_isaaclab 那样开箱即用的仿真桥。

---

## 2. 五大工作块 · 工作量评估

> 工作量为粗略人日估算，含调试；"依赖"列标明是否需要真机/仿真才能验证。

### Block 1 — G2 手臂 IK 类 `G2_ArmIK`
- **目标**：仿 `robot_arm_ik.py`，加载 G2+O10 URDF，输入 Quest3 头/手位姿 → 输出双臂 14 关节角。
- **参照**：`teleop/robot_control/robot_arm_ik.py`（G1_29 的 casadi/pinocchio IK）。
- **难点**：确定 base 帧、末端(EE)帧、关节顺序/限位、`arm_reference_mode`；G2 臂 7 DoF 与 G1 结构相近，可套用。
- **工作量**：中（~1-2 人日）。**依赖**：无（可用 `rerun_visualizer` 离线验证 IK 解）✅

### Block 3 — O10 手部重定向 `o10.yml`
- **目标**：让 `dex-retargeting` 把 Quest3 手部骨架重定向到 O10 关节。
- **参照**：`assets/unitree_hand/unitree_dex3.yml`（现成 dex3 配置）+ dex-retargeting 的 optimizer/vector 配置格式。
- **难点**：O10 关节命名/顺序/限位、欠驱动耦合（16 关节 vs ~10 主动）、tip 点对齐；需从 O10 URDF 提取。
- **工作量**：中（~1-2 人日）。**依赖**：无（离线可验证重定向输出）✅

### Block 2 — G2 手臂控制器（GDK）
- **目标**：仿 `robot_arm.py` 接口，把 IK 出的 14 关节角经 GDK `servo_control_arm_pos` 高频下发；`get_joint_states` 回读。
- **难点**：GDK 环境（`env.sh` / py3.10 / 10.42.1.x）、初始化与使能流程、控制频率与安全性（限速/急停）、joint_names 顺序对齐（注意 head 顺序是 1,3,2）。
- **工作量**：中（~1-2 人日）。**依赖**：需**真机或 GDK 兼容仿真**才能端到端验证 ⚠️

### Block 4 — O10 手控制器（GDK）
- **目标**：仿 `robot_hand_unitree.py`，把重定向出的 O10 关节角经 `move_ee_pos(group=left/right_tool, target_type=o10?, states=[...])` 下发；`get_end_state` 回读。
- **难点**：确认 O10 的 `target_type` 字符串与 `nums`/关节顺序（用 `get_end_state()['names']` 读实际顺序）。
- **工作量**：中（~1 人日）。**依赖**：需真机或仿真 ⚠️

### Block 5 — 本体 / 仿真后端（**最重，且取决于决策 A/B**）
- **路线① 真机**：Block2/4 直接驱动 G2；无需 sim。需硬件 + 10.42.1.x 网络 + 安全场地/急停。
- **路线② MuJoCo 仿真**：MJCF 现成（`G2_t2_crs_o10_t2_flat.xml`），起仿真快；但要写 **teleop↔MuJoCo 桥**（接收关节指令、回传状态、渲染相机图像给 teleimager/WebRTC）。~3-5 人日。
- **路线③ Isaac Sim 仿真**：延续 Phase 1 栈，但 **G2+O10 无 USD**，需 URDF→USD 转换 + 自建场景 + 配置控制器 + 接 teleimager 图像/DDS 桥。~1-2 周（最重）。

---

## 3. 关键决策点（已拍板 ✅ 2026-09-09）

> **决策记录**：**A = 无真机 → 走仿真** · **B = Isaac Sim（延续 Phase 1 栈）** · **C = 先做 Block1+3（纯软件离线）**。
> **架构提醒（因 A=无真机）**：GDK 的 `servo_control_arm_pos`/`move_ee_pos` 是驱动**真实硬件**的，纯仿真里没有对象可连。所以 Block2/4/5 的 teleop↔Isaac Sim 接口**大概率沿用 Phase 1 的 DDS 桥**（而非 GDK）；GDK 那条路等将来有真机再启用。

### 决策 A：目标本体——真机 G2 还是仿真？ → ✅ **仿真（无真机）**
- 有真机 → 可跳过 Block5，直接 teleop→GDK→真机（Block2/4 直接对真机验证）。
- **无真机 → 必须做 Block5（仿真后端）**。← 本项选定

### 决策 B（走仿真）：MuJoCo 还是 Isaac Sim？ → ✅ **Isaac Sim**
- MuJoCo（求快/先验证）：MJCF 现成，但要写 teleop↔MuJoCo 桥。
- **Isaac Sim（延续 Phase 1）**：图像/状态管线思路一致，但 G2+O10 要 URDF→USD + 建场景，最重。← 本项选定

### 决策 C：起步顺序 → ✅ **先做 Block1+3**
- **先做 Block 1 + Block 3（纯软件、零硬件/仿真依赖、可离线用 Meshcat/rerun 验证）**——把 "Quest3 手势 → G2 双臂关节角 + O10 关节角" 先算通、可视化验证。← 本项选定，手顺已产出
- 再做 Block 2 + 4（控制器，此时才需要仿真）。
- 最后处理 Block 5（Isaac Sim G2+O10 场景 / DDS 桥）。

---

## 4. 推荐路线（已确认）

> **先离线打通算法（Block1+3）→ 再接 GDK 控制（Block2+4）→ 本体/仿真按 A/B 定（Block5）**

理由：
1. Block1+3 不依赖任何硬件/仿真，**立刻能开工、能验证**，且是所有路线的公共前置。
2. 把最大的不确定性（GDK 通信、仿真后端）往后放，先用低风险的软件块建立信心和可复用的 IK/重定向。
3. 决策 A/B 可以等 Block1+3 做完再定，不阻塞起步。

---

## 5. 下一步
决策已定（A=仿真 / B=Isaac Sim / C=先 Block1+3），**Block1+3 详细执行手顺已产出** → [Phase2_Block1_3_G2IK_O10Retarget.md](./Phase2_Block1_3_G2IK_O10Retarget.md)（分步、含验证点/命令，全在 `tv` 环境离线跑，零硬件/仿真/sudo/新依赖）。
按手顺做完 Block1+3 并验收通过后，再展开 **Block2（G2 臂控制器）+ Block4（O10 手控制器）+ Block5（Isaac Sim G2+O10 场景 / DDS 桥）** 的手顺，进入实时链路。

> 备注：本文所有 GDK API、DoF、模型可用性均来自对 `G2_Robot` 的只读摸底（readme/AGENTS/SDK 文档/URDF/MJCF/示例代码），未改动任何文件。
