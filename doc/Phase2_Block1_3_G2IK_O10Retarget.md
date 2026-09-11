# Phase 2 · Block 1+3 执行手顺：G2 双臂 IK + O10 手重定向（纯离线）

> **目标**：离线打通「人手位姿/骨架 → G2 双臂 14 关节角 + O10 双手各 10 主动关节角」，用 Meshcat 可视化验证。**零硬件、零仿真、零 sudo、零新依赖**。
> **决策已定**：A=无真机走仿真 · B=Isaac Sim · C=先做 Block1+3。
> **分工不变**：我写手顺 + 只读诊断；你执行（本阶段全是 `tv` 环境内的只读/离线运行，无系统变更）。
> **前置**：Phase 1 已跑通；`tv` 环境已含 `pinocchio 3.1.0 / casadi / meshcat / dex_retargeting / rerun`。

---

## 0. 环境与全局约定

- **conda 环境**：`conda activate tv`（找不到就用全路径 `conda activate /home/amit/DATA/envs/tv`）。
- **运行目录**：Block1 的 IK 测试在 `teleop/` 下跑；Block3 的重定向测试也在 `teleop/` 下跑（与 Phase 1 一致，保证 `../assets` 相对路径成立）。
- **防污染**：若之前 `source` 过 `G2_Robot/app/env.sh`，先 `unset PYTHONPATH LD_LIBRARY_PATH` 再 `conda activate tv`（本阶段用不到 GDK）。
- **机型选定**：本手顺以 **`G2_t2_crsB_o10_t2`**（洛石臂）为准 ✅。4 个变体都已生成，切换只改下表 URDF 路径：
  | 变体 | URDF |
  |---|---|
  | **G2_t2_crsB（本手顺用 ✅）** | `/opt/workspace/G2_Robot/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf` |
  | G2_t2_crs（天机臂） | `.../urdf/G2_t2_crs/G2_t2_crs_o10_t2.urdf` |
  | G2_t2v2_crsB | `.../urdf/G2_t2v2_crsB/G2_t2v2_crsB_o10_t2.urdf` |
  | G2_t2v2_crs | `.../urdf/G2_t2v2_crs/G2_t2v2_crs_o10_t2.urdf` |
  > **crsB vs crs 实测**：可动关节(62)、双臂关节名、手臂限位、O10 双手结构**完全一致**；仅末端几何差 5mm（`arm_l_end_joint` origin x：crsB=0.09 / crs=0.095）。故换臂型**只改 URDF 路径**，锁定表/target/EE 挂载关节名一律不动。
- **两个常量**（后面反复用到）：
  - `G2_URDF = /opt/workspace/G2_Robot/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf`
  - `G2_PKG_DIR = /opt/workspace/G2_Robot`　← pinocchio 用它解析 `package://genie_robot_description/meshes/...`（crsB 已验证 71 个 mesh 全部命中）

---

## 1. 摸底结论速查（写代码要用的事实，均已只读核实）

### 1.1 G2 关节全景（`G2_t2_crsB_o10_t2`，可动关节共 62）
| 分组 | 数量 | 处理 | 关节名 |
|---|---|---|---|
| **双臂（保留）** | **14** | IK 求解 | `idx21~27_arm_l_joint1~7`、`idx61~67_arm_r_joint1~7` |
| 躯干 body | 5 | 锁定 | `idx01~05_body_joint1~5` |
| 头 head | 3 | 锁定 | `idx11_head_joint1`、`idx12_head_joint2`、`idx13_head_joint3` |
| 底盘 chassis | 8 | 锁定 | `idx111/112_chassis_lwheel_front_joint1/2`、`idx131/132_chassis_rwheel_front_joint1/2`、`idx141/142_chassis_rwheel_rear_joint1/2`、`idx121/122_chassis_lwheel_rear_joint1/2` |
| 左 O10 手 | 16 | 锁定 | 见 1.2（含 5 个 mimic） |
| 右 O10 手 | 16 | 锁定 | 见 1.2（含 5 个 mimic） |

> 锁定 48、保留 14 → `reduced_robot.model.nq == 14`（Step 1.1 会验证）。
> `fixed` 关节（`idx20_arm_base_joint`、`arm_l/r_end_joint`、`idx31/71_hand_*_hand_base_joint`、`idx39/79_hand_*_middle_abad_joint`、各 `*_tip_joint`）**不进锁定表**（pinocchio 视为帧，非可动关节）。

### 1.2 O10 单手结构（左为例；右把 `l→r`、`idx3x/4x→idx7x/8x`）
- **10 个主动关节**（重定向 `target_joint_names` 就列这 10 个）：
  `idx31_hand_l_thumb_roll_joint`、`idx32_hand_l_thumb_abad_joint`、`idx33_hand_l_thumb_mcp_joint`、`idx36_hand_l_index_abad_joint`、`idx37_hand_l_index_pip_joint`、`idx39_hand_l_middle_pip_joint`、`idx41_hand_l_ring_abad_joint`、`idx42_hand_l_ring_pip_joint`、`idx44_hand_l_pinky_abad_joint`、`idx45_hand_l_pinky_pip_joint`
- **6 个 `<mimic>` 耦合关节**（自动跟随，**不列**进 target）：`idx34_thumb_pip`(×1.33 of mcp)、`idx35_thumb_dip`(×1.3 of mcp)、`idx38_index_dip`(×1.097 of index_pip)、`idx40_middle_dip`、`idx43_ring_dip`、`idx46_pinky_dip`
- **中指无 abad**（`idx39_hand_l_middle_abad_joint` 是 fixed）→ 中指只有 pip 1 个主动。
- **tip 链接**（重定向 task/finger_tip）：`hand_l_thumb_tip`、`hand_l_index_tip`、`hand_l_middle_tip`、`hand_l_ring_tip`、`hand_l_pinky_tip`
- **腕/base 链接**（重定向 wrist_link）：`hand_l_base_link`
- **利好**：`dex_retargeting` 原生解析 `<mimic>` 并挂 `MimicJointKinematicAdaptor`（默认 `ignore_mimic_joint=False`），6 个耦合关节自动联动，无需手动算。

---

# Block 1 — G2 双臂 IK（`G2_ArmIK`）

## Step 1.1 【只读冒烟】pinocchio 能否加载 G2 URDF + 锁定后是否 = 14 DoF

```bash
unset PYTHONPATH LD_LIBRARY_PATH
conda activate tv
cd /opt/workspace/xr_teleoperate/teleop
python - <<'PY'
import pinocchio as pin, numpy as np
URDF="/opt/workspace/G2_Robot/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf"
PKG="/opt/workspace/G2_Robot"
r=pin.RobotWrapper.BuildFromURDF(URDF,[PKG])
print("full  nq=",r.model.nq," nv=",r.model.nv," njoints=",r.model.njoints)   # 期望 nq=62
lock=["idx01_body_joint1","idx02_body_joint2","idx03_body_joint3","idx04_body_joint4","idx05_body_joint5",
 "idx11_head_joint1","idx12_head_joint2","idx13_head_joint3",
 "idx111_chassis_lwheel_front_joint1","idx112_chassis_lwheel_front_joint2",
 "idx131_chassis_rwheel_front_joint1","idx132_chassis_rwheel_front_joint2",
 "idx141_chassis_rwheel_rear_joint1","idx142_chassis_rwheel_rear_joint2",
 "idx121_chassis_lwheel_rear_joint1","idx122_chassis_lwheel_rear_joint2"]
# 双手 32 个可动关节
for s,base in (("l",31),("r",71)):
    pass
hands=[]
hands += [f"idx31_hand_l_thumb_roll_joint",f"idx32_hand_l_thumb_abad_joint",f"idx33_hand_l_thumb_mcp_joint",f"idx34_hand_l_thumb_pip_joint",f"idx35_hand_l_thumb_dip_joint",
          f"idx36_hand_l_index_abad_joint",f"idx37_hand_l_index_pip_joint",f"idx38_hand_l_index_dip_joint",
          f"idx39_hand_l_middle_pip_joint",f"idx40_hand_l_middle_dip_joint",
          f"idx41_hand_l_ring_abad_joint",f"idx42_hand_l_ring_pip_joint",f"idx43_hand_l_ring_dip_joint",
          f"idx44_hand_l_pinky_abad_joint",f"idx45_hand_l_pinky_pip_joint",f"idx46_hand_l_pinky_dip_joint"]
hands += [f"idx71_hand_r_thumb_roll_joint",f"idx72_hand_r_thumb_abad_joint",f"idx73_hand_r_thumb_mcp_joint",f"idx74_hand_r_thumb_pip_joint",f"idx75_hand_r_thumb_dip_joint",
          f"idx76_hand_r_index_abad_joint",f"idx77_hand_r_index_pip_joint",f"idx78_hand_r_index_dip_joint",
          f"idx79_hand_r_middle_pip_joint",f"idx80_hand_r_middle_dip_joint",
          f"idx81_hand_r_ring_abad_joint",f"idx82_hand_r_ring_pip_joint",f"idx83_hand_r_ring_dip_joint",
          f"idx84_hand_r_pinky_abad_joint",f"idx85_hand_r_pinky_pip_joint",f"idx86_hand_r_pinky_dip_joint"]
lock+=hands
print("lock count=",len(lock))   # 期望 48
rr=r.buildReducedRobot(lock, np.zeros(r.model.nq))
print("reduced nq=",rr.model.nq)   # 期望 14
for jn in ["idx27_arm_l_joint7","idx67_arm_r_joint7"]:
    print(jn,"-> jointId",rr.model.getJointId(jn))
PY
```
**验收点**：`full nq=62`、`lock count=48`、`reduced nq=14`、两个 wrist 关节 id 均 ≥0。
若 mesh 报错 → 检查 `G2_PKG_DIR` 是否传成列表 `[PKG]`；仍报错则见「附·风险 R1（去 mesh 方案）」。

## Step 1.2 在 `robot_arm_ik.py` 新增 `G2_ArmIK` 类

**做法（最小改动）**：把 `G1_29_ArmIK` 整个类（`robot_arm_ik.py` 第 18–310 行）复制一份，改名 `G2_ArmIK`，**只改下面 5 处**：

**① 路径 + 缓存名**（`__init__` 开头）：
```python
        self.cache_path = "g2_model_cache.pkl"
        # G2 用绝对路径 + package_dir，Unit_Test 不再区分路径
        self.urdf_path = '/opt/workspace/G2_Robot/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf'
        self.model_dir = '/opt/workspace/G2_Robot'
```
> 注意：`BuildFromURDF` 那行改成传**列表**：`pin.RobotWrapper.BuildFromURDF(self.urdf_path, [self.model_dir])`（G1 原本传的是字符串目录，G2 用 package:// 需要列表形式的 package_dirs）。

**② 锁定表**（把 `self.mixed_jointsToLockIDs = [...]` 整个换成）：
```python
            self.mixed_jointsToLockIDs = [
                # body 5
                "idx01_body_joint1","idx02_body_joint2","idx03_body_joint3","idx04_body_joint4","idx05_body_joint5",
                # head 3
                "idx11_head_joint1","idx12_head_joint2","idx13_head_joint3",
                # chassis 8
                "idx111_chassis_lwheel_front_joint1","idx112_chassis_lwheel_front_joint2",
                "idx131_chassis_rwheel_front_joint1","idx132_chassis_rwheel_front_joint2",
                "idx141_chassis_rwheel_rear_joint1","idx142_chassis_rwheel_rear_joint2",
                "idx121_chassis_lwheel_rear_joint1","idx122_chassis_lwheel_rear_joint2",
                # left O10 hand 16 (含 5 mimic；middle_abad 是 fixed 不列)
                "idx31_hand_l_thumb_roll_joint","idx32_hand_l_thumb_abad_joint","idx33_hand_l_thumb_mcp_joint",
                "idx34_hand_l_thumb_pip_joint","idx35_hand_l_thumb_dip_joint",
                "idx36_hand_l_index_abad_joint","idx37_hand_l_index_pip_joint","idx38_hand_l_index_dip_joint",
                "idx39_hand_l_middle_pip_joint","idx40_hand_l_middle_dip_joint",
                "idx41_hand_l_ring_abad_joint","idx42_hand_l_ring_pip_joint","idx43_hand_l_ring_dip_joint",
                "idx44_hand_l_pinky_abad_joint","idx45_hand_l_pinky_pip_joint","idx46_hand_l_pinky_dip_joint",
                # right O10 hand 16
                "idx71_hand_r_thumb_roll_joint","idx72_hand_r_thumb_abad_joint","idx73_hand_r_thumb_mcp_joint",
                "idx74_hand_r_thumb_pip_joint","idx75_hand_r_thumb_dip_joint",
                "idx76_hand_r_index_abad_joint","idx77_hand_r_index_pip_joint","idx78_hand_r_index_dip_joint",
                "idx79_hand_r_middle_pip_joint","idx80_hand_r_middle_dip_joint",
                "idx81_hand_r_ring_abad_joint","idx82_hand_r_ring_pip_joint","idx83_hand_r_ring_dip_joint",
                "idx84_hand_r_pinky_abad_joint","idx85_hand_r_pinky_pip_joint","idx86_hand_r_pinky_dip_joint",
            ]
```

**③ 末端帧 `L_ee`/`R_ee`**（`addFrame` 的 parent 必须是 joint，故挂手臂最后一个关节 `idx27/67_arm_*_joint7`；但 placement **直接复用 pinocchio 已算好的手掌基座帧 `hand_*_base_link` 的精确变换**，不手工猜 offset）：
```python
            # URDF 的 link 在 pinocchio 里本就是 BODY Frame；hand_*_base_link 的 parent_joint 就是 joint7，
            # 其 placement 已含 joint7→手掌基座的精确平移[0.108,0,0]与手掌朝向旋转（自动合并了中间 fixed 链）。
            _L_ee_ref = self.reduced_robot.model.frames[
                self.reduced_robot.model.getFrameId("hand_l_base_link")].placement
            _R_ee_ref = self.reduced_robot.model.frames[
                self.reduced_robot.model.getFrameId("hand_r_base_link")].placement
            # 若要把 EE 从腕基座再推到掌心/指尖，在此叠加本地位移（见 Step 1.4）：
            #   _L_ee_ref = _L_ee_ref * pin.SE3(np.eye(3), np.array([dx, dy, dz]))
            self.reduced_robot.model.addFrame(
                pin.Frame('L_ee', self.reduced_robot.model.getJointId('idx27_arm_l_joint7'),
                          _L_ee_ref, pin.FrameType.OP_FRAME))
            self.reduced_robot.model.addFrame(
                pin.Frame('R_ee', self.reduced_robot.model.getJointId('idx67_arm_r_joint7'),
                          _R_ee_ref, pin.FrameType.OP_FRAME))
```
> ⚠️ 别再用 `pin.SE3(np.eye(3), [0.09,0,0.018])` 这类手工 offset：① `arm_l_end_joint` 带旋转，0.018 的 z 实际被并进 x（正确平移是 **0.108**）；② 单位阵 `eye(3)` 丢掉手掌朝向，会让 IK 把 joint7 姿态而非手掌姿态对齐手腕。用 `hand_*_base_link.placement` 一步到位（实测 EE 与手掌基座世界位姿完全重合）。

**④ 平滑滤波器维度**：`WeightedMovingFilter(np.array([0.4,0.3,0.2,0.1]), 14)` —— G1_29 本就是 14，**不用改**（G2 双臂也是 14）。

**⑤ 可视化帧 id**（`if self.Visualization:` 里）：把 `frame_ids=[107, 108]` 改成 `frame_ids=[self.L_hand_id, self.R_hand_id]`（G2 帧号不同，用变量最稳）。

> `solve_ik`、casadi 建模、ipopt 选项、`save_cache/load_cache`、`scale_arms` **全部照抄不动**。

## Step 1.3 【离线验证】Meshcat 里看双臂跟随目标

在 `robot_arm_ik.py` 末尾 `if __name__ == "__main__":` 里，把实例化那行换成：
```python
    arm_ik = G2_ArmIK(Unit_Test=False, Visualization=True)
```
（`L_tf_target`/`R_tf_target` 的初始 `np.array([0.25, ±0.25, 0.1])` 可先沿用；G2 臂展与 G1 不同，若目标太远解不到，把 z 调到 `0.2~0.4`、y 调到 `±0.2` 附近。）

```bash
conda activate tv
cd /opt/workspace/xr_teleoperate/teleop
python robot_control/robot_arm_ik.py
# 终端出现 meshcat 地址（默认 http://127.0.0.1:7000/static/），浏览器打开；输入 s 回车开始
```
**验收点**：Meshcat 里出现 G2 双臂+双手模型；按 `s` 后两条末端目标坐标轴（`L_ee_target`/`R_ee_target`）做正弦运动，**双臂末端帧跟随目标**、关节不超限、无 NaN 报错。

## Step 1.4 EE 落点微调（可选：从手掌基座推到掌心/指尖）

Step 1.2③ 已让 `L_ee`/`R_ee` **精确锚在手掌基座 `hand_*_base_link`**（含正确姿态），IK 会让手掌根部对齐 Quest3 手腕 —— 通常已够用。
若想让抓握点落在**掌心或指尖**（而非腕部），不必手工猜数值，只在 Step 1.2③ 的 `_L_ee_ref` 上叠加**本地位移**：
```python
            _L_ee_ref = _L_ee_ref * pin.SE3(np.eye(3), np.array([dx, dy, dz]))   # 右乘 = 在 hand_base_link 自身坐标系下偏移
```
- `[dx,dy,dz]` 是 **hand_base_link 本地坐标系**下的偏移；哪个轴指向指尖，重跑后在 Meshcat 看 `L_ee` 帧三色轴朝向即可确定（沿该轴 +几厘米到掌心）。左右手分别调 `_L_ee_ref`/`_R_ee_ref`。
- **改完删缓存**：`rm -f teleop/g2_model_cache.pkl`（否则加载旧模型），重跑 Step 1.3。
- **验证**：q=0 时 `L_ee`/`R_ee` 世界坐标应与 `hand_*_base_link` 重合、左右对称（实测手掌基座 `[0.102, ±0.9615, 1.3415]`）。

**Block 1 验收**：目标帧与末端帧在合理误差内重合、双臂平滑跟随、`reduced nq=14`。

---

# Block 3 — O10 手部重定向（`o10.yml` + `HandType.O10`）

## Step 3.1 生成独立 O10 左/右手 URDF（从完整 URDF 抽子树、去 mesh）

重定向只需运动学，独立小手 URDF 最稳（避开 `package://` mesh 解析、加载快）。
把下面脚本存成 `teleop/robot_control/extract_o10_hand.py`：

```python
#!/usr/bin/env python3
# 从完整 G2 URDF 抽取独立 O10 左/右手 URDF：以 hand_X_base_link 为根，保留 joint(含 mimic/limit)，去掉 visual/collision。
import os, copy, xml.etree.ElementTree as ET

SRC = "/opt/workspace/G2_Robot/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf"
OUT_DIR = "/opt/workspace/xr_teleoperate/assets/o10_hand"

def extract(src, root_link, out_path):
    robot = ET.parse(src).getroot()
    joints = robot.findall("joint"); links = robot.findall("link")
    jtrip = [(j, j.find("parent").get("link"), j.find("child").get("link")) for j in joints]
    keep = {root_link}; changed = True
    while changed:                       # 从 root 沿 parent->child 扩散
        changed = False
        for j, p, c in jtrip:
            if p in keep and c not in keep:
                keep.add(c); changed = True
    keep_j = [j for j, p, c in jtrip if p in keep and c in keep]
    new = ET.Element("robot", {"name": root_link})
    for l in links:                      # 只留 link 名 + inertial，去掉 visual/collision/mesh
        if l.get("name") in keep:
            nl = ET.SubElement(new, "link", {"name": l.get("name")})
            ine = l.find("inertial")
            if ine is not None: nl.append(copy.deepcopy(ine))
    for j in keep_j: new.append(copy.deepcopy(j))
    ET.ElementTree(new).write(out_path, encoding="utf-8", xml_declaration=True)
    print(f"[ok] {out_path}: links={len(keep)} joints={len(keep_j)}")

if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    extract(SRC, "hand_l_base_link", f"{OUT_DIR}/o10_left.urdf")
    extract(SRC, "hand_r_base_link", f"{OUT_DIR}/o10_right.urdf")
```
```bash
conda activate tv
cd /opt/workspace/xr_teleoperate/teleop
python robot_control/extract_o10_hand.py
```
**验收点**：打印 `links≈13 joints=16`（每手 16 可动关节 + 若干 fixed tip 关节；link 数含 base+各指节+5 tip）。
快速自检能加载：
```bash
python - <<'PY'
from dex_retargeting import yourdfpy as urdf
for s in ("left","right"):
    u=urdf.URDF.load(f"/opt/workspace/xr_teleoperate/assets/o10_hand/o10_{s}.urdf", build_scene_graph=False)
    print(s,"joints=",len(u.joint_map),"links=",len(u.link_map),
          "mimic=",sum(1 for j in u.joint_map.values() if j.mimic is not None))
PY
```
期望每手 `mimic=6`（若你的变体不同，以实际为准）。

## Step 3.2 写 `assets/o10_hand/o10.yml`

新建 `/opt/workspace/xr_teleoperate/assets/o10_hand/o10.yml`（5 指模板参照 `inspire_hand.yml`；`type` 先用 `vector`，最直观易调，稳定后可改 `DexPilot`）：

```yaml
left:
  type: vector            # 可改 DexPilot
  urdf_path: o10_hand/o10_left.urdf
  target_joint_names:
    [
      "idx31_hand_l_thumb_roll_joint",
      "idx32_hand_l_thumb_abad_joint",
      "idx33_hand_l_thumb_mcp_joint",
      "idx36_hand_l_index_abad_joint",
      "idx37_hand_l_index_pip_joint",
      "idx39_hand_l_middle_pip_joint",
      "idx41_hand_l_ring_abad_joint",
      "idx42_hand_l_ring_pip_joint",
      "idx44_hand_l_pinky_abad_joint",
      "idx45_hand_l_pinky_pip_joint",
    ]
  # DexPilot 字段
  wrist_link_name: "hand_l_base_link"
  finger_tip_link_names: ["hand_l_thumb_tip","hand_l_index_tip","hand_l_middle_tip","hand_l_ring_tip","hand_l_pinky_tip"]
  target_link_human_indices_dexpilot: [[9,14,19,24,14,19,24,19,24,24,0,0,0,0,0],[4,4,4,4,9,9,9,14,14,19,4,9,14,19,24]]
  # vector 字段
  target_origin_link_names: ["hand_l_base_link","hand_l_base_link","hand_l_base_link","hand_l_base_link","hand_l_base_link"]
  target_task_link_names: ["hand_l_thumb_tip","hand_l_index_tip","hand_l_middle_tip","hand_l_ring_tip","hand_l_pinky_tip"]
  target_link_human_indices_vector: [[0,0,0,0,0],[4,9,14,19,24]]
  scaling_factor: 1.0     # O10 相对人手大小，先 1.0，Step 3.4 视情况调
  low_pass_alpha: 0.2
  # ignore_mimic_joint 默认 False → 6 个 mimic 自动联动，勿改

right:
  type: vector
  urdf_path: o10_hand/o10_right.urdf
  target_joint_names:
    [
      "idx71_hand_r_thumb_roll_joint",
      "idx72_hand_r_thumb_abad_joint",
      "idx73_hand_r_thumb_mcp_joint",
      "idx76_hand_r_index_abad_joint",
      "idx77_hand_r_index_pip_joint",
      "idx79_hand_r_middle_pip_joint",
      "idx81_hand_r_ring_abad_joint",
      "idx82_hand_r_ring_pip_joint",
      "idx84_hand_r_pinky_abad_joint",
      "idx85_hand_r_pinky_pip_joint",
    ]
  wrist_link_name: "hand_r_base_link"
  finger_tip_link_names: ["hand_r_thumb_tip","hand_r_index_tip","hand_r_middle_tip","hand_r_ring_tip","hand_r_pinky_tip"]
  target_link_human_indices_dexpilot: [[9,14,19,24,14,19,24,19,24,24,0,0,0,0,0],[4,4,4,4,9,9,9,14,14,19,4,9,14,19,24]]
  target_origin_link_names: ["hand_r_base_link","hand_r_base_link","hand_r_base_link","hand_r_base_link","hand_r_base_link"]
  target_task_link_names: ["hand_r_thumb_tip","hand_r_index_tip","hand_r_middle_tip","hand_r_ring_tip","hand_r_pinky_tip"]
  target_link_human_indices_vector: [[0,0,0,0,0],[4,9,14,19,24]]
  scaling_factor: 1.0
  low_pass_alpha: 0.2
```
> 人手 landmark 约定（25 点）：`0`=腕，指尖 `4/9/14/19/24`=拇指/食指/中指/无名指/小指 → 与 televuer 送来的 `left_hand_pos`(25×3) 一致，直接复用 Phase 1 的手数据。

## Step 3.3 `hand_retargeting.py` 加 O10 分支（3 处小改）

**① `HandType` 枚举**追加：
```python
    O10 = "../assets/o10_hand/o10.yml"
    O10_Unit_Test = "../../assets/o10_hand/o10.yml"
```
**② `__init__` 里 `set_default_urdf_dir` 分支**追加（放在 brainco 分支后）：
```python
        elif hand_type == HandType.O10:
            RetargetingConfig.set_default_urdf_dir('../assets')
        elif hand_type == HandType.O10_Unit_Test:
            RetargetingConfig.set_default_urdf_dir('../../assets')
```
**③ 硬件顺序映射**追加（放在 brainco 的 `elif` 后；O10 的 API 顺序暂用主动关节顺序，Block4 接 GDK/仿真时再对齐真实硬件序）：
```python
            elif hand_type == HandType.O10 or hand_type == HandType.O10_Unit_Test:
                self.left_o10_api_joint_names = [
                    'idx31_hand_l_thumb_roll_joint','idx32_hand_l_thumb_abad_joint','idx33_hand_l_thumb_mcp_joint',
                    'idx36_hand_l_index_abad_joint','idx37_hand_l_index_pip_joint','idx39_hand_l_middle_pip_joint',
                    'idx41_hand_l_ring_abad_joint','idx42_hand_l_ring_pip_joint',
                    'idx44_hand_l_pinky_abad_joint','idx45_hand_l_pinky_pip_joint']
                self.right_o10_api_joint_names = [
                    'idx71_hand_r_thumb_roll_joint','idx72_hand_r_thumb_abad_joint','idx73_hand_r_thumb_mcp_joint',
                    'idx76_hand_r_index_abad_joint','idx77_hand_r_index_pip_joint','idx79_hand_r_middle_pip_joint',
                    'idx81_hand_r_ring_abad_joint','idx82_hand_r_ring_pip_joint',
                    'idx84_hand_r_pinky_abad_joint','idx85_hand_r_pinky_pip_joint']
                self.left_dex_retargeting_to_hardware  = [self.left_retargeting_joint_names.index(n)  for n in self.left_o10_api_joint_names]
                self.right_dex_retargeting_to_hardware = [self.right_retargeting_joint_names.index(n) for n in self.right_o10_api_joint_names]
```

## Step 3.4 【离线验证】重定向冒烟 + 关节角输出

存成 `teleop/robot_control/test_o10_retarget.py`：
```python
import numpy as np
from hand_retargeting import HandRetargeting, HandType   # 在 teleop/robot_control 下运行

hr = HandRetargeting(HandType.O10)
print("left retarget joint_names =", hr.left_retargeting_joint_names)
print("left_indices shape =", hr.left_indices.shape)

def synth(open_hand: bool):
    # 合成 25x3 人手 landmark：腕在原点，5 指沿 +x 展开；open=伸直, fist=指尖收拢
    lm = np.zeros((25,3)); lm[0]=[0,0,0]
    tips  = [4,9,14,19,24]; 
    for k,tip in enumerate(tips):
        spread = (k-2)*0.03                       # y 方向指间展开
        reach  = 0.18 if open_hand else 0.06      # 伸直 vs 握拳
        for j in range(1,5):
            idx = tip-4+j-1 if tip!=4 else j      # 粗略填充各指关节
            if 0<=idx<25: lm[idx]=[reach*j/4, spread, 0]
        lm[tip]=[reach, spread, 0]
    return lm

for open_hand in (True, False):
    lm = synth(open_hand)
    ref = lm[hr.left_indices[1,:]] - lm[hr.left_indices[0,:]]
    q = hr.left_retargeting.retarget(ref)
    print(f"open={open_hand}  q_dim={q.shape}  q=", np.round(q,3))
```
```bash
conda activate tv
cd /opt/workspace/xr_teleoperate/teleop
python ./robot_control/test_o10_retarget.py
```
**验收点**：
1. 首次加载打印蓝色提示 **“Mimic joint adaptor enabled…”** → 说明 6 个耦合关节会自动联动 ✅
2. `left_retargeting_joint_names` 打印出实际返回顺序（**据此核对 Step 3.3③ 的 index 映射是否成立**；若 `retarget()` 返回含 mimic 的 16 维，则 `.index(name)` 仍能对上主动关节名，映射有效）。
3. `open=True` 与 `open=False` 两组 → `q` 数值明显不同、方向合理（握拳时弯曲关节角增大）。

> **⚠️ 帧约定（本合成测试测不出来、但实机必需）**：上面的合成 landmark 把手指放在 **+x** 方向，只是为了跑通链路；**它并不验证帧约定**。真实 Quest3/dex-retargeting 的人手骨架约定是手指指向 **-Y**（与仓库内能正常工作的 `inspire`/`dex3` 一致），而 **O10 URDF 四指在 q=0 时指向 +Z**。两者差一个固定旋转，若不修正，`retarget` 会把 `pip` 顶到上限→**手恒定握拳、不跟随张/合**。
> 修正位置在 Block4 的 `O10_Controller.control_process`：对 `ref_left/right_value` 右乘 `_HUMAN_TO_O10_ROT = [[-1,0,0],[0,0,-1],[0,-1,0]]`（把 -Y 旋进 +Z，左右手通用）。详见 `Phase2_Block2_4_5_G2_IsaacSim_Integration.md` Step 4。
> **如何自查**：比较各手 URDF 在 q=0 时 `wrist→middle_tip` 的方向——O10=`[0,0,1]`、inspire/dex3=`[0,-1,0]`；不一致就需要旋转。

> 合成 landmark 只为跑通链路；**更真实的验证**：接上 Quest3（Phase 1 已通）跑 `teleop_hand_and_arm.py`（Block4 接入后），或用 Phase 1 录一段 `tele_data.left_hand_pos` 回放喂给本脚本。

## Step 3.5（可选）Meshcat 可视化 O10 手指

用 `o10_left.urdf` + Step 3.4 得到的 `q`（主动 10 维）在 Meshcat 里 `display(q)`，肉眼确认张/合方向正确、mimic 指节联动。此步非必须，冒烟通过即可进下一阶段。

---

## 4. 集成预览（本阶段不做，仅明确接线点）

Block1+3 验证通过后，接入实时链路还需（属 Block2/4/5）：
- `--arm` 增加 `G2` 选项：`teleop_hand_and_arm.py` 第 79 行 choices 追加 `'G2'`，并 `from ...robot_arm_ik import G2_ArmIK`、`arm_ik=G2_ArmIK()`；配套 `robot_arm.py` 里的 **`G2_ArmController`（Block2）**。
- `--ee` 增加 `o10` 选项：仿 `Dex3_1_Controller` 写 **`O10_Controller`（Block4）**，内部 `HandRetargeting(HandType.O10)`，共享数组维度按 O10 主动 DoF（每手 10、双手 20）设 `dual_hand_state/action_array`。
- ⚠️ **无真机 → Block2/4 的下发对象是 Isaac Sim，不是 GDK**：GDK 的 `servo_control_arm_pos`/`move_ee_pos` 是驱动真实硬件的，纯仿真里无对象可连。teleop↔Isaac Sim 大概率沿用 Phase 1 的 **DDS 桥**（`ChannelFactoryInitialize(1)` + 自定义 G2 场景订阅）。这点在 Block5 手顺里定。

## 5. 验收清单 & 下一步

- [ ] **Block1**：Step1.1 `reduced nq=14`；Step1.3 Meshcat 双臂跟随；Step1.4 `L_ee/R_ee` 落在掌心。
- [ ] **Block3**：Step3.1 生成两只手 URDF（mimic=6）；Step3.4 冒烟打印 mimic adaptor 启用 + 开/合 q 有区分。
- 完成后 → **Block2（G2 臂控制器）+ Block4（O10 手控制器）+ Block5（Isaac Sim G2+O10 场景/DDS 桥）**，进入实时链路。

## 附 · 风险与回退

- **R1 mesh 加载失败**（Block1）：先确认传的是 `[G2_PKG_DIR]` 列表。仍不行 → 用 Step3.1 同款「去 visual/collision」思路，对整机 URDF 生成一份 kinematics-only 版给 IK 用（IK 不需 mesh；只是 Meshcat 看不到外形，可后续再补）。
- **R2 EE 帧轴/offset 不对**：Step1.4 调 offset；必要时改挂载关节（`idx27_arm_l_joint7` ↔ 直接用 `arm_l_end_link` 对应的 fixed 关节帧）。改完务必 `rm g2_model_cache.pkl`。
- **R3 mimic 方向反了 / 耦合异常**：临时在 `o10.yml` 加 `ignore_mimic_joint: true` 看主动关节是否合理，再回退；或核对 URDF `<mimic multiplier>` 符号。
- **R4 手指尺度不匹配**：调 `scaling_factor`（O10 比人手大就 >1，参考 inspire 用 1.20）。
- **R5 机型变体**：本手顺已用 **crsB（洛石臂）**。换 crs/t2v2 时，只需同步替换 Step1.1/1.2 的 URDF 路径 + Step3.1 脚本 `SRC`；实测 crs 与 crsB 的可动关节(62)/双臂关节名/手臂限位/O10 手结构完全一致，锁定表/target 表/EE 挂载关节名都不用改（仍建议重跑 Step1.1 核对 `nq=14`）。

> 本文所有关节名、链接名、mimic 关系、mesh 可达性均来自对 `G2_Robot` 与 `xr_teleoperate` 的只读核实，未改动任何源码。
