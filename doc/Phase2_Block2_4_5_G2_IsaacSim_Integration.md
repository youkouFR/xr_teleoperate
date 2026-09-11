# Phase 2 · Block2 / Block4 / Block5 —— 在 Isaac Sim 中用 Quest3 遥操作 AgiBot G2 + O10

> 承接 [`Phase2_Block1_3_G2IK_O10Retarget.md`](./Phase2_Block1_3_G2IK_O10Retarget.md)（已完成到 Step 3.4）。
> 本文把 **G2 双臂 IK（Block1）** 与 **O10 手重定向（Block3）** 接入实时链路，做到「像宇树 G1 那样」：
> **Quest3 → `teleop_hand_and_arm.py` → DDS(domain 1) → Isaac Sim（G2+O10 场景）→ 相机画面回传 Quest3**。

---

## 0. 本阶段决策（已与你确认锁定）

| # | 决策 | 选择 | 理由 |
|---|---|---|---|
| A | Sim 集成路线 | **独立轻量 Isaac Lab 脚本** | 无真机、目标是遥操作演示而非 RL 任务；不必新建整套 gym task 包（scene/obs/reward/termination），最轻、最稳、最少踩坑。teleop 侧改动与「复用完整框架」完全一致。 |
| B | VR 第一视角 | **需要**（相机回传 Quest3） | 与 Phase 1 体验一致；复用 `teleimager.image_server` + `tasks/common_observations/camera_state.py` 的共享内存相机管线。 |
| C | G2 资产导入源 | **URDF**（`G2_t2_crsB_o10_t2.urdf`）→ USD | 对 Isaac Lab `Articulation` 最标准；O10 的 6 个耦合关节走 URDF `<mimic>` 标签，命名与 Block1 pinocchio 建模同源、一致。 |

**分工**（延续既定方式）：本文是**手顺**，你按步骤执行；涉及系统/环境变更（含 `sudo`、conda、Isaac Sim GUI）由你亲手操作。跨环境诊断前先 `unset PYTHONPATH LD_LIBRARY_PATH`。

---

## 1. 全局架构与数据流

```
┌─────────────┐   WebXR(HTTPS/WSS:8012)   ┌──────────────────────────────────────────┐
│  Quest 3    │ ◀─────────────────────────▶│ teleop 端 (conda: tv, py3.10)             │
│  (VR 头显)   │   手/头 6D 位姿 + 手骨架     │  teleop/teleop_hand_and_arm.py            │
└─────────────┘                            │   ├─ G2_ArmIK.solve_ik()      (Block1 ✅) │
      ▲                                     │   ├─ G2_ArmController         (Block2 ◀── 本文)
      │ 第一视角画面                          │   ├─ HandRetargeting(O10)     (Block3 ✅) │
      │ (WebRTC/ZMQ)                         │   └─ O10_Controller           (Block4 ◀── 本文)
      │                                      └───────────────┬──────────────────────────┘
      │                                                      │ CycloneDDS  domain 1
      │        rt/lowcmd (LowCmd_,14臂)  ────────────────────┤
      │        rt/o10/{l,r}/cmd (HandCmd_,10指) ─────────────┤
      │        rt/lowstate (LowState_)   ◀───────────────────┤
      │        rt/o10/{l,r}/state (HandState_) ◀─────────────┘
      │                                                      ▼
      │                              ┌──────────────────────────────────────────────┐
      └──────────────────────────────│ sim 端 (conda: env_isaaclab, py3.11)          │
        teleimager.image_server      │  unitree_sim_isaaclab/g2_teleop_sim.py        │
        (共享内存 isaac_*_image_shm)   │   (Block5 ◀── 本文, 独立脚本)                  │
                                     │   ├─ InteractiveScene: G2 Articulation+3相机  │
                                     │   ├─ DDS 桥(直接收发, 无共享内存)              │
                                     │   ├─ 主循环: cmd→set_joint_position_target    │
                                     │   └─ camera_state.get_camera_image(shim)      │
                                     └──────────────────────────────────────────────┘
```

### 1.1 关节索引约定（teleop ↔ sim 的**唯一契约**，两侧必须一致）

> 无真机，索引由我们自定义。原则：**臂走 `LowCmd_` 的 `motor_cmd[0..13]`；每只手走 `HandCmd_` 的 `motor_cmd[0..9]`**。

**双臂（14）——`rt/lowcmd` / `rt/lowstate`，`LowCmd_.motor_cmd` 是 `array[35]`，只用前 14 槽：**

| motor_cmd 索引 | G2 关节名 | 说明 |
|---|---|---|
| 0..6 | `idx21_arm_l_joint1` … `idx27_arm_l_joint7` | 左臂 7 |
| 7..13 | `idx61_arm_r_joint1` … `idx67_arm_r_joint7` | 右臂 7 |
| 14..34 | （未用，置 0） | — |

**双手（每手 10 主动）——`rt/o10/{left,right}/{cmd,state}`，`HandCmd_.motor_cmd` 是动态 `sequence`：**

| motor_cmd 索引 | 左手关节名 | 右手关节名 |
|---|---|---|
| 0 | `idx31_hand_l_thumb_roll_joint` | `idx71_hand_r_thumb_roll_joint` |
| 1 | `idx32_hand_l_thumb_abad_joint` | `idx72_hand_r_thumb_abad_joint` |
| 2 | `idx33_hand_l_thumb_mcp_joint` | `idx73_hand_r_thumb_mcp_joint` |
| 3 | `idx36_hand_l_index_abad_joint` | `idx76_hand_r_index_abad_joint` |
| 4 | `idx37_hand_l_index_pip_joint` | `idx77_hand_r_index_pip_joint` |
| 5 | `idx39_hand_l_middle_pip_joint` | `idx79_hand_r_middle_pip_joint` |
| 6 | `idx41_hand_l_ring_abad_joint` | `idx81_hand_r_ring_abad_joint` |
| 7 | `idx42_hand_l_ring_pip_joint` | `idx82_hand_r_ring_pip_joint` |
| 8 | `idx44_hand_l_pinky_abad_joint` | `idx84_hand_r_pinky_abad_joint` |
| 9 | `idx45_hand_l_pinky_pip_joint` | `idx85_hand_r_pinky_pip_joint` |

> 该顺序 = Block3 `assets/o10_hand/o10.yml` 的 `target_joint_names` = `hand_retargeting.py` 的 `left/right_o10_api_joint_names`，三处已一致，无需改动。
> **6 个 mimic 关节/手**（`thumb_pip/thumb_dip/index_dip/middle_dip/ring_dip/pinky_dip`）不占 motor_cmd 槽，由 URDF `<mimic>` 在 USD 内自动联动（Step 5.7 验证；若导入未保留则用倍率手动驱动，见 Step 5.7 回退）。

---

## 2. 前置条件检查（开始前逐条确认）

```bash
# 2.1 Block1/Block3 已完成
ls /opt/workspace/xr_teleoperate/assets/o10_hand/          # 应有 o10_left.urdf o10_right.urdf o10.yml
grep -n "class G2_ArmIK" /opt/workspace/xr_teleoperate/teleop/robot_control/robot_arm_ik.py   # 应命中
grep -n "O10 = " /opt/workspace/xr_teleoperate/teleop/robot_control/hand_retargeting.py       # 应命中 HandType.O10

# 2.2 Phase 1 的 sim 仓库与 conda 环境就位
ls /home/amit/DATA/unitree_sim_isaaclab/sim_main.py        # 应存在（Phase1 跑通过）
conda env list                                             # 应含 tv 与 env_isaaclab

# 2.3 G2 URDF 与 mesh 源
ls /opt/workspace/G2_Robot/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf
ls /opt/workspace/G2_Robot/genie_robot_description/meshes/ | head
```

- [ ] 2.1 Block1（`G2_ArmIK`，reduced nq=14）+ Block3（O10 手 URDF/yml/retarget）均已验收通过。
- [ ] 2.2 `unitree_sim_isaaclab` 与 `env_isaaclab` 就位（Phase 1 已跑通 G1+Dex3）。
- [ ] 2.3 G2 URDF + meshes 可读。

> **本文所有路径基线**：teleop 端仓库 `/opt/workspace/xr_teleoperate`；sim 端仓库 `/home/amit/DATA/unitree_sim_isaaclab`；G2 资产 `/opt/workspace/G2_Robot/genie_robot_description`。若你的实际路径不同，全局替换即可。

---

## 3. Block2 —— G2 双臂控制器 `G2_ArmController`

**目标**：在 `teleop/robot_control/robot_arm.py` 里新增 `G2_ArmController`，**完全对照** `G1_29_ArmController`（同文件 `line 79`）改写：发布 `rt/lowcmd`、订阅 `rt/lowstate`，把 IK 解算的 14 维 `q/tauff` 按 §1.1 索引写进 `motor_cmd[0..13]`。

### Step 3.1 新增 G2 关节索引常量

在 `robot_arm.py` 顶部常量区（`G1_29_Num_Motors = 35` 附近）追加：

```python
# ===== AgiBot G2 (双臂 14 DoF, 无真机 / 纯仿真) =====
G2_Num_Arm_Motors = 14          # 仅双臂参与遥操作
G2_Num_Motors     = 35          # 复用 unitree_hg LowCmd_/LowState_ 的 array[35]

class G2_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G2_Num_Motors)]

class G2_JointArmIndex(IntEnum):
    # 左臂 → motor_cmd[0..6]
    kLeftArmJoint1 = 0
    kLeftArmJoint2 = 1
    kLeftArmJoint3 = 2
    kLeftArmJoint4 = 3
    kLeftArmJoint5 = 4
    kLeftArmJoint6 = 5
    kLeftArmJoint7 = 6
    # 右臂 → motor_cmd[7..13]
    kRightArmJoint1 = 7
    kRightArmJoint2 = 8
    kRightArmJoint3 = 9
    kRightArmJoint4 = 10
    kRightArmJoint5 = 11
    kRightArmJoint6 = 12
    kRightArmJoint7 = 13
```

> `MotorState` / `IntEnum` / `CRC` / `ChannelPublisher` / `ChannelSubscriber` / `DataBuffer` / `hg_LowCmd` / `hg_LowState` / `unitree_hg_msg_dds__LowCmd_` / `wait_for_dds` / `logger_mp` 都已在 `robot_arm.py` 顶部导入（G1 在用），无需重复 import。

### Step 3.2 新增 `G2_ArmController` 类

在 `robot_arm.py` 里（`G1_29_ArmController` 之后）追加下面的类。它是 `G1_29_ArmController` 的精简版：**去掉全身锁定**（sim 侧自己锁 body/head/wheels）、**臂索引改为 0..13**、其余（发布线程 250Hz、订阅线程、CRC、`simulation_mode` 跳过限速）与 G1 一致。

```python
class G2_ArmController:
    """AgiBot G2 双臂控制器（无真机 / 纯仿真）。
    对照 G1_29_ArmController 改写：发布 rt/lowcmd，订阅 rt/lowstate，
    仅驱动双臂 14 关节（motor_cmd[0..13]）。"""
    def __init__(self, motion_mode=False, simulation_mode=False):
        logger_mp.info("Initialize G2_ArmController...")
        self.q_target = np.zeros(G2_Num_Arm_Motors)
        self.tauff_target = np.zeros(G2_Num_Arm_Motors)
        self.motion_mode = motion_mode          # G2 无 motion/debug 之分，保留形参兼容主程序
        self.simulation_mode = simulation_mode
        # 臂增益（可后续调；肩/肘硬一点，腕软一点）
        self.kp_shoulder = 300.0; self.kd_shoulder = 3.0
        self.kp_elbow    = 300.0; self.kd_elbow    = 3.0
        self.kp_wrist    = 40.0;  self.kd_wrist    = 1.5
        self.control_dt  = 1.0 / 250.0

        # 无真机：固定走 debug 话题 rt/lowcmd（与 sim 侧订阅一致）
        self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.mode_machine = None
        self.lowstate_sub_ready = False

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()
        wait_for_dds(lambda: self.lowstate_sub_ready, "G2_ArmController")

        # 初始化 lowcmd 报文：仅双臂 14 槽上电（mode=1）+ 增益
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()
        self.all_motor_q = self.get_current_motor_q()
        for id in G2_JointArmIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in (5, 6, 12, 13):      # 腕关节（joint6/joint7）用软增益
                self.msg.motor_cmd[id].kp = self.kp_wrist
                self.msg.motor_cmd[id].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[id].kp = self.kp_shoulder
                self.msg.motor_cmd[id].kd = self.kd_shoulder
            self.msg.motor_cmd[id].q = self.all_motor_q[id]
        logger_mp.info("G2 arms initialized (motor_cmd[0..13]).")

        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()
        logger_mp.info("Initialize G2_ArmController OK!")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G2_LowState()
                for id in range(G2_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
                self.mode_machine = msg.mode_machine
                self.lowstate_sub_ready = True
            time.sleep(0.002)

    def _ctrl_motor_state(self):
        while True:
            start_time = time.time()
            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target
            # 纯仿真：直接下发（不做真机限速裁剪）
            cliped_arm_q_target = arm_q_target
            for idx, id in enumerate(G2_JointArmIndex):
                self.msg.motor_cmd[id].q   = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq  = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
            time.sleep(max(0, self.control_dt - (time.time() - start_time)))

    def ctrl_dual_arm(self, q_target, tauff_target):
        '''设置双臂 14 关节的目标 q 与 tauff。'''
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self):
        if self.mode_machine is None:
            raise RuntimeError("G2 low state is not ready.")
        return self.mode_machine

    def get_current_motor_q(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in range(G2_Num_Motors)])

    def get_current_dual_arm_q(self):
        '''返回双臂当前 q（14）。'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G2_JointArmIndex])

    def get_current_dual_arm_dq(self):
        '''返回双臂当前 dq（14）。'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in G2_JointArmIndex])

    def ctrl_dual_arm_go_home(self):
        '''双臂回零（q_target=0）。'''
        logger_mp.info("[G2_ArmController] ctrl_dual_arm_go_home start...")
        with self.ctrl_lock:
            self.q_target = np.zeros(G2_Num_Arm_Motors)
        for _ in range(100):
            if np.all(np.abs(self.get_current_dual_arm_q()) < 0.05):
                logger_mp.info("[G2_ArmController] both arms reached home.")
                break
            time.sleep(0.02)
```

> **对照要点**：`G1_29_ArmController` 用 `G1_29_JointArmIndex`（臂在 `motor_cmd[15..28]`）并锁定全身；G2 版把臂放到 `motor_cmd[0..13]`、不锁全身（sim 侧负责锁定 body/head/wheels）。`kTopicLowCommand_Debug="rt/lowcmd"`、`kTopicLowState="rt/lowstate"` 沿用文件已有常量。

- [ ] **Step 3.2 验收**：`python -c "import ast; ast.parse(open('/opt/workspace/xr_teleoperate/teleop/robot_control/robot_arm.py').read()); print('syntax OK')"` 通过（语法无误）。

---

## 4. Block4 —— O10 手控制器 `O10_Controller`

**目标**：新建 `teleop/robot_control/robot_hand_o10.py`，**完全对照** `robot_hand_unitree.py` 的 `Dex3_1_Controller`（`line 34`）改写：内部用 Block3 的 `HandRetargeting(HandType.O10)`，把 75 维手骨架重定向成每手 10 维 `q`，发布到 `rt/o10/{left,right}/cmd`（`HandCmd_`），订阅 `rt/o10/{left,right}/state`。

### Step 4.1 新建 `teleop/robot_control/robot_hand_o10.py`

```python
# teleop/robot_control/robot_hand_o10.py
# AgiBot O10 灵巧手控制器（无真机 / 纯仿真），对照 robot_hand_unitree.py 的 Dex3_1_Controller 改写。
import time
import numpy as np
from enum import IntEnum
from multiprocessing import Array, Process, Lock
import threading

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, HandCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandState_, unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__MotorCmd_

import os
import sys
parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)                        # 使 teleop.* 绝对导入可解析（对照 robot_hand_unitree.py:18-19）
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType

import logging_mp                                   # 全仓库统一约定：logging_mp 是已安装的 pip 包（对照 robot_hand_unitree.py:23-24）
logger_mp = logging_mp.getLogger(__name__)

O10_Num_Motors = 10                                  # 每手主动关节数
kTopicO10LeftCommand  = "rt/o10/left/cmd"
kTopicO10RightCommand = "rt/o10/right/cmd"
kTopicO10LeftState    = "rt/o10/left/state"
kTopicO10RightState   = "rt/o10/right/state"

class O10_Left_JointIndex(IntEnum):
    kThumbRoll=0; kThumbAbad=1; kThumbMcp=2; kIndexAbad=3; kIndexPip=4
    kMiddlePip=5; kRingAbad=6; kRingPip=7; kPinkyAbad=8; kPinkyPip=9

class O10_Right_JointIndex(IntEnum):
    kThumbRoll=0; kThumbAbad=1; kThumbMcp=2; kIndexAbad=3; kIndexPip=4
    kMiddlePip=5; kRingAbad=6; kRingPip=7; kPinkyAbad=8; kPinkyPip=9

# ⚠️ 帧约定修正（关键，不加则 O10 恒定握拳且不跟随）：O10 URDF 四指在 q=0 时指向 +Z，
#    而 dex-retargeting 的人手骨架约定与能正常工作的 inspire/dex3 一致（手指指向 -Y）。
#    不旋转就会把 pip 顶到上限(1.48)。ref @ _HUMAN_TO_O10_ROT 把【库 -Y】旋进【O10 +Z】（左右手通用）。
_HUMAN_TO_O10_ROT = np.array([[-1.0,  0.0,  0.0],
                              [ 0.0,  0.0, -1.0],
                              [ 0.0, -1.0,  0.0]])


class O10_Controller:
    def __init__(self, left_hand_array_in, right_hand_array_in, dual_hand_data_lock=None,
                 dual_hand_state_array_out=None, dual_hand_action_array_out=None,
                 fps=100.0, Unit_Test=False, simulation_mode=False, xr_motion_data_ready_in=None):
        """
        left/right_hand_array_in : [input] 25*3=75 维手骨架（来自 XR），multiprocessing.Array
        dual_hand_state_array_out: [output] 左(10)+右(10)=20 维手状态
        dual_hand_action_array_out:[output] 左(10)+右(10)=20 维手动作
        """
        logger_mp.info("Initialize O10_Controller...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.hand_retargeting = HandRetargeting(HandType.O10_Unit_Test if Unit_Test else HandType.O10)

        self.LeftHandCmb_publisher  = ChannelPublisher(kTopicO10LeftCommand, HandCmd_);  self.LeftHandCmb_publisher.Init()
        self.RightHandCmb_publisher = ChannelPublisher(kTopicO10RightCommand, HandCmd_); self.RightHandCmb_publisher.Init()
        self.LeftHandState_subscriber  = ChannelSubscriber(kTopicO10LeftState, HandState_);  self.LeftHandState_subscriber.Init()
        self.RightHandState_subscriber = ChannelSubscriber(kTopicO10RightState, HandState_); self.RightHandState_subscriber.Init()

        self.left_hand_state_array  = Array('d', O10_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', O10_Num_Motors, lock=True)

        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        hand_control_process = Process(
            target=self.control_process,
            args=(left_hand_array_in, right_hand_array_in, self.left_hand_state_array, self.right_hand_state_array,
                  dual_hand_data_lock, dual_hand_state_array_out, dual_hand_action_array_out, xr_motion_data_ready_in))
        hand_control_process.daemon = True
        hand_control_process.start()
        logger_mp.info("Initialize O10_Controller OK!")

    def _subscribe_hand_state(self):
        while True:
            left_hand_msg  = self.LeftHandState_subscriber.Read()
            right_hand_msg = self.RightHandState_subscriber.Read()
            if left_hand_msg is not None and right_hand_msg is not None:
                for idx, id in enumerate(O10_Left_JointIndex):
                    self.left_hand_state_array[idx] = left_hand_msg.motor_state[id].q
                for idx, id in enumerate(O10_Right_JointIndex):
                    self.right_hand_state_array[idx] = right_hand_msg.motor_state[id].q
            time.sleep(0.002)

    class _RIS_Mode:
        def __init__(self, id=0, status=0x01, timeout=0):
            self.motor_mode = 0
            self.id = id & 0x0F
            self.status = status & 0x07
            self.timeout = timeout & 0x01
        def _mode_to_uint8(self):
            self.motor_mode |= (self.id & 0x0F)
            self.motor_mode |= (self.status & 0x07) << 4
            self.motor_mode |= (self.timeout & 0x01) << 7
            return self.motor_mode

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        for idx, id in enumerate(O10_Left_JointIndex):
            self.left_msg.motor_cmd[id].q = left_q_target[idx]
        for idx, id in enumerate(O10_Right_JointIndex):
            self.right_msg.motor_cmd[id].q = right_q_target[idx]
        self.LeftHandCmb_publisher.Write(self.left_msg)
        self.RightHandCmb_publisher.Write(self.right_msg)

    def control_process(self, left_hand_array_in, right_hand_array_in, left_hand_state_array, right_hand_state_array,
                        dual_hand_data_lock=None, dual_hand_state_array_out=None, dual_hand_action_array_out=None,
                        xr_motion_data_ready_in=None):
        self.running = True
        left_q_target  = np.full(O10_Num_Motors, 0.0)
        right_q_target = np.full(O10_Num_Motors, 0.0)
        q = 0.0; dq = 0.0; tau = 0.0
        kp = 1.5; kd = 0.2                       # O10 手指增益（可后续调）

        self.left_msg  = unitree_hg_msg_dds__HandCmd_()
        # ⚠️ HandCmd_ 默认工厂只预填 7 个 motor_cmd（为 Dex3 设计）；O10 每手 10 个 → 必须先扩到 10，
        #     否则下面 motor_cmd[7..9] 会 IndexError。motor_cmd 是动态 sequence，可直接 append。
        while len(self.left_msg.motor_cmd) < O10_Num_Motors:
            self.left_msg.motor_cmd.append(unitree_hg_msg_dds__MotorCmd_())
        for id in O10_Left_JointIndex:
            self.left_msg.motor_cmd[id].mode = self._RIS_Mode(id=id, status=0x01)._mode_to_uint8()
            self.left_msg.motor_cmd[id].q=q; self.left_msg.motor_cmd[id].dq=dq
            self.left_msg.motor_cmd[id].tau=tau; self.left_msg.motor_cmd[id].kp=kp; self.left_msg.motor_cmd[id].kd=kd

        self.right_msg = unitree_hg_msg_dds__HandCmd_()
        while len(self.right_msg.motor_cmd) < O10_Num_Motors:
            self.right_msg.motor_cmd.append(unitree_hg_msg_dds__MotorCmd_())
        for id in O10_Right_JointIndex:
            self.right_msg.motor_cmd[id].mode = self._RIS_Mode(id=id, status=0x01)._mode_to_uint8()
            self.right_msg.motor_cmd[id].q=q; self.right_msg.motor_cmd[id].dq=dq
            self.right_msg.motor_cmd[id].tau=tau; self.right_msg.motor_cmd[id].kp=kp; self.right_msg.motor_cmd[id].kd=kd

        try:
            while self.running:
                start_time = time.time()
                with left_hand_array_in.get_lock():
                    left_hand_data  = np.array(left_hand_array_in[:]).reshape(25, 3).copy()
                with right_hand_array_in.get_lock():
                    right_hand_data = np.array(right_hand_array_in[:]).reshape(25, 3).copy()
                if xr_motion_data_ready_in is not None:
                    with xr_motion_data_ready_in.get_lock():
                        xr_motion_data_ready = xr_motion_data_ready_in.value
                else:
                    xr_motion_data_ready = True

                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if xr_motion_data_ready:
                    ref_left_value  = (left_hand_data[self.hand_retargeting.left_indices[1,:]]  - left_hand_data[self.hand_retargeting.left_indices[0,:]]) @ _HUMAN_TO_O10_ROT
                    ref_right_value = (right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]) @ _HUMAN_TO_O10_ROT
                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array_out and dual_hand_action_array_out:
                    with dual_hand_data_lock:
                        dual_hand_state_array_out[:]  = state_data
                        dual_hand_action_array_out[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)
                time.sleep(max(0, (1 / self.fps) - (time.time() - start_time)))
        finally:
            logger_mp.info("[O10_Controller] control_process stopped.")
```

> **对照要点**：与 `Dex3_1_Controller` 唯一实质差异是——每手 7→**10** 电机、话题 `rt/dex3/*`→`rt/o10/*`、`HandType.UNITREE_DEX3`→`HandType.O10`、去掉了「订阅到非零状态才继续」的阻塞等待（sim 起步时手状态可能全 0，避免卡死）。`left_indices/right_indices/left_retargeting/left_dex_retargeting_to_hardware` 均来自 Block3 的 `HandRetargeting`。
> **⚠️ 关键坑（上例已修正）**：`unitree_hg_msg_dds__HandCmd_()` / `HandState_()` 默认工厂**只预填 7 个** motor 元素（Dex3 用）。O10 每手 10 个，**发布侧必须先把 `motor_cmd` / `motor_state` 扩到 10**（`while len(...)<10: append(unitree_hg_msg_dds__MotorCmd_()/MotorState_())`），否则索引 `[7..9]` 直接 IndexError。sim 侧发布 `HandState_` 同理（见 Step 6.4 的 `G2SimBridge`）。
> **`logger_mp` 正确写法（已核实，全仓库一致）**：`teleop/utils/` 下**没有** `logging_utils.py`（别写 `from teleop.utils.logging_utils import logger_mp`，会 ModuleNotFoundError）。真实约定是 `import logging_mp` + `logger_mp = logging_mp.getLogger(__name__)`（`logging_mp` 是 tv 环境里已装的 pip 包，见 `robot_hand_unitree.py:23-24`、`robot_hand_inspire.py:11-12`、`hand_retargeting.py:5-6`）。上面 Step 4.1 代码块已按此写好。
> **⚠️ 帧约定坑（已修正，否则手会恒定握拳且不跟随）**：O10 URDF 四指在 q=0 时指向 **+Z**，而 dex-retargeting 的人手骨架约定与能正常工作的 `inspire`/`dex3` 一致——手指指向 **-Y**。两者差一个固定旋转；直接把人手向量喂给 O10 `retarget`，优化器会把 `pip` 关节顶到上限(1.48)，**张手/握拳输出几乎相同→看上去就是“死死握拳、不跟手势”**。修正：在 `control_process` 里对 `ref_left/right_value` 右乘 `_HUMAN_TO_O10_ROT`（把 -Y 旋进 +Z，左右手通用）。实测：张手→`pip≈0.07`、握拳→`pip≈0.44`，5 指均无饱和、可跟随。（排查方法：比较各手 URDF 在 q=0 时 `wrist→middle_tip` 的方向，O10=`[0,0,1]`、inspire/dex3=`[0,-1,0]`。）

### Step 4.2 冒烟：单独验证 O10_Controller 能起进程 + 发布

```bash
unset PYTHONPATH LD_LIBRARY_PATH
conda activate tv
cd /opt/workspace/xr_teleoperate/teleop     # ★ CWD 必须是 teleop/：hand_retargeting 用 ../assets 相对路径找 o10.yml
python - <<'PY'
import sys, os
sys.path.append(os.path.dirname(os.getcwd()))   # ★ 把仓库根加进 sys.path，import teleop.* 才成立
import time
from multiprocessing import Array, Lock
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
ChannelFactoryInitialize(1)
L = Array('d', 75, lock=True); R = Array('d', 75, lock=True)
st = Array('d', 20, lock=False); ac = Array('d', 20, lock=False)
from teleop.robot_control.robot_hand_o10 import O10_Controller
c = O10_Controller(L, R, Lock(), st, ac, simulation_mode=True)
time.sleep(1.0)
print("O10_Controller started OK; action_array len =", len(ac))
PY
```

> **为什么这么跑（你刚踩的坑）**：`ModuleNotFoundError: No module named 'teleop'` 的根因是——`teleop/` 下没有 `__init__.py`，它是**命名空间包**，只有当**仓库根** `/opt/workspace/xr_teleoperate` 在 `sys.path` 上时 `import teleop.*` 才成立；而 `hand_retargeting` 又用 `../assets/...` **相对 CWD** 找 `o10.yml`（`Path(hand_type.value).open()`），所以 **CWD 必须停在 `teleop/`**。两者靠「CWD=teleop/ + 手动把仓库根加进 sys.path」同时满足（`teleop_hand_and_arm.py:11-13` 就是这么做的，故主程序能跑）。

- [ ] **Step 4.2 验收**：打印 `Initialize O10_Controller OK!` 且 `action_array len = 20`，无 import/维度报错。（此时无 sim 订阅，`rt/o10/*/state` 收不到属正常。）

---

## 5. teleop 主程序接线（`teleop_hand_and_arm.py`）

**目标**：让 `--arm G2 --ee o10 --sim` 走通。改动点全部对照现有 `G1_29` / `dex3` 分支。

### Step 5.1 `--arm` / `--ee` choices 增加选项

- `line 79` 的 `--arm` choices 追加 `'G2'`：
  ```python
  parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'H2', 'R1_A5', 'R1_A7', 'G2'])
  ```
- `line 80` 的 `--ee` choices 追加 `'o10'`：
  ```python
  parser.add_argument('--ee', type=str, choices=['dex1', 'dex1_internal', 'dex3', 'inspire_ftp', 'inspire_dfx', 'brainco', 'o10'])
  ```

### Step 5.2 顶部 import 增加 G2

在 `line 17-18` 的 import 处补上 `G2_ArmController` 与 `G2_ArmIK`：

```python
from teleop.robot_control.robot_arm import G1_29_ArmController, G2_ArmController   # 追加 G2_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G2_ArmIK                # 追加 G2_ArmIK
```

> 按文件里实际的 import 行改写（可能是多行 `from ... import (...)`，把 `G2_ArmController`/`G2_ArmIK` 加进去即可）。

### Step 5.3 arm 分支增加 `G2`（`line 162` 的 `if args.arm == "G1_29":` 同级）

```python
        elif args.arm == "G2":
            arm_ik = G2_ArmIK()
            arm_ctrl = G2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
```

### Step 5.4 ee 分支增加 `o10`（`line 191` 的 `elif args.ee == "dex3":` 同级）

```python
        elif args.ee == "o10":
            from teleop.robot_control.robot_hand_o10 import O10_Controller
            left_hand_pos_array  = Array('d', 75, lock=True)     # [input] 手骨架
            right_hand_pos_array = Array('d', 75, lock=True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array  = Array('d', 20, lock=False)  # [output] 左10+右10
            dual_hand_action_array = Array('d', 20, lock=False)  # [output] 左10+右10
            hand_ctrl = O10_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock,
                                       dual_hand_state_array, dual_hand_action_array,
                                       simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
```

> 主循环里 `arm_ik.solve_ik` + `arm_ctrl.ctrl_dual_arm`（喂 `tele_data.left/right_wrist_pose`）确实是**机型无关**的通用逻辑，`G2` 自动复用、无需改动——这就是"机械臂随手腕动"的原因。
> ⚠️ **但手骨架的喂数不是自动适配的**：它被 `args.ee in (...)` 白名单挡住，必须把 `o10` 加进去，否则手骨架数组恒为全 0（见 Step 5.5）。

### Step 5.5 主循环手骨架喂数分支加入 `o10`（**关键，漏掉则 O10 手不动**）

`line 331` 的喂数守卫是一个 `args.ee` 白名单，默认**不含 `o10`**。若不改，`left/right_hand_pos_array` 永远全 0 → `O10_Controller` 重定向出全 0 → 发布的 `rt/o10/*/cmd` 全 0 → **O10 手完全不动**（而机械臂走的是另一条 `wrist_pose` 路径，不受影响，于是表现为"臂动手不动"）。

把 `o10` 追加进这个元组：

```python
            if args.ee in ("dex3", "inspire_ftp", "inspire_dfx", "brainco", "o10")  and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
```

> 判据：改动前 `--ee o10 --input-mode hand` 会落到 `else: pass`（`line 355-356`），手骨架不被填充；改动后在 Quest 3 握拳应能看到 O10 手指跟随。

- [ ] **Step 5 验收（不连 sim，仅查接线）**：
  ```bash
  unset PYTHONPATH LD_LIBRARY_PATH; conda activate tv
  cd /opt/workspace/xr_teleoperate/teleop
  python -c "import ast; ast.parse(open('teleop_hand_and_arm.py').read()); print('syntax OK')"
  python teleop_hand_and_arm.py --help | grep -E "\-\-arm|\-\-ee"   # choices 里应出现 G2 / o10
  ```

---

## 6. Block5 —— Isaac Sim 侧（独立轻量脚本 `g2_teleop_sim.py`）

> **路线 B**：不新建 gym task 包，而是写一个**独立脚本**，直接搭 `InteractiveScene`（G2 + 地面 + 灯 + 3 相机），
> 复用 `unitree_sim_isaaclab` 已有的 `teleimager.image_server`（相机回传）与 `tasks/common_observations/camera_state.py`（相机→共享内存）。
> **脚本放在 `unitree_sim_isaaclab/` 根目录**，这样能直接 `import teleimager`、`from tasks.common_observations.camera_state import ...`、`from robots.agibot import ...`、`from unitree_sdk2py...`。

### Step 6.1 URDF → USD（Isaac Sim URDF Importer）

**产物目标**：`/home/amit/DATA/unitree_sim_isaaclab/assets/robots/g2-o10-base-fix-usd/g2_o10.usd`（**Fix Base Link = 开**，与 G1 base_fix 一致）。

**方式一（推荐，GUI 最稳）**：
1. 启动 Isaac Sim（`env_isaaclab`）：`cd /home/amit/DATA/unitree_sim_isaaclab && conda activate env_isaaclab && ./isaac-sim.sh`（或你 Phase1 的启动方式）。
2. 菜单 `Window ▸ Importer ▸ URDF`。
3. 关键设置：
   - **Input File**：`/opt/workspace/G2_Robot/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf`
   - **Output Directory**：`/home/amit/DATA/unitree_sim_isaaclab/assets/robots/g2-o10-base-fix-usd`
   - **Fix Base Link**：✅ **勾选**（G2 底座固定，只动臂+手）
   - **Import Inertia Tensor**：✅；**Convex Decomposition**：碰撞用凸包（可选，先默认）
   - **Joint Drive Type**：`None`（增益我们在 `ArticulationCfg` 里给，避免和 USD 内置 drive 打架）
4. 点 `Import`。生成 `g2_o10.usd`。

**方式二（headless 脚本，API 随 Isaac Sim 版本略有差异，失败就用方式一）**：
```bash
unset PYTHONPATH LD_LIBRARY_PATH; conda activate env_isaaclab
cd /home/amit/DATA/unitree_sim_isaaclab
mkdir -p assets/robots/g2-o10-base-fix-usd
python - <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.kit.app
mgr = omni.kit.app.get_app().get_extension_manager()
mgr.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
import omni.kit.commands
from isaacsim.asset.importer.urdf import _urdf  # 若导入名不同，用 GUI 方式
import omni.kit.commands as kc
cfg = kc.execute("URDFCreateImportContext")[1] if False else None
# 直接用 Parse-and-Import（多数版本支持）：
result, prim = kc.execute(
    "URDFParseAndImportFile",
    urdf_path="/opt/workspace/G2_Robot/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB_o10_t2.urdf",
    import_config=_urdf.UrdfImportContext(),
    dest_path="/home/amit/DATA/unitree_sim_isaaclab/assets/robots/g2-o10-base-fix-usd/g2_o10.usd",
)
print("import result:", result, prim)
app.close()
PY
```
> 方式二里 `import_config` 的默认值需按版本设置 `fix_base=True`、`import_inertia_tensor=True`、`joint_drive_type=None`。**若 API 名对不上，别纠结，直接用方式一 GUI**（最省心）。

**⚠️ mesh 路径（`package://genie_robot_description/meshes/...`）解析**：
- URDF 在 `.../genie_robot_description/urdf/G2_t2_crsB/`，mesh 在 `.../genie_robot_description/meshes/`。Importer 一般会在 URDF 的**祖先目录**里找名为 `genie_robot_description` 的包目录 → 命中。
- 若导入报「mesh not found」：在 Importer 面板把 **Asset Root / ROS Package Path** 指到 `/opt/workspace/G2_Robot`（其下有 `genie_robot_description/`），或临时软链：
  ```bash
  ln -s /opt/workspace/G2_Robot/genie_robot_description /home/amit/DATA/unitree_sim_isaaclab/assets/genie_robot_description
  ```

- [ ] **Step 6.1 验收**：`ls -la /home/amit/DATA/unitree_sim_isaaclab/assets/robots/g2-o10-base-fix-usd/g2_o10.usd` 存在且 >1MB；在 Isaac Sim 里拖入该 USD 能看到完整 G2（底座+双臂+双 O10 手），底座固定不下坠。

### Step 6.2 资产探查（**必做**，用于确定关节名与相机父链接）

导入后的 USD 里，关节名/链接名以**实际导入结果为准**。跑下面脚本，把输出贴到你的笔记里，Step 6.3/6.4 要据此填链接名：

```bash
unset PYTHONPATH LD_LIBRARY_PATH; conda activate env_isaaclab
cd /home/amit/DATA/unitree_sim_isaaclab
python - <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import isaacsim.util.debug_draw._debug_draw as _dd  # noqa
from pxr import UsdPhysics, Usd
stage = Usd.Stage.Open("assets/robots/G2_t2_crsB_o10_t2/G2_t2_crsB_o10_t2.usd")
joints, links, cams = [], [], []
for p in stage.Traverse():
    n = p.GetName(); path = str(p.GetPath())
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        joints.append((n, path))
    if "camera" in n.lower():
        cams.append((n, path))
    if p.IsA(UsdPhysics.RigidBodyAPI) or n.endswith("_link") or "link" in n.lower():
        links.append(path)
print("==== JOINTS (%d) ====" % len(joints))
for n,pa in joints: print(f"  {n}\t{pa}")
print("==== CAMERA-ish prims ====")
for n,pa in cams: print(f"  {n}\t{pa}")
print("==== candidate head / hand_base links (grep 用) ====")
for pa in links:
    if any(k in pa.lower() for k in ["head", "hand_l_base", "hand_r_base", "camera", "arm_l_link7", "arm_r_link7"]):
        print("  ", pa)
app.close()
PY
```

**从输出里确认 3 件事**（记为 `<HEAD_LINK>` / `<L_HAND_CAM_LINK>` / `<R_HAND_CAM_LINK>`）：
- 头部相机父链接：形如 `head_link3` 或 URDF 里的 `idx13_head_...` 末端 link（G2 URDF 头部有 `head_link1..3`）。
- 左/右腕相机父链接：O10 手上有相机 mount（URDF 里见过 `idx31_hand_l_camera_joint` → 对应 `hand_l_camera_link` 之类）；若没有独立 camera link，就用 `hand_l_base_link` / `arm_l_link7`。
- 关节名是否与 §1.1 一致（`idx21_arm_l_joint1`…、`idx31_hand_l_thumb_roll_joint`…）。**若 USD 里关节名带了前缀/改名，以 USD 为准，并同步改 Step 6.3 的正则与 Step 6.4 的映射。**

- [ ] **Step 6.2 验收**：拿到 `<HEAD_LINK>`、`<L_HAND_CAM_LINK>`、`<R_HAND_CAM_LINK>` 三个链接名，且确认臂/手关节名与 §1.1 表一致。

### Step 6.3 机器人配置 `robots/agibot.py`（`ArticulationCfg`）

在 `unitree_sim_isaaclab/robots/` 下新建 `agibot.py`（对照 `robots/unitree.py` 的 `G129_CFG_WITH_DEX3_BASE_FIX`）：

```python
# robots/agibot.py
"""AgiBot G2 + O10 configuration (base fixed)."""
import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

project_root = os.environ.get("PROJECT_ROOT", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 双臂 14（正则匹配 idx21..27_arm_l_joint1..7 / idx61..67_arm_r_joint1..7）
_ARM_EXPR = ["idx2._arm_l_joint.", "idx6._arm_r_joint."]
# O10 每手 10 主动关节（mimic 6 关节不在此列，由 URDF <mimic> 联动）
_O10_ACTIVE = [
    "idx3._hand_l_thumb_roll_joint", "idx3._hand_l_thumb_abad_joint", "idx3._hand_l_thumb_mcp_joint",
    "idx3._hand_l_index_abad_joint", "idx3._hand_l_index_pip_joint", "idx3._hand_l_middle_pip_joint",
    "idx4._hand_l_ring_abad_joint", "idx4._hand_l_ring_pip_joint",
    "idx4._hand_l_pinky_abad_joint", "idx4._hand_l_pinky_pip_joint",
    "idx7._hand_r_thumb_roll_joint", "idx7._hand_r_thumb_abad_joint", "idx7._hand_r_thumb_mcp_joint",
    "idx7._hand_r_index_abad_joint", "idx7._hand_r_index_pip_joint", "idx7._hand_r_middle_pip_joint",
    "idx8._hand_r_ring_abad_joint", "idx8._hand_r_ring_pip_joint",
    "idx8._hand_r_pinky_abad_joint", "idx8._hand_r_pinky_pip_joint",
]
# O10 每手 6 个 mimic 耦合关节（URDF <mimic>，type=revolute）。若 Importer 把它们导成独立 revolute，
# 它们也是 DOF，必须有 actuator 覆盖，否则 Isaac Lab 报「joint 无 actuator」；主循环按倍率驱动（Step 6.4/6.6）。
# 若 Step 6.2 发现这些关节被 Importer 导成 fixed / 合并（不在关节列表里），则删掉本列表与下面的 hands_mimic 组。
_O10_MIMIC = [
    "idx34_hand_l_thumb_pip_joint", "idx35_hand_l_thumb_dip_joint",
    "idx38_hand_l_index_dip_joint", "idx40_hand_l_middle_dip_joint",
    "idx43_hand_l_ring_dip_joint",  "idx46_hand_l_pinky_dip_joint",
    "idx74_hand_r_thumb_pip_joint", "idx75_hand_r_thumb_dip_joint",
    "idx78_hand_r_index_dip_joint", "idx80_hand_r_middle_dip_joint",
    "idx83_hand_r_ring_dip_joint",  "idx86_hand_r_pinky_dip_joint",
]
# 其余（body/head/wheels）锁定：高刚度 + 0 速度限位，保持默认位姿
_LOCKED_EXPR = ["idx0._body_joint.", "idx1._head_joint.", "idx1.._chassis_.*"]

G2_CFG_WITH_O10_BASE_FIX = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/G2_t2_crsB_o10_t2/G2_t2_crsB_o10_t2.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False, retain_accelerations=True,
            linear_damping=0.0, angular_damping=0.0,
            max_linear_velocity=1000.0, max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),          # 底座固定，z 按 USD 实际离地高度微调
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "arms": ImplicitActuatorCfg(
            joint_names_expr=_ARM_EXPR,
            effort_limit=None, velocity_limit=None,
            stiffness={"idx2._arm_l_joint[1-5]": 300.0, "idx6._arm_r_joint[1-5]": 300.0,
                       "idx2._arm_l_joint[6-7]": 40.0,  "idx6._arm_r_joint[6-7]": 40.0},
            damping={"idx2._arm_l_joint[1-5]": 3.0, "idx6._arm_r_joint[1-5]": 3.0,
                     "idx2._arm_l_joint[6-7]": 1.5, "idx6._arm_r_joint[6-7]": 1.5},
            armature=None,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=_O10_ACTIVE,
            effort_limit=100.0, velocity_limit=50.0,
            stiffness={".*": 40.0}, damping={".*": 5.0}, armature={".*": 0.01},
        ),
        "hands_mimic": ImplicitActuatorCfg(          # 6×2 耦合关节：位置驱动，主循环按倍率喂目标
            joint_names_expr=_O10_MIMIC,
            effort_limit=50.0, velocity_limit=50.0,
            stiffness={".*": 40.0}, damping={".*": 5.0}, armature={".*": 0.01},
        ),
        "locked": ImplicitActuatorCfg(
            joint_names_expr=_LOCKED_EXPR,
            effort_limit=1000.0, velocity_limit=0.0,
            stiffness={".*": 10000.0}, damping={".*": 10000.0}, armature=None,
        ),
    },
)
```

> **说明 / 可能要调的点**：
> - 正则 `idx2._arm_l_joint.` 里 `.` 匹配任意单字符（数字），能命中 `idx21_arm_l_joint1`…`idx27_arm_l_joint7`。**若 Step 6.2 发现 USD 关节名不同，改这些正则。**
> - **mimic 关节**：若 URDF Importer 把 6 个 mimic 关节导成了普通 revolute（未保留耦合），它们没被任何 actuator 覆盖 → Isaac Lab 可能报「joint 无 actuator」。两种处理见 **Step 6.6**（推荐：在 Importer 里让 mimic 生效；否则给它们也加一个 `hands_mimic` 执行器组并在主循环按倍率驱动）。
> - `locked` 组把 body/head/wheels 钉死；若某类关节名没被三组正则覆盖，Isaac Lab 会报错提示缺 actuator —— 按报错补正则即可。

- [ ] **Step 6.3 验收**：`python -c "import ast; ast.parse(open('robots/agibot.py').read()); print('OK')"`（在 `unitree_sim_isaaclab` 目录下）语法通过。真正加载校验放到 Step 6.4 脚本首跑。

### Step 6.4 独立脚本 `g2_teleop_sim.py`（Block5 主体）

在 `unitree_sim_isaaclab/` **根目录**新建 `g2_teleop_sim.py`（放根目录才能像 `sim_main.py` 一样 `import teleimager` / `from tasks...` / `from robots.agibot...`）。它做四件事：**搭场景 → 直接 DDS 收发 → 主循环把命令喂给关节 → 相机回传**。

> ⚠️ 先把下面 3 个占位符替换成 **Step 6.2 实测的链接名**：`<HEAD_LINK>`、`<L_HAND_CAM_LINK>`、`<R_HAND_CAM_LINK>`。

```python
# g2_teleop_sim.py  —— 放在 unitree_sim_isaaclab/ 根目录
"""AgiBot G2 + O10 遥操作仿真（独立轻量脚本 / Route B）。
InteractiveScene(G2 base-fix + 地面 + 灯 + 3 相机) + 直接 DDS 收发(domain 1)
+ 复用 teleimager.image_server / camera_state.get_camera_image 回传第一视角到 Quest3。
运行: conda activate env_isaaclab; cd unitree_sim_isaaclab; python g2_teleop_sim.py [--headless] [--no_camera]
      需要相机(VR第一视角)时: python g2_teleop_sim.py --enable_cameras  (且须先填好 Step 6.2 的相机父链接名)
"""
import argparse
import numpy as np
import torch

# ---------- 1. 必须用 AppLauncher 启动（不要用裸 SimulationApp）----------
# AppLauncher 加载 IsaacLab 的 .kit experience，其 exts.folders 含 "${app}/../source"，
# 使 isaaclab_contrib 等【源码扩展】能被发现并按需启用；裸 SimulationApp 不扫描该目录，
# 于是 import isaaclab.scene 会在 interactive_scene.py:39 触发
# "ModuleNotFoundError: No module named 'isaaclab_contrib'"。这也是 sim_main.py:95 用 AppLauncher 的原因。
from isaaclab.app import AppLauncher
_ap = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(_ap)          # 提供 --headless/--device/--enable_cameras/--xr 等（勿再自定义 --headless）
_ap.add_argument("--num_envs", type=int, default=1)
_ap.add_argument("--no_camera", action="store_true", help="不回传相机(省GPU, 仅验证控制)")
args = _ap.parse_args()
app_launcher = AppLauncher(args)                 # 启动 app 并启用 IsaacLab 扩展（含 isaaclab_contrib）
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg
from isaaclab.utils import configclass

from robots.agibot import G2_CFG_WITH_O10_BASE_FIX
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_, HandState_, HandCmd_
from unitree_sdk2py.idl.default import (unitree_hg_msg_dds__LowState_, unitree_hg_msg_dds__HandState_,
                                        unitree_hg_msg_dds__MotorState_)
from unitree_sdk2py.utils.crc import CRC

# ---------- 2. §1.1 索引契约（与 teleop 侧严格一致）----------
ARM_JOINT_NAMES = [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)] + \
                  [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]           # 14: motor_cmd[0..13]
L_HAND_ACTIVE = ["idx31_hand_l_thumb_roll_joint", "idx32_hand_l_thumb_abad_joint", "idx33_hand_l_thumb_mcp_joint",
                 "idx36_hand_l_index_abad_joint", "idx37_hand_l_index_pip_joint", "idx39_hand_l_middle_pip_joint",
                 "idx41_hand_l_ring_abad_joint", "idx42_hand_l_ring_pip_joint",
                 "idx44_hand_l_pinky_abad_joint", "idx45_hand_l_pinky_pip_joint"]   # 10: motor_cmd[0..9]
R_HAND_ACTIVE = ["idx71_hand_r_thumb_roll_joint", "idx72_hand_r_thumb_abad_joint", "idx73_hand_r_thumb_mcp_joint",
                 "idx76_hand_r_index_abad_joint", "idx77_hand_r_index_pip_joint", "idx79_hand_r_middle_pip_joint",
                 "idx81_hand_r_ring_abad_joint", "idx82_hand_r_ring_pip_joint",
                 "idx84_hand_r_pinky_abad_joint", "idx85_hand_r_pinky_pip_joint"]
# mimic: (child, parent, multiplier)  —— 来自 URDF <mimic>（offset 均为 0）
MIMIC_MAP = [
    ("idx34_hand_l_thumb_pip_joint", "idx33_hand_l_thumb_mcp_joint", 1.33),
    ("idx35_hand_l_thumb_dip_joint", "idx33_hand_l_thumb_mcp_joint", 1.3),
    ("idx38_hand_l_index_dip_joint", "idx37_hand_l_index_pip_joint", 1.097),
    ("idx40_hand_l_middle_dip_joint", "idx39_hand_l_middle_pip_joint", 1.097),
    ("idx43_hand_l_ring_dip_joint", "idx42_hand_l_ring_pip_joint", 1.097),
    ("idx46_hand_l_pinky_dip_joint", "idx45_hand_l_pinky_pip_joint", 1.097),
    ("idx74_hand_r_thumb_pip_joint", "idx73_hand_r_thumb_mcp_joint", 1.33),
    ("idx75_hand_r_thumb_dip_joint", "idx73_hand_r_thumb_mcp_joint", 1.3),
    ("idx78_hand_r_index_dip_joint", "idx77_hand_r_index_pip_joint", 1.097),
    ("idx80_hand_r_middle_dip_joint", "idx79_hand_r_middle_pip_joint", 1.097),
    ("idx83_hand_r_ring_dip_joint", "idx82_hand_r_ring_pip_joint", 1.097),
    ("idx86_hand_r_pinky_dip_joint", "idx85_hand_r_pinky_pip_joint", 1.097),
]

# ---------- 3. 相机父链接（★替换成 Step 6.2 的实测名★）----------
HEAD_LINK       = "<HEAD_LINK>"          # 例: head_link3 / idx13_head_...
L_HAND_CAM_LINK = "<L_HAND_CAM_LINK>"    # 例: idx31_hand_l_camera_joint 的 child link
R_HAND_CAM_LINK = "<R_HAND_CAM_LINK>"

def _cam(link, name, pos, rot, focal=12.0):
    return CameraCfg(
        prim_path=f"/World/envs/env_.*/Robot/{link}/{name}",
        update_period=0.02, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=focal, focus_distance=400.0,
                                         horizontal_aperture=20.0, clipping_range=(0.1, 1.0e5)),
        offset=CameraCfg.OffsetCfg(pos=pos, rot=rot, convention="ros"))

@configclass
class G2TeleopSceneCfg(InteractiveSceneCfg):
    num_envs = 1
    env_spacing = 2.5
    replicate_physics = True
    ground = AssetBaseCfg(prim_path="/World/GroundPlane", spawn=GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/light",
                         spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0))
    robot: ArticulationCfg = G2_CFG_WITH_O10_BASE_FIX.replace(prim_path="{ENV_REGEX_NS}/Robot")
    # 前方(头)相机：ros convention 朝前的标准四元数；腕相机 pos/rot 先给近似值，跑起来后按画面微调
    front_camera: CameraCfg = _cam(HEAD_LINK, "front_cam", (0.0, 0.0, 0.0), (0.5, -0.5, 0.5, -0.5), focal=7.6)
    left_wrist_camera: CameraCfg = _cam(L_HAND_CAM_LINK, "left_wrist_camera", (0.0, 0.0, 0.05), (0.5, -0.5, 0.5, -0.5))
    right_wrist_camera: CameraCfg = _cam(R_HAND_CAM_LINK, "right_wrist_camera", (0.0, 0.0, 0.05), (0.5, -0.5, 0.5, -0.5))

# ---------- 4. DDS 桥（直接 in-process 收发, 无共享内存 / 无 dds_master）----------
class G2SimBridge:
    N_ARM, N_HAND = 14, 10
    def __init__(self):
        self.crc = CRC()
        ChannelFactoryInitialize(1)                       # domain id=1, 与 teleop 侧一致
        self.arm_q   = np.zeros(self.N_ARM)
        self.lhand_q = np.zeros(self.N_HAND)
        self.rhand_q = np.zeros(self.N_HAND)
        # 订阅命令
        self.lowcmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_);           self.lowcmd_sub.Init(self._on_arm, 32)
        self.lcmd_sub   = ChannelSubscriber("rt/o10/left/cmd", HandCmd_);    self.lcmd_sub.Init(self._on_lhand, 32)
        self.rcmd_sub   = ChannelSubscriber("rt/o10/right/cmd", HandCmd_);   self.rcmd_sub.Init(self._on_rhand, 32)
        # 发布状态
        self.lowstate_pub = ChannelPublisher("rt/lowstate", LowState_);         self.lowstate_pub.Init()
        self.lstate_pub   = ChannelPublisher("rt/o10/left/state", HandState_);  self.lstate_pub.Init()
        self.rstate_pub   = ChannelPublisher("rt/o10/right/state", HandState_); self.rstate_pub.Init()
        self.lowstate = unitree_hg_msg_dds__LowState_()   # motor_state = array[35], 前14槽给双臂
        self.lstate = unitree_hg_msg_dds__HandState_()    # ⚠️ 默认仅7, 扩到10
        self.rstate = unitree_hg_msg_dds__HandState_()
        for s in (self.lstate, self.rstate):
            while len(s.motor_state) < self.N_HAND:
                s.motor_state.append(unitree_hg_msg_dds__MotorState_())

    def _on_arm(self, msg: LowCmd_):
        if self.crc.Crc(msg) != msg.crc:                  # CRC 校验失败直接丢弃
            return
        self.arm_q = np.array([msg.motor_cmd[i].q for i in range(self.N_ARM)], dtype=np.float64)
    def _on_lhand(self, msg: HandCmd_):
        self.lhand_q = np.array([msg.motor_cmd[i].q for i in range(min(self.N_HAND, len(msg.motor_cmd)))], dtype=np.float64)
    def _on_rhand(self, msg: HandCmd_):
        self.rhand_q = np.array([msg.motor_cmd[i].q for i in range(min(self.N_HAND, len(msg.motor_cmd)))], dtype=np.float64)

    def publish_arm_state(self, q, dq):
        for i in range(self.N_ARM):
            self.lowstate.motor_state[i].q = float(q[i]); self.lowstate.motor_state[i].dq = float(dq[i])
        self.lowstate.mode_machine = 0                    # teleop 侧 get_mode_machine() 需要非 None
        self.lowstate.crc = self.crc.Crc(self.lowstate)
        self.lowstate_pub.Write(self.lowstate)
    def publish_hand_state(self, lq, ldq, rq, rdq):
        for i in range(self.N_HAND):
            self.lstate.motor_state[i].q = float(lq[i]); self.lstate.motor_state[i].dq = float(ldq[i])
            self.rstate.motor_state[i].q = float(rq[i]); self.rstate.motor_state[i].dq = float(rdq[i])
        self.lstate_pub.Write(self.lstate); self.rstate_pub.Write(self.rstate)

# ---------- 5. 主程序 ----------
def _resolve(all_names, wanted, tag):
    missing = [n for n in wanted if n not in all_names]
    if missing:
        raise RuntimeError(f"[{tag}] USD 里找不到关节 {missing}\n请对照 Step 6.2 输出核对关节名(可能被 Importer 改名/加前缀)。")
    return [all_names.index(n) for n in wanted]

def main():
    sim = SimulationContext(SimulationCfg(dt=0.005, render_interval=2, device=args.device))
    scene_cfg = G2TeleopSceneCfg(num_envs=args.num_envs, env_spacing=2.5, replicate_physics=True)
    if args.no_camera:                           # 关相机时不 spawn 相机 prim：此时链接名仍是占位符 <HEAD_LINK>，prim_path 非法会崩
        scene_cfg.front_camera = None            # InteractiveScene 对 None 字段直接跳过(interactive_scene.py:732)
        scene_cfg.left_wrist_camera = None
        scene_cfg.right_wrist_camera = None
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    robot = scene["robot"]
    dev = robot.device

    jn = list(robot.data.joint_names)
    arm_idx   = _resolve(jn, ARM_JOINT_NAMES, "arm")
    lhand_idx = _resolve(jn, L_HAND_ACTIVE, "left-hand")
    rhand_idx = _resolve(jn, R_HAND_ACTIVE, "right-hand")
    mimic_present = [(c, p, r) for (c, p, r) in MIMIC_MAP if c in jn and p in jn]
    print(f"[G2-SIM] joints={len(jn)}  mimic present={len(mimic_present)}/12")
    arm_t   = torch.tensor(arm_idx,   device=dev, dtype=torch.long)
    lhand_t = torch.tensor(lhand_idx, device=dev, dtype=torch.long)
    rhand_t = torch.tensor(rhand_idx, device=dev, dtype=torch.long)
    mim_c = torch.tensor([jn.index(c) for (c, p, r) in mimic_present], device=dev, dtype=torch.long)
    mim_p = torch.tensor([jn.index(p) for (c, p, r) in mimic_present], device=dev, dtype=torch.long)
    mim_r = torch.tensor([r for (c, p, r) in mimic_present], device=dev, dtype=torch.float32)

    bridge = G2SimBridge()

    env_shim = None
    if not args.no_camera:
        from tasks.common_observations.camera_state import get_camera_image
        from teleimager.image_server import run_isaacsim_server
        class _EnvShim:                                    # 让 get_camera_image(env) 认得我们的 scene
            def __init__(self, scene, dt): self.scene = scene; self.physics_dt = dt
        env_shim = _EnvShim(scene, sim.get_physics_dt())
        run_isaacsim_server()                              # 启动 ZMQ/WebRTC 图像服务(读 isaac_*_image_shm)

    print("[G2-SIM] ready，等待 teleop 命令 (rt/lowcmd, rt/o10/*/cmd) ...")
    while simulation_app.is_running():
        target = robot.data.default_joint_pos.clone()      # (num_envs, nj)：先回默认(锁定关节保持)
        target[:, arm_t]   = torch.tensor(bridge.arm_q,   device=dev, dtype=torch.float32)
        target[:, lhand_t] = torch.tensor(bridge.lhand_q, device=dev, dtype=torch.float32)
        target[:, rhand_t] = torch.tensor(bridge.rhand_q, device=dev, dtype=torch.float32)
        if mim_c.numel() > 0:                              # mimic = 倍率 × 父关节【命令值】
            target[:, mim_c] = mim_r * target[:, mim_p]
        robot.set_joint_position_target(target)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())

        q  = robot.data.joint_pos[0].detach().cpu().numpy()
        dq = robot.data.joint_vel[0].detach().cpu().numpy()
        bridge.publish_arm_state(q[arm_idx], dq[arm_idx])
        bridge.publish_hand_state(q[lhand_idx], dq[lhand_idx], q[rhand_idx], dq[rhand_idx])

        if env_shim is not None:
            get_camera_image(env_shim)                     # 相机 → 共享内存 → Quest3

    simulation_app.close()

if __name__ == "__main__":
    main()
```

> **关键点说明**
> - **必须用 `AppLauncher` 启动（本 Step 已修正）**：`isaaclab.scene.interactive_scene:39` 硬 import `isaaclab_contrib`，而 `env_isaaclab` 只 pip editable 装了 isaaclab/assets/tasks/mimic/rl 五个，**没有 isaaclab_contrib**。`AppLauncher` 加载的 `isaaclab.python*.kit` 里 `exts.folders` 含 `${app}/../source`，Kit 扩展管理器据此**发现并按需启用** `isaaclab_contrib`；裸 `SimulationApp` 不扫描该目录 → `ModuleNotFoundError: No module named 'isaaclab_contrib'`。这正是本仓库 `sim_main.py:95` 用 `AppLauncher` 的原因。
> - **`--headless` 不要自定义**：`AppLauncher.add_app_launcher_args` 已提供 `--headless/--device/--enable_cameras/--xr`；脚本只额外加 `--num_envs/--no_camera`，否则 argparse 报 `conflicting option string: --headless`。
> - **`--no_camera` 会把三个相机字段置 None**：否则相机 prim 的父链接仍是占位符 `<HEAD_LINK>`，`prim_path` 含非法字符 `<>` 会崩；`InteractiveScene` 对 None 字段直接跳过（`interactive_scene.py:732`）。要真出图须先填 Step 6.2 实测链接名 + 加 `--enable_cameras`。
> - **DDS 收发用回调**：`ChannelSubscriber.Init(callback, 32)` 与 `dds/g1_robot_dds.py`、`dds/dex3_dds.py` 一致（回调在 DDS 线程里把最新命令**整体替换**成一个新的 numpy 数组，主循环读到的永远是一致快照，无需加锁）。
> - **臂 CRC**：订阅 `rt/lowcmd` 必须 `self.crc.Crc(msg) != msg.crc` 校验（teleop 侧 `G2_ArmController` 发布时算了 CRC）；发布 `rt/lowstate` 也要回算 `crc` 并给 `mode_machine`（teleop 侧 `get_mode_machine()` 需要非 `None`）。
> - **手状态扩容**：`unitree_hg_msg_dds__HandState_()` 默认只有 7 个 `motor_state`，O10 要 10 → `__init__` 里 append 到 10（与 Step 4.1 teleop 侧对称）。
> - **锁定关节**：`target` 每帧先 `= default_joint_pos`，body/head/wheels 因为 `locked` 组高刚度+0 速度限位，会稳稳停在默认位姿；只有臂(14)+手(20)+mimic(12) 被命令覆盖。
> - **相机 env-shim**：`get_camera_image` 只用到 `env.scene[...]` / `env.scene.keys()` / `env.physics_dt`，一个只有 `.scene`、`.physics_dt` 的壳对象即可复用，无需搭完整 `ManagerBasedRLEnv`。

- [ ] **Step 6.4 验收（先不连 teleop，单独起 sim）**：
  ```bash
  unset PYTHONPATH LD_LIBRARY_PATH; conda activate env_isaaclab
  cd /home/amit/DATA/unitree_sim_isaaclab
  python g2_teleop_sim.py --no_camera        # 先关相机, 只验证场景+DDS能起
  ```
  应看到 G2 出现在地面、底座固定不下坠，打印 `[G2-SIM] joints=... mimic present=N/12` 与 `[G2-SIM] ready...`，且**无**「joint 无 actuator / no joints matched」报错。

### Step 6.5 两端联调运行（先 sim 后 teleop）

**① sim 端（conda: `env_isaaclab`）——先启动**（它会持续发 `rt/lowstate`，teleop 的 `G2_ArmController` 靠 `wait_for_dds` 才能解除阻塞）：
```bash
unset PYTHONPATH LD_LIBRARY_PATH
conda activate env_isaaclab
cd /home/amit/DATA/unitree_sim_isaaclab
python g2_teleop_sim.py --no_camera      # 先只验证控制链路：不 spawn 相机(此时相机父链接名还是占位符 <HEAD_LINK>，spawn 会崩)
# ★ VR 第一视角(要相机)： python g2_teleop_sim.py --enable_cameras           (须先填好 Step 6.2 相机父链接名)
# ★ 无显示器 + 相机：     python g2_teleop_sim.py --headless --enable_cameras
```

**② teleop 端（conda: `tv`）——后启动**（与你 Phase 1 跑通 G1 的命令一致，只把 `--arm`/`--ee` 换掉）：
```bash
unset PYTHONPATH LD_LIBRARY_PATH
conda activate tv
cd /opt/workspace/xr_teleoperate/teleop
python teleop_hand_and_arm.py --arm G2 --ee o10 --motion --sim \
       --img-server-ip <SIM端_IP> --input-mode hand
```
> - `<SIM端_IP>`：sim 机器的 IP（相机 WebRTC/ZMQ 回传用）。同机调试可用 `127.0.0.1`；Phase 1 默认是 `192.168.123.164`。
> - 其余标志（`--motion`/`--sim`/`--input-mode`）沿用你 Phase 1 跑通 G1 时的写法即可。**注意**：机械臂逻辑机型无关可自动复用，但手骨架喂数须按 **Step 5.5** 把 `o10` 加入白名单，否则手不动。

**③ Quest3 端**：戴上头显，打开与 Phase 1 相同的 WebXR 页面（teleop 起的 HTTPS:8012），允许手部追踪；挥动双手/屈指验证。

- [ ] **Step 6.5 验收**：teleop 端日志出现 `G2 arms initialized` 与 `Initialize O10_Controller OK!`，**没有**卡在 `Waiting to subscribe dds`；sim 端持续收到命令（可在主循环临时打印 `bridge.arm_q` 验证）。

### Step 6.6 mimic 耦合关节验证 / 回退

O10 每手 6 个 `dip/pip` 耦合关节（共 12）靠 URDF `<mimic>` 联动。本手顺采用**“当普通 revolute 关节 + 主循环按倍率驱动”**的确定性方案（不依赖 Importer 是否保留 mimic）。看 Step 6.4 启动打印的 `mimic present=N/12`：

| 情况 | 含义 | 处理 |
|---|---|---|
| **N = 12** | Importer 把 12 个耦合关节导成了独立 revolute（最常见） | ✅ 无需改动，主循环已按倍率驱动；`agibot.py` 保留 `hands_mimic` 组 |
| **N = 0** | Importer 把它们导成 fixed / 合并到父关节 | 从 `agibot.py` **删掉** `_O10_MIMIC` 与 `hands_mimic` 组（否则报 no joints matched）；指尖不会单独弯，但 pip 仍动，遥操演示可接受 |
| **0 < N < 12** | 部分保留 | 主循环已自动只驱动存在的那几个（`mimic_present` 过滤）；若报 `hands_mimic` 组 no joints matched，把 `_O10_MIMIC` 改成只保留实际存在的那几个 |

**倍率表**（已写进 `MIMIC_MAP`，源自 URDF，offset 均为 0）：

| 耦合关节 | = 倍率 × 父关节 |
|---|---|
| `thumb_pip` | 1.33 × `thumb_mcp` |
| `thumb_dip` | 1.30 × `thumb_mcp` |
| `index_dip` | 1.097 × `index_pip` |
| `middle_dip` | 1.097 × `middle_pip` |
| `ring_dip` | 1.097 × `ring_pip` |
| `pinky_dip` | 1.097 × `pinky_pip` |

- [ ] **Step 6.6 验收**：VR 里握拳 → sim 中 G2 五指**连指尖(dip)一起弯曲**（而非只弯近节）。若指尖不动→ 查 `mimic present` 是否 12、`MIMIC_MAP` 名字是否与 Step 6.2 一致。

---

## 7. 整体验收清单（端到端）

- [ ] **资产**：`g2_o10.usd` 导入成功，底座固定不下坠，双臂+双 O10 手完整。
- [ ] **场景**：`python g2_teleop_sim.py --no_camera` 无 actuator/关节报错，打印 `mimic present=N/12`。
- [ ] **链路**：sim 先起、teleop 后起；teleop 无 `Waiting to subscribe dds` 卡死。
- [ ] **双臂**：挥动 Quest3 手 → G2 双臂跟随（`G2_ArmIK` 解算 → `rt/lowcmd[0..13]` → sim `set_joint_position_target`）。
- [ ] **双手**：屈指/张手 → O10 五指跟随，且指尖(dip)随 mimic 联动。
- [ ] **第一视角**：Quest3 里看到头部相机画面（及双腕相机，若启用）。
- [ ] **稳定性**：无 CRC 告警刷屏；连续运行 1~2 min 不崩溃/不发散。

---

## 8. 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `ModuleNotFoundError: No module named 'isaaclab_contrib'`（import isaaclab.scene 时） | 用了裸 `SimulationApp` 而非 `AppLauncher`；isaaclab_contrib 未 pip 安装，只有 Kit 扫 `${app}/../source` 才能发现并启用 | Step 6.4 脚本头部改用 `AppLauncher(args)` 启动（本手顺已修正）；`--headless` 交给 `add_app_launcher_args`，勿自定义 |
| teleop 卡在 `Waiting to subscribe dds` | sim 未先发 `rt/lowstate`；domain 不一致；跨机网络/防火墙 | 确认 sim 先起且在发状态；两端都 `ChannelFactoryInitialize(1)`；同机先验证 |
| `IndexError ... motor_cmd[7]` / `motor_state[7]` | 忘了把 `HandCmd_`/`HandState_` 默认 7 扩到 10 | 核对 Step 4.1（teleop）与 Step 6.4 `G2SimBridge`（sim）的 `while len(...)<10: append(...)` |
| `joint ... has no actuator`（idx34/idx38…） | 12 个 mimic 关节未被 actuator 覆盖 | `agibot.py` 保留 `hands_mimic` 组（Step 6.3） |
| `no joints matched` for `hands_mimic` | Importer 把 mimic 导成了 fixed（N=0） | 删掉 `_O10_MIMIC` + `hands_mimic` 组（Step 6.6） |
| `_resolve` 报 `USD 里找不到关节 [...]` | Importer 改了关节名/前缀 | 用 Step 6.2 实测名修正 `ARM_JOINT_NAMES`/`L_HAND_ACTIVE`/`R_HAND_ACTIVE` 与 `agibot.py` 正则 |
| 臂下垂/抖动/跟不上 | 增益不当 | 调 `agibot.py` `arms` 组 stiffness/damping（肩肘~300/3，腕~40/1.5）；必要时同步调 teleop `G2_ArmController` 的 kp/kd |
| **机械臂随手腕动，但 O10 手完全不动** | `teleop_hand_and_arm.py:331` 喂手骨架的白名单 `args.ee in (...)` 漏了 `o10` → 手骨架数组恒为全 0 → 发布全 0 cmd（臂走 `wrist_pose` 另一条路不受影响） | 按 **Step 5.5** 把 `o10` 加入该元组；改后重启 teleop |
| **O10 手动了但恒定握拳、不跟随张/合** | O10 URDF 四指朝 +Z，与 dex-retargeting 人手骨架约定(-Y，同 inspire/dex3) 差一个旋转 → `retarget` 把 `pip` 顶到上限，张手/握拳输出几乎相同 | 在 `O10_Controller.control_process` 对 `ref_left/right_value` 右乘 `_HUMAN_TO_O10_ROT`（见 Step 4 代码/要点，已修正）；改后重启 teleop |
| Quest3 画面黑屏 / 无图 | 相机未渲染 / shm 名不匹配 / 未调 `get_camera_image` | 先不加 `--no_camera`；确认 Step 6.2 的 `<HEAD_LINK>` 等链接名正确；查 `isaac_{head,left,right}_image_shm` 是否在写 |
| 机器人爆炸/下坠 | 底座未固定 / init z 不对 | 重导 URDF 时勾 **Fix Base Link**；调 `agibot.py` `init_state.pos` 的 z |
| 手腕相机画面方向怪 | `offset.rot` 默认值不合适 | 按实际画面调 `_cam(...)` 的 `pos`/`rot`（参考 `camera_configs.py` 里 dex3 腕相机的四元数） |

---

## 9. 改动文件汇总

**teleop 端（`/opt/workspace/xr_teleoperate`）**
| 文件 | 改动 | 对应 |
|---|---|---|
| `teleop/robot_control/robot_arm.py` | 新增 `G2_Num_*` 常量、`G2_LowState`、`G2_JointArmIndex`、`G2_ArmController` | Block2 / Step 3 |
| `teleop/robot_control/robot_hand_o10.py` | **新建**：`O10_Controller`（含 7→10 扩容） | Block4 / Step 4 |
| `teleop/teleop_hand_and_arm.py` | `--arm` 加 `G2`、`--ee` 加 `o10`；import；arm/ee 分支 | Step 5 |

**sim 端（`/home/amit/DATA/unitree_sim_isaaclab`）**
| 文件 | 改动 | 对应 |
|---|---|---|
| `assets/robots/g2-o10-base-fix-usd/g2_o10.usd` | **新建**：URDF→USD（Fix Base Link） | Step 6.1 |
| `robots/agibot.py` | **新建**：`G2_CFG_WITH_O10_BASE_FIX`（arms/hands/hands_mimic/locked） | Step 6.3 |
| `g2_teleop_sim.py` | **新建**：独立仿真脚本（场景+DDS桥+主循环+相机） | Step 6.4 |

**复用（未改）**：`tasks/common_observations/camera_state.py`、`teleimager/image_server.py`（相机回传）；`assets/o10_hand/o10.yml`（Block3）；`teleop/robot_control/robot_arm_ik.py` 的 `G2_ArmIK`（Block1）。

> 至此，**Quest3 → teleop(G2_ArmIK + O10 retarget) → DDS → Isaac Sim(G2+O10) → 相机回传 Quest3** 全链路打通，体验与 Phase 1 的 G1+Dex3 一致。
