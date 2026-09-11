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

# ⚠️ 帧约定修正（关键）：O10 URDF 的四指在 q=0 时指向 +Z，而 dex-retargeting 的人手骨架
#    约定与仓库里能正常工作的 inspire/dex3 一致——手指指向 -Y。两者差一个固定旋转。
#    若直接把人手向量喂给 O10 retarget，优化器会追不到的方向→把 pip 关节顶到上限(1.48)，
#    表现为【恒定握拳、不跟随手势】。用下面的矩阵把【库约定 -Y】的人手向量旋进【O10 URDF +Z】：
#    ref = ref @ _HUMAN_TO_O10_ROT  （左右手通用；已用 inspire FK 真实手形实测：张手→pip≈0.1，握拳→pip≈0.3，无饱和）。
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