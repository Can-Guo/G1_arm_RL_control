import time
import sys

from unitree_sdk2_python.unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2_python.unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2_python.unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2_python.unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2_python.unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2_python.unitree_sdk2py.utils.crc import CRC
from unitree_sdk2_python.unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2_python.unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import numpy as np
Set_Angle = 40.0  # degrees

G1_NUM_MOTOR = 14

Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1   # arms 
]


class G1ARMJointIndex:
    """Joint index mapping for the G1 arm."""
    LEFT_ARM = {
        "shoulder_pitch": 15,
        "shoulder_roll": 16,
        "shoulder_yaw": 17,
        "elbow": 18,
        "wrist_roll": 19,
        "wrist_pitch": 20,
        "wrist_yaw": 21,
    }
    RIGHT_ARM = {
        "shoulder_pitch": 22,
        "shoulder_roll": 23,
        "shoulder_yaw": 24,
        "elbow": 25,
        "wrist_roll": 26,
        "wrist_pitch": 27,
        "wrist_yaw": 28,
    }

class G1LowLevelControl:
    def __init__(self):
        """Initialize the low-level control interface for the Unitree G1 robot.
        """
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 2.0    # [2 s] for safety
        self.counter_ = 0
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()
        print("[G1LowLevelControl] CRC initialized")

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        print("[G1LowLevelControl] MotionSwitcherClient initialized")

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        print("[G1LowLevelControl] Motion mode released")

        # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()
        print("[G1LowLevelControl] LowCmd publisher initialized")

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)
        print("[G1LowLevelControl] LowState subscriber initialized")

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            self.lowCmdWriteThreadPtr.Start()

        print("[G1LowLevelControl] Control thread started")
        

    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        if self.time_ < self.duration_ :
            # [Stage 1]: set robot

            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            # self.low_cmd.mode_pr = Mode.PR
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.motor_cmd[G1ARMJointIndex.RIGHT_ARM["elbow"]].mode =  1 # 1:Enable, 0:Disable 
            self.low_cmd.motor_cmd[G1ARMJointIndex.RIGHT_ARM["elbow"]].tau = 0. 
            self.low_cmd.motor_cmd[G1ARMJointIndex.RIGHT_ARM["elbow"]].q = (1.0 - ratio) * Set_Angle * np.pi / 180.0 
            self.low_cmd.motor_cmd[G1ARMJointIndex.RIGHT_ARM["elbow"]].dq = 0. 
            self.low_cmd.motor_cmd[G1ARMJointIndex.RIGHT_ARM["elbow"]].kp = Kp[G1ARMJointIndex.RIGHT_ARM["elbow"]] 
            self.low_cmd.motor_cmd[G1ARMJointIndex.RIGHT_ARM["elbow"]].kd = Kd[G1ARMJointIndex.RIGHT_ARM["elbow"]] 

        self.low_cmd.crc = self.crc.Crc(self.low_cmd) 
        self.lowcmd_publisher_.Write(self.low_cmd) 


    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True


if __name__ == "__main__":
    # Args:
    #     iface (str): Network interface to use for DDS communication.
    # """
    # Initialize DDS channels
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    time.sleep(1)  # wait for dds init
    G1_LowControl = G1LowLevelControl()
    print("G1LowControl initialized")
    G1_LowControl.Start()

    time.sleep(10)
    print("Example finished, exiting...")
    exit()

    
    



