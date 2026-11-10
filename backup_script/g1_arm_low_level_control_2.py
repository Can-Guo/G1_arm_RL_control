import time
import sys
import threading
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Dict, Optional

# Unitree SDK imports
from unitree_sdk2_python.unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2_python.unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2_python.unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2_python.unitree_sdk2py.utils.crc import CRC
from unitree_sdk2_python.unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2_python.unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

# Constants
G1_NUM_MOTOR = 14
CONTROL_DT = 0.002  # 2ms control loop
DEFAULT_DURATION = 2.0  # 2 seconds for movement
MIN_ANGLE_ADJUSTMENT = 3.0  # degrees
MAX_ANGLE_ADJUSTMENT = 10.0  # degrees

# Joint control parameters
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

class ArmSide(Enum):
    LEFT = "left"
    RIGHT = "right"

class JointType(Enum):
    SHOULDER_PITCH = "shoulder_pitch"
    SHOULDER_ROLL = "shoulder_roll"
    SHOULDER_YAW = "shoulder_yaw"
    ELBOW = "elbow"
    WRIST_ROLL = "wrist_roll"
    WRIST_PITCH = "wrist_pitch"
    WRIST_YAW = "wrist_yaw"

class G1ARMJointIndex:
    """Joint index mapping for the G1 arm."""
    LEFT_ARM = {
        JointType.SHOULDER_PITCH: 15,
        JointType.SHOULDER_ROLL: 16,
        JointType.SHOULDER_YAW: 17,
        JointType.ELBOW: 18,
        JointType.WRIST_ROLL: 19,
        JointType.WRIST_PITCH: 20,
        JointType.WRIST_YAW: 21,
    }
    RIGHT_ARM = {
        JointType.SHOULDER_PITCH: 22,
        JointType.SHOULDER_ROLL: 23,
        JointType.SHOULDER_YAW: 24,
        JointType.ELBOW: 25,
        JointType.WRIST_ROLL: 26,
        JointType.WRIST_PITCH: 27,
        JointType.WRIST_YAW: 28,
    }

@dataclass
class JointState:
    current_angle: float  # radians
    target_angle: float   # radians
    is_moving: bool

class G1ArmController:
    """Secure API for controlling G1 robot arms with minimal adjustments."""
    
    def __init__(self, iface: Optional[str] = None):
        """Initialize the arm controller.
        
        Args:
            iface: Network interface to use for DDS communication
        """
        # Initialize DDS channels
        if iface:
            ChannelFactoryInitialize(0, iface)
        else:
            ChannelFactoryInitialize(0)
            
        time.sleep(1)  # wait for dds init
        
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()
        self.mode_machine_ = 0
        self.update_mode_machine_ = False
        self.control_active = False
        self.control_thread = None
        
        # Joint state tracking
        self.joint_states: Dict[int, JointState] = {}
        
        # Initialize motion switcher
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        print("[G1ArmController] MotionSwitcherClient initialized")
        
        # Release any existing motion mode
        self._release_motion_mode()
        
        # Create publisher and subscriber
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()
        
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._low_state_handler, 10)
        
        print("[G1ArmController] Initialization complete")
        
    def _release_motion_mode(self):
        """Release any existing motion mode for safety."""
        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        print("[G1ArmController] Motion mode released")
    
    def _low_state_handler(self, msg: LowState_):
        """Handle incoming low state messages."""
        self.low_state = msg
        
        if not self.update_mode_machine_:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True
            
        # Update joint states with current positions
        for joint_id in self.joint_states:
            if 0 <= joint_id < len(msg.motor_state):
                self.joint_states[joint_id].current_angle = msg.motor_state[joint_id].q
    
    def _control_loop(self):
        """Main control loop running at 500Hz."""
        while self.control_active:
            self._publish_joint_commands()
            time.sleep(CONTROL_DT)
    
    def _publish_joint_commands(self):
        """Publish commands for all controlled joints."""
        # Set machine mode
        self.low_cmd.mode_machine = self.mode_machine_
        
        # Set commands for all controlled joints
        for joint_id, joint_state in self.joint_states.items():
            if 0 <= joint_id < len(self.low_cmd.motor_cmd):
                # Enable motor
                self.low_cmd.motor_cmd[joint_id].mode = 1  # 1:Enable, 0:Disable
                self.low_cmd.motor_cmd[joint_id].tau = 0.0  # Zero torque for position control
                
                # Set target position
                self.low_cmd.motor_cmd[joint_id].q = joint_state.target_angle
                self.low_cmd.motor_cmd[joint_id].dq = 0.0  # Zero velocity
                
                # Set PID gains
                self.low_cmd.motor_cmd[joint_id].kp = Kp[joint_id]
                self.low_cmd.motor_cmd[joint_id].kd = Kd[joint_id]
        
        # Calculate and set CRC
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        
        # Publish command
        self.lowcmd_publisher.Write(self.low_cmd)
    
    def start_control(self):
        """Start the control loop."""
        if not self.update_mode_machine_:
            print("Warning: Machine mode not yet available")
            return False
            
        if self.control_active:
            print("Control already active")
            return True
            
        self.control_active = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        print("[G1ArmController] Control started")
        return True
    
    def stop_control(self):
        """Stop the control loop."""
        self.control_active = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        print("[G1ArmController] Control stopped")
    
    def get_joint_id(self, arm_side: ArmSide, joint_type: JointType) -> int:
        """Get the joint ID for a specific arm and joint type."""
        if arm_side == ArmSide.LEFT:
            return G1ARMJointIndex.LEFT_ARM[joint_type]
        else:
            return G1ARMJointIndex.RIGHT_ARM[joint_type]
    
    def add_joint_control(self, arm_side: ArmSide, joint_type: JointType):
        """Add a joint to be controlled.
        
        Args:
            arm_side: Left or right arm
            joint_type: Type of joint to control
        """
        joint_id = self.get_joint_id(arm_side, joint_type)
        
        # Initialize with current position if available, otherwise 0
        current_angle = 0.0
        if self.low_state and 0 <= joint_id < len(self.low_state.motor_state):
            current_angle = self.low_state.motor_state[joint_id].q
        
        self.joint_states[joint_id] = JointState(
            current_angle=current_angle,
            target_angle=current_angle,
            is_moving=False
        )
        
        print(f"[G1ArmController] Added control for {arm_side.value} arm {joint_type.value} (ID: {joint_id})")
    
    def set_joint_angle(self, arm_side: ArmSide, joint_type: JointType, 
                       angle_deg: float, duration: float = DEFAULT_DURATION) -> bool:
        """Set a joint to a specific angle.
        
        Args:
            arm_side: Left or right arm
            joint_type: Type of joint to control
            angle_deg: Target angle in degrees
            duration: Time to reach target in seconds
            
        Returns:
            True if command was accepted, False otherwise
        """
        joint_id = self.get_joint_id(arm_side, joint_type)
        
        if joint_id not in self.joint_states:
            print(f"Error: Joint {joint_type.value} on {arm_side.value} arm is not being controlled")
            return False
        
        # Convert to radians and update target
        angle_rad = np.deg2rad(angle_deg)
        self.joint_states[joint_id].target_angle = angle_rad
        self.joint_states[joint_id].is_moving = True
        
        print(f"Setting {arm_side.value} arm {joint_type.value} to {angle_deg}°")
        
        # Start a timer to mark when movement is complete
        def movement_complete():
            time.sleep(duration)
            self.joint_states[joint_id].is_moving = False
            
        threading.Thread(target=movement_complete, daemon=True).start()
        
        return True
    
    def adjust_joint_angle(self, arm_side: ArmSide, joint_type: JointType, 
                          adjustment_deg: float, duration: float = DEFAULT_DURATION) -> bool:
        """Adjust a joint angle by a specified amount.
        
        Args:
            arm_side: Left or right arm
            joint_type: Type of joint to control
            adjustment_deg: Angle adjustment in degrees
            duration: Time to reach target in seconds
            
        Returns:
            True if command was accepted, False otherwise
        """
        # Safety check - limit adjustment size
        if abs(adjustment_deg) > MAX_ANGLE_ADJUSTMENT:
            print(f"Error: Adjustment too large (max: {MAX_ANGLE_ADJUSTMENT}°)")
            return False
            
        joint_id = self.get_joint_id(arm_side, joint_type)
        
        if joint_id not in self.joint_states:
            print(f"Error: Joint {joint_type.value} on {arm_side.value} arm is not being controlled")
            return False
        
        if self.joint_states[joint_id].is_moving:
            print(f"Error: Joint {joint_type.value} on {arm_side.value} arm is already moving")
            return False
        
        # Calculate new target angle
        current_deg = np.rad2deg(self.joint_states[joint_id].current_angle)
        new_angle_deg = current_deg + adjustment_deg
        
        return self.set_joint_angle(arm_side, joint_type, new_angle_deg, duration)
    
    def minimal_adjustment(self, arm_side: ArmSide, joint_type: JointType, 
                          positive: bool = True) -> bool:
        """Make a minimal adjustment to a joint angle.
        
        Args:
            arm_side: Left or right arm
            joint_type: Type of joint to control
            positive: True for positive adjustment, False for negative
            
        Returns:
            True if command was accepted, False otherwise
        """
        adjustment = MIN_ANGLE_ADJUSTMENT if positive else -MIN_ANGLE_ADJUSTMENT
        return self.adjust_joint_angle(arm_side, joint_type, adjustment)
    
    def get_current_angle(self, arm_side: ArmSide, joint_type: JointType) -> Optional[float]:
        """Get the current angle of a joint in degrees.
        
        Args:
            arm_side: Left or right arm
            joint_type: Type of joint
            
        Returns:
            Current angle in degrees or None if joint not available
        """
        joint_id = self.get_joint_id(arm_side, joint_type)
        
        if joint_id in self.joint_states:
            return np.rad2deg(self.joint_states[joint_id].current_angle)
        
        return None

def keyboard_control_example():
    """Example of keyboard control for the G1 robot arms."""
    controller = G1ArmController()
    
    # Add joints to control
    controller.add_joint_control(ArmSide.RIGHT, JointType.ELBOW)
    controller.add_joint_control(ArmSide.RIGHT, JointType.SHOULDER_PITCH)
    
    # Start control loop
    if not controller.start_control():
        print("Failed to start control")
        return
    
    try:
        print("Keyboard control started")
        print("Press '1' to increase elbow angle by 3°")
        print("Press '2' to decrease elbow angle by 3°")
        print("Press '3' to increase shoulder angle by 3°")
        print("Press '4' to decrease shoulder angle by 3°")
        print("Press 'q' to quit")
        
        while True:
            key = input().strip().lower()
            
            if key == 'q':
                break
            elif key == '1':
                controller.minimal_adjustment(ArmSide.RIGHT, JointType.ELBOW, True)
            elif key == '2':
                controller.minimal_adjustment(ArmSide.RIGHT, JointType.ELBOW, False)
            elif key == '3':
                controller.minimal_adjustment(ArmSide.RIGHT, JointType.SHOULDER_PITCH, True)
            elif key == '4':
                controller.minimal_adjustment(ArmSide.RIGHT, JointType.SHOULDER_PITCH, False)
            else:
                print("Invalid key")
                
            # Display current angles
            elbow_angle = controller.get_current_angle(ArmSide.RIGHT, JointType.ELBOW)
            shoulder_angle = controller.get_current_angle(ArmSide.RIGHT, JointType.SHOULDER_PITCH)
            print(f"Current angles - Elbow: {elbow_angle:.1f}°, Shoulder: {shoulder_angle:.1f}°")
                
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        controller.stop_control()
        print("Control stopped")

if __name__ == "__main__":
    # You can run the keyboard control example
    # keyboard_control_example()
    
    # Or use the API programmatically
    controller = G1ArmController()
    
    # Add elbow joint control
    controller.add_joint_control(ArmSide.RIGHT, JointType.ELBOW)
    
    # Start control
    controller.start_control()
    
    # Make a small adjustment
    controller.minimal_adjustment(ArmSide.RIGHT, JointType.ELBOW, True)
    
    # Wait for movement to complete
    time.sleep(2.5)
    
    # Stop control
    controller.stop_control()