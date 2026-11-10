#!/usr/bin/env python3.10
"""g1_dual_arm_policy_controller.py – 双臂交互式控制器，使用预训练的PPO策略

该脚本允许同时控制G-1机器人的左臂和右臂，使用不同的键盘按键：
- 左臂：W/A/S/D/Q/E (目标位置控制)
- 右臂：I/K/J/L/U/O (目标位置控制)

按键绑定
========
左臂控制：
  w  : 目标 ↑  (+z)
  s  : 目标 ↓  (−z)
  a  : 目标 ←  (+y)
  d  : 目标 →  (−y)
  q  : 目标 前进  (+x)
  e  : 目标 后退  (−x)

右臂控制：
  i  : 目标 ↑  (+z)
  k  : 目标 ↓  (−z)
  j  : 目标 ←  (+y)
  l  : 目标 →  (−y)
  u  : 目标 前进  (+x)
  o  : 目标 后退  (−x)

其他控制：
  r  : 切换左臂真实机器人输出
  t  : 切换左臂仿真输出
  n  : 切换右臂真实机器人输出
  m  : 切换右臂仿真输出
  Esc/x : 退出

支持两种独立的输出模式：
  • MuJoCo仿真 (始终可用, "sim")
  • 通过SDK-2的真实机器人 ("robot") – 可选，需要 ``unitree_sdk2py``

默认情况下只启用仿真。使用对应的按键可以激活真实机器人的关节目标流输出。
"""

from __future__ import annotations

import argparse
import curses
import math
import pathlib
import threading
import time
from types import SimpleNamespace
from typing import Dict, List, Tuple
import mujoco
import numpy as np

# ---------------------------------------------------------------------------
# 0.  Stable-Baselines3 策略加载器
# ---------------------------------------------------------------------------

def load_policy(path: pathlib.Path):  # noqa: D401
    from stable_baselines3 import PPO  # type: ignore
    from stable_baselines3 import SAC  # type: ignore

    try:
        policy = PPO.load(str(path), device="cpu")
        policy.set_parameters(policy.get_parameters())  # 确保确定性
        policy.policy.set_training_mode(False)
        print(f"[policy] 从 {path} 加载PPO模型 …")
        return policy, "PPO"
    except:
        try:
            policy = SAC.load(str(path), device="cpu")
            policy.set_parameters(policy.get_parameters())  # 确保确定性
            policy.policy.set_training_mode(False)
            print(f"[policy] 从 {path} 加载SAC模型 …")
            return policy, "SAC"
        except Exception as e:
            print(f"[policy] 加载模型失败: {e}")
            raise

# ---------------------------------------------------------------------------
# 1.  双臂MuJoCo环境
# ---------------------------------------------------------------------------

class DualArmEnv:
    """双臂环境包装器，使用单一模型统一管理双臂"""
    def __init__(self, render: bool = True):
        import g1_arm_rl_env as _env
        import mujoco as mujoco
        import pathlib as pathlib
        
        # 只创建一个基础环境用于初始化
        base_env = _env.G1ArmReachEnv(render_mode="none", left_or_right_arm="left")
        
        # 加载双臂XML文件
        root = pathlib.Path(__file__).resolve().parent
        dual_xml = root / "unitree_mujoco" / "unitree_robots" / "g1" / "g1_29dof_with_hand_dual.xml"
        
        if dual_xml.exists():
            self.model = mujoco.MjModel.from_xml_path(str(dual_xml))
            print(f"[dual_env] 已加载双臂XML: {dual_xml}")
        else:
            self.model = base_env.model
            print(f"[dual_env] 双臂XML文件未找到，使用默认XML")
        
        self.data = mujoco.MjData(self.model)
        
        # 只创建一个渲染器
        self.viewer = None
        if render:
            try:
                from mujoco import viewer as mj_viewer  # type: ignore
                self.viewer = mj_viewer.launch_passive(self.model, self.data)
            except Exception:
                self.viewer = None
        
        # 初始化双臂配置（包含 self._mujoco 等）
        self._setup_arm_configs(base_env)

        # 先初始化安全目标与当前目标，再去定位 mocap 并同步
        self.left_safe_goal  = np.array([0.1, 0.25, 0.70], dtype=np.float32)
        self.right_safe_goal = np.array([0.1, -0.25, 0.70], dtype=np.float32)
        self.left_goal = self.left_safe_goal.copy()
        self.right_goal = self.right_safe_goal.copy()

        # 定位目标体和mocap（内部会尝试 _sync_goals）
        self._setup_goal_bodies()
        
        # 环境状态
        self.max_steps = 500
        self.left_step_count = 0
        self.right_step_count = 0

        # 启动时可以放一次安全姿态；之后不要在改目标时再 reset
        self.reset_dual()
    
    def set_left_goal(self, goal_pos):
        """设置左臂目标位置（仅移动mocap，不reset）"""
        self.left_goal = np.asarray(goal_pos, dtype=np.float32).copy()
        self._sync_goals()
                
    def set_right_goal(self, goal_pos):
        """设置右臂目标位置（仅移动mocap，不reset）"""
        self.right_goal = np.asarray(goal_pos, dtype=np.float32).copy()
        self._sync_goals()
    
    def _setup_arm_configs(self, base_env):
        """设置双臂配置信息"""
        # 复制关节配置
        self._left_joint_names = base_env._left_joint_names
        self._right_joint_names = base_env._right_joint_names
        
        # 计算关节地址
        self._left_qadr = []
        self._right_qadr = []
        self._left_dadr = []
        self._right_dadr = []
        
        for name in self._left_joint_names:
            jid = base_env._mujoco.mj_name2id(self.model, base_env._mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                self._left_qadr.append(int(self.model.jnt_qposadr[jid]))
                self._left_dadr.append(int(self.model.jnt_dofadr[jid]))
        
        for name in self._right_joint_names:
            jid = base_env._mujoco.mj_name2id(self.model, base_env._mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                self._right_qadr.append(int(self.model.jnt_qposadr[jid]))
                self._right_dadr.append(int(self.model.jnt_dofadr[jid]))
        
        # 保存mujoco引用
        self._mujoco = base_env._mujoco
        
        # 设置停用臂的rest pose
        self._setup_rest_poses()
        
        # 手掌位置计算设置 (复制自原环境)
        self._setup_palm_configs(base_env)
    
    def _setup_rest_poses(self):
        """设置停用臂的静止姿态"""
        import math
        # 右臂静止姿态
        right_rest_deg = [16, -16, 2, 55, -12, 0, 0]
        self._right_rest_q = np.array(right_rest_deg, dtype=np.float32) * (math.pi / 180.0)
        
        # 左臂静止姿态（镜像）
        left_rest_deg = [16, 16, -2, 55, 12, 0, 0]
        self._left_rest_q = np.array(left_rest_deg, dtype=np.float32) * (math.pi / 180.0)
    
    def _setup_palm_configs(self, base_env):
        """设置手掌位置计算配置"""
        # 左臂手掌配置
        self._left_use_wrist_offset = True
        left_wrist_body = "left_wrist_yaw_link"
        self._left_body_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, left_wrist_body)
        self._left_palm_offset = np.array([0.0415, 0.003, 0.0], dtype=np.float32)
        
        # 右臂手掌配置
        self._right_use_wrist_offset = True
        right_wrist_body = "right_wrist_yaw_link"
        self._right_body_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, right_wrist_body)
        self._right_palm_offset = np.array([0.0415, -0.003, 0.0], dtype=np.float32)
    
    def _setup_goal_bodies(self):
        """设置左臂和右臂目标体的ID（带兜底）"""
        try:
            self.left_goal_bid = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, "left_goal_body")
            self.right_goal_bid = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, "right_goal_body")
        except Exception as e:
            raise RuntimeError(f"查找目标体失败: {e}")
        
        self.left_goal_mid = -1
        self.right_goal_mid = -1

        if self.left_goal_bid != -1:
            self.left_goal_mid = int(self.model.body_mocapid[self.left_goal_bid])
        if self.right_goal_bid != -1:
            self.right_goal_mid = int(self.model.body_mocapid[self.right_goal_bid])

        # 兜底1：扫描名字包含“goal”的 body
        if self.left_goal_mid == -1 or self.right_goal_mid == -1:
            candidates = []
            for bid in range(self.model.nbody):
                name = self._mujoco.mj_id2name(self.model, self._mujoco.mjtObj.mjOBJ_BODY, bid)
                if not name:
                    continue
                if "goal" in name:
                    mid = int(self.model.body_mocapid[bid])
                    if mid != -1:
                        candidates.append((bid, mid, name))
            if self.left_goal_mid == -1 and len(candidates) >= 1:
                self.left_goal_bid, self.left_goal_mid, _ = candidates[0]
            if self.right_goal_mid == -1 and len(candidates) >= 2:
                self.right_goal_bid, self.right_goal_mid, _ = candidates[1]

        # 兜底2：如果仍失败但模型里至少有两个 mocap，则用 mocap 0/1
        if (self.left_goal_mid == -1 or self.right_goal_mid == -1) and getattr(self.model, "nmocap", 0) >= 2:
            if self.left_goal_mid == -1:
                self.left_goal_mid = 0
            if self.right_goal_mid == -1:
                self.right_goal_mid = 1

        # 初始化一次目标位置（如果有 mocap 可用）— 仅在目标已存在时同步
        if hasattr(self, "left_goal") and hasattr(self, "right_goal"):
            self._sync_goals()

        print(f"[dual_env] 左臂目标体ID: {self.left_goal_bid}, mocap: {self.left_goal_mid}")
        print(f"[dual_env] 右臂目标体ID: {self.right_goal_bid}, mocap: {self.right_goal_mid}")

    def reset_dual(self):
        """重置双臂环境（仅在启动或用户要求时调用）"""
        self._reset_to_safe_pose()
        self.left_step_count = 0
        self.right_step_count = 0
        mujoco.mj_forward(self.model, self.data)

    # 新增：只重置计数器并返回观测，不改变关节，也不改变目标
    def reset_left_arm(self):
        self.left_step_count = 0
        return self.get_left_obs()

    def reset_right_arm(self):
        self.right_step_count = 0
        return self.get_right_obs()
    
    def _reset_to_safe_pose(self):
        """将两臂放到 rest 姿态；不会被 set_*_goal 触发"""
        # 左臂
        for i, adr in enumerate(self._left_qadr):
            self.data.qpos[adr] = float(self._left_rest_q[i])
        # 右臂
        for i, adr in enumerate(self._right_qadr):
            self.data.qpos[adr] = float(self._right_rest_q[i])
        # 速度清零，避免抖动
        self.data.qvel[:] = 0.0
    
    def _fk_left(self):
        """计算左臂手掌位置"""
        if self._left_use_wrist_offset:
            wrist_pos = np.array(self.data.xpos[self._left_body_id])
            wrist_rot = np.array(self.data.xmat[self._left_body_id]).reshape(3, 3)
            return wrist_pos + wrist_rot.dot(self._left_palm_offset)
        return np.array([0, 0, 0])  # fallback
    
    def _fk_right(self):
        """计算右臂手掌位置"""
        if self._right_use_wrist_offset:
            wrist_pos = np.array(self.data.xpos[self._right_body_id])
            wrist_rot = np.array(self.data.xmat[self._right_body_id]).reshape(3, 3)
            return wrist_pos + wrist_rot.dot(self._right_palm_offset)
        return np.array([0, 0, 0])  # fallback
    
    
    def step_left(self, action):
        """左臂步进"""
        # 限制动作范围
        action = np.clip(action, -0.05, 0.05)
        
        # 应用关节增量
        for i, (qadr, delta_q) in enumerate(zip(self._left_qadr, action)):
            self.data.qpos[qadr] = np.clip(
                self.data.qpos[qadr] + float(delta_q), -3.14, 3.14
            )
        
        # 锁定其他关节
        self._lock_joints()
        
        # 前向动力学
        self._mujoco.mj_forward(self.model, self.data)
        
        # 计算奖励
        reward, done = self._compute_left_reward()
        
        # 更新步数
        self.left_step_count += 1
        if self.left_step_count >= self.max_steps:
            done = True
        
        obs = self.get_left_obs()
        info = {"dist": np.linalg.norm(self.left_goal - self._fk_left())}
        
        return obs, reward, done, False, info
    
    def step_right(self, action):
        """右臂步进"""
        # 限制动作范围
        action = np.clip(action, -0.05, 0.05)
        
        # 应用关节增量
        for i, (qadr, delta_q) in enumerate(zip(self._right_qadr, action)):
            self.data.qpos[qadr] = np.clip(
                self.data.qpos[qadr] + float(delta_q), -3.14, 3.14
            )
        
        # 锁定其他关节
        self._lock_joints()
        
        # 前向动力学
        self._mujoco.mj_forward(self.model, self.data)
        
        # 计算奖励
        reward, done = self._compute_right_reward()
        
        # 更新步数
        self.right_step_count += 1
        if self.right_step_count >= self.max_steps:
            done = True
        
        obs = self.get_right_obs()
        info = {"dist": np.linalg.norm(self.right_goal - self._fk_right())}
        
        return obs, reward, done, False, info
    
    def _lock_joints(self):
        """锁定腰部关节"""
        waist_joints = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
        for joint_name in waist_joints:
            jid = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if jid != -1:
                qadr = int(self.model.jnt_qposadr[jid])
                dadr = int(self.model.jnt_dofadr[jid])
                self.data.qpos[qadr] = 0.0
                self.data.qvel[dadr] = 0.0
    
    def _compute_left_reward(self):
        """计算左臂奖励"""
        p_hand = self._fk_left()
        delta = self.left_goal - p_hand
        dist = np.linalg.norm(delta)
        
        # 基础距离奖励
        reward = -5.0 * dist
        
        # 成功奖励
        success = dist < 0.03
        if success:
            reward += 1.0
            return reward, True
        
        return reward, False
    
    def _compute_right_reward(self):
        """计算右臂奖励"""
        p_hand = self._fk_right()
        delta = self.right_goal - p_hand
        dist = np.linalg.norm(delta)
        
        # 基础距离奖励
        reward = -5.0 * dist
        
        # 成功奖励
        success = dist < 0.03
        if success:
            reward += 1.0
            return reward, True
        
        return reward, False
    
    def get_left_obs(self):
        """获取左臂观测"""
        # 关节角度和速度
        q = np.array([self.data.qpos[qadr] for qadr in self._left_qadr], dtype=np.float32)
        dq = np.array([self.data.qvel[dadr] for dadr in self._left_dadr], dtype=np.float32)
        
        # 手掌位置
        p_hand = self._fk_left()
        
        # 目标位置和距离
        delta = self.left_goal - p_hand
        step_frac = np.array([self.left_step_count / self.max_steps], dtype=np.float32)
        
        return np.concatenate([q, dq, p_hand, self.left_goal, delta, step_frac]).astype(np.float32)
    
    def get_right_obs(self):
        """获取右臂观测"""
        # 关节角度和速度
        q = np.array([self.data.qpos[qadr] for qadr in self._right_qadr], dtype=np.float32)
        dq = np.array([self.data.qvel[dadr] for dadr in self._right_dadr], dtype=np.float32)
        
        # 手掌位置
        p_hand = self._fk_right()
        
        # 目标位置和距离
        delta = self.right_goal - p_hand
        step_frac = np.array([self.right_step_count / self.max_steps], dtype=np.float32)
        
        return np.concatenate([q, dq, p_hand, self.right_goal, delta, step_frac]).astype(np.float32)
    
    def _sync_goals(self):
        """把当前 left_goal/right_goal 写到 mocap，不做任何关节/状态重置"""
        if hasattr(self, 'data') and hasattr(self.data, 'mocap_pos'):
            if getattr(self, 'left_goal_mid', -1) != -1:
                self.data.mocap_pos[self.left_goal_mid] = self.left_goal
            if getattr(self, 'right_goal_mid', -1) != -1:
                self.data.mocap_pos[self.right_goal_mid] = self.right_goal
            mujoco.mj_forward(self.model, self.data)

    def set_left_goal(self, goal_pos):
        """设置左臂目标位置（仅移动mocap，不reset）"""
        self.left_goal = np.asarray(goal_pos, dtype=np.float32).copy()
        self._sync_goals()
                
    def set_right_goal(self, goal_pos):
        """设置右臂目标位置（仅移动mocap，不reset）"""
        self.right_goal = np.asarray(goal_pos, dtype=np.float32).copy()
        self._sync_goals()
        # 注意：不要在这里调用 reset_dual()，否则每次改目标都会回到rest姿态
    
    def render(self):
        """渲染主环境"""
        if self.viewer is not None:
            self.viewer.sync()

# ---------------------------------------------------------------------------
# 2.  机器人桥接器
# ---------------------------------------------------------------------------

class RobotBridge:
    """发布 LowCmd 消息的简单包装器"""

    def __init__(self, iface: str, domain: int):
        try:
            from unitree_sdk2py.core.channel import (  # type: ignore
                ChannelFactoryInitialize,
                ChannelPublisher,
            )
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_  # type: ignore
            from unitree_sdk2py.idl.default import (  # type: ignore
                unitree_hg_msg_dds__LowCmd_,
            )
        except Exception:
            print("[robot] SDK-2 不存在 – 机器人输出已禁用")
            self.ok = False
            return

        try:
            print("domain, iface: ", domain," ", iface)
            ChannelFactoryInitialize(domain, iface)
            self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self._pub.Init()

            self._cmd = unitree_hg_msg_dds__LowCmd_()

            for mc in self._cmd.motor_cmd:
                mc.mode = 0
                mc.kp = 40.0
                mc.kd = 1.0

            if 29 < len(self._cmd.motor_cmd):
                self._cmd.motor_cmd[29].q = 1.0

            try:
                from unitree_sdk2py.utils.crc import CRC  # type: ignore
                self._crc = CRC()
            except Exception:
                self._crc = None

            self.ok = True
        except Exception as e:
            print(f"[robot] DDS初始化失败 – 机器人已禁用 ({e})")
            self.ok = False

    def send_qpos(self, q: Dict[int, float]) -> None:
        """发送关节位置命令"""
        if not self.ok:
            return

        # 只有前29个条目是电机 – 为了安全跳过手部
        for idx, val in q.items():
            if idx >= 29:
                continue
            if idx < len(self._cmd.motor_cmd):
                self._cmd.motor_cmd[idx].q = float(val)

        if self._crc is not None:
            if hasattr(self._crc, "Crc"):
                self._cmd.crc = self._crc.Crc(self._cmd)
            elif hasattr(self._crc, "calculate_crc"):
                self._cmd.crc = self._crc.calculate_crc(self._cmd)

        self._pub.Write(self._cmd)

# ---------------------------------------------------------------------------
# 3.  关节索引映射
# ---------------------------------------------------------------------------

# 电机索引，人类可读标签，MuJoCo关节/执行器前缀
JOINTS: List[Tuple[int, str, str]] = [
    (15, "L shoulder-pitch", "left_shoulder_pitch"),
    (16, "L shoulder-roll",  "left_shoulder_roll"),
    (17, "L shoulder-yaw",   "left_shoulder_yaw"),
    (18, "L elbow",          "left_elbow"),
    (19, "L wrist-roll",     "left_wrist_roll"),
    (20, "L wrist-pitch",    "left_wrist_pitch"),
    (21, "L wrist-yaw",      "left_wrist_yaw"),
    (22, "R shoulder-pitch", "right_shoulder_pitch"),
    (23, "R shoulder-roll",  "right_shoulder_roll"),
    (24, "R shoulder-yaw",   "right_shoulder_yaw"),
    (25, "R elbow",          "right_elbow"),
    (26, "R wrist-roll",     "right_wrist_roll"),
    (27, "R wrist-pitch",    "right_wrist_pitch"),
    (28, "R wrist-yaw",      "right_wrist_yaw"),
]

IDX2LABEL = {idx: lbl for idx, lbl, _ in JOINTS}

# ---------------------------------------------------------------------------
# 4.  物理步进线程（MuJoCo查看器同步）
# ---------------------------------------------------------------------------

def viewer_sync(env, stop_evt: threading.Event, speed, lock: threading.Lock):
    """持续调用 env.render() 同时确保MuJoCo线程安全"""
    # 不再使用线程渲染，保留函数但不调用，避免GL上下文跨线程
    while not stop_evt.is_set():
        with lock:
            env.render()
        time.sleep(speed.dt)

# ---------------------------------------------------------------------------
# 5.  Curses UI主循环
# ---------------------------------------------------------------------------

def run_curses(stdscr, env, left_policy, right_policy, robot: RobotBridge, 
               out_left_sim: bool, out_left_robot: bool,
               out_right_sim: bool, out_right_robot: bool,
               speed, lock: threading.Lock):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    STEP = 0.02

    # 初始化姿态应用（如果可用）
    try:
        from g1_initial_pose import POSE_DICT  # type: ignore
        import mujoco as _mj

        def _joint_name(idx: int) -> str | None:
            left = [
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
                "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            ]
            right = [
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
            ]

            if idx == 12:
                return "waist_yaw_joint"
            if 15 <= idx <= 21:
                return left[idx - 15]
            if 22 <= idx <= 28:
                return right[idx - 22]
            return None

        def apply_pose():
            with lock:
                for midx, q in POSE_DICT.items():
                    jname = _joint_name(midx)
                    if jname is None:
                        continue
                    jid = _mj.mj_name2id(env.model, _mj.mjtObj.mjOBJ_JOINT, jname)
                    if jid == -1:
                        continue
                    qadr = int(env.model.jnt_qposadr[jid])
                    env.data.qpos[qadr] = float(q)

                _mj.mj_forward(env.model, env.data)

                # 将目标移动到当前手腕位置并同步到可视化
                try:
                    left_hand = env._fk_left()
                    env.set_left_goal(left_hand)

                    right_hand = env._fk_right()
                    env.set_right_goal(right_hand)
                except Exception as e:
                    print(f"[init] 目标位置初始化失败: {e}")

        apply_pose()
        
        # 验证目标体设置
        print(f"[init] 左臂目标体状态: bid={env.left_goal_bid}, mid={env.left_goal_mid}")
        print(f"[init] 右臂目标体状态: bid={env.right_goal_bid}, mid={env.right_goal_mid}")
        
    except Exception as e:
        print(f"[init] 初始姿态应用失败: {e}")

    # 初始观测需要加锁
    with lock:
        left_obs = env.get_left_obs()
        right_obs = env.get_right_obs()

    # 保持目标的直接引用以便快速更新
    left_goal = env.left_goal.copy()
    right_goal = env.right_goal.copy()

    last_robot_send = 0.0
    last_safe_qpos = None
    left_hold_mode = False
    right_hold_mode = False
    left_collision_freeze = False
    right_collision_freeze = False

    while True:
        ch = stdscr.getch()
        if ch != -1:
            key = chr(ch & 0xFF)
            if key in ("x", "\x1b"):
                break
            # 左臂控制 (wsadqe)
            elif key == "w":
                left_goal[2] += STEP
            elif key == "s":
                left_goal[2] -= STEP
            elif key == "a":
                left_goal[1] += STEP
            elif key == "d":
                left_goal[1] -= STEP
            elif key == "q":
                left_goal[0] += STEP
            elif key == "e":
                left_goal[0] -= STEP
            
            # 右臂控制 (ikjluo)
            elif key == "i":
                right_goal[2] += STEP
            elif key == "k":
                right_goal[2] -= STEP
            elif key == "j":
                right_goal[1] += STEP
            elif key == "l":
                right_goal[1] -= STEP
            elif key == "u":
                right_goal[0] += STEP
            elif key == "o":
                right_goal[0] -= STEP
            
            # 输出控制
            elif key == "r":
                out_left_robot = not out_left_robot and robot.ok
            elif key == "t":
                out_left_sim = not out_left_sim
            elif key == "n":
                out_right_robot = not out_right_robot and robot.ok
            elif key == "m":
                out_right_sim = not out_right_sim
            
            # 速度控制
            elif key == "]":  # 渲染更快
                speed.dt = max(0.005, speed.dt * 0.5)
            elif key == "[":  # 渲染更慢
                speed.dt = min(0.5, speed.dt * 2.0)
            elif key in (">", "."):
                speed.mult = min(4.0, speed.mult * 2.0)
            elif key in ("<", ","):
                speed.mult = max(0.25, speed.mult * 0.5)

        # 限制目标到合理的范围
        left_goal = np.clip(left_goal, [0.0, 0.0, 0.55], [0.6, 0.6, 1.3])
        right_goal = np.clip(right_goal, [0.0, -0.6, 0.55], [0.6, 0.0, 1.3])

        # 应用新目标到环境（加锁）
        with lock:
            env.set_left_goal(left_goal)
            env.set_right_goal(right_goal)

        # 左臂策略控制
        if out_left_sim or out_left_robot:
            left_collided = False
            if hasattr(env, "left_arm_gids") and hasattr(env, "left_protect_gids"):
                # 读取接触数据需要加锁（已在下面使用 lock 包裹距离计算和step）
                pass

            with lock:
                left_dist = np.linalg.norm(env.left_goal - env._fk_left())

            if left_collision_freeze:
                left_hold_mode = True
            else:
                if not left_hold_mode and left_dist < 0.03:
                    left_hold_mode = True
                elif left_hold_mode and left_dist >= 0.04:
                    left_hold_mode = False
                    left_collision_freeze = False

            if left_hold_mode:
                with lock:
                    left_obs, _, _, _, _ = env.step_left(np.zeros(7, dtype=np.float32))
                env.left_step_count = 0
            else:
                left_action, _ = left_policy.predict(left_obs, deterministic=True)
                left_action = np.clip(left_action * speed.mult, -0.05, 0.05)
                with lock:
                    left_obs, _, left_done, _, left_info = env.step_left(left_action)
                    if not left_collided:
                        last_safe_qpos = env.data.qpos.copy()
                    if left_done:
                        left_obs = env.reset_left_arm()
                        env.set_left_goal(left_goal)

        # 右臂策略控制
        if out_right_sim or out_right_robot:
            right_collided = False
            # 同上省略接触筛选，真正用到时需放到锁内
            with lock:
                right_dist = np.linalg.norm(env.right_goal - env._fk_right())

            if right_collision_freeze:
                right_hold_mode = True
            else:
                if not right_hold_mode and right_dist < 0.03:
                    right_hold_mode = True
                elif right_hold_mode and right_dist >= 0.04:
                    right_hold_mode = False
                    right_collision_freeze = False

            if right_hold_mode:
                with lock:
                    right_obs, _, _, _, _ = env.step_right(np.zeros(7, dtype=np.float32))
                env.right_step_count = 0
            else:
                right_action, _ = right_policy.predict(right_obs, deterministic=True)
                right_action = np.clip(right_action * speed.mult, -0.05, 0.05)
                with lock:
                    right_obs, _, right_done, _, right_info = env.step_right(right_action)
                    if right_done:
                        right_obs = env.reset_right_arm()
                        env.set_right_goal(right_goal)

        # 可选机器人输出（读取 qpos 需加锁）
        if (out_left_robot or out_right_robot) and robot.ok and (time.time() - last_robot_send) > 0.02:
            with lock:
                if not hasattr(env, "_motor_qadr"):
                    import mujoco as _mj
                    qadr = {}
                    for idx, _lbl, mj_short in JOINTS:
                        jname_joint = mj_short + "_joint"
                        jid = _mj.mj_name2id(env.model, _mj.mjtObj.mjOBJ_JOINT, jname_joint)
                        if jid != -1:
                            qadr[idx] = int(env.model.jnt_qposadr[jid])
                    env._motor_qadr = qadr

                qpos = {idx: float(env.data.qpos[adr]) for idx, adr in env._motor_qadr.items()}
            robot.send_qpos(qpos)
            last_robot_send = time.time()

        # 绘制UI覆盖层（读取手掌位置需加锁）
        stdscr.erase()
        stdscr.addstr(0, 0, "G-1 双臂RL控制器 – 左臂:wsadqe 右臂:ikjluo [] 速度 rt(左臂) nm(右臂) Esc退出")
        
        # 输出状态显示
        left_sim_status = 'X' if out_left_sim else ' '
        left_robot_status = 'X' if out_left_robot else '-' if robot.ok else '-'
        right_sim_status = 'X' if out_right_sim else ' '
        right_robot_status = 'X' if out_right_robot else '-' if robot.ok else '-'
        
        stdscr.addstr(1, 0, f"左臂 → 仿真:[{left_sim_status}] 机器人:[{left_robot_status}]  右臂 → 仿真:[{right_sim_status}] 机器人:[{right_robot_status}]")
        stdscr.addstr(2, 0, f"速率:{speed.dt*1000:.0f} ms  倍数:{speed.mult:.2f}")

        # 左臂状态
        left_status = "HOLD" if left_hold_mode else "RUN "
        if left_collision_freeze:
            left_status = "COLL"
        
        # 右臂状态
        right_status = "HOLD" if right_hold_mode else "RUN "
        if right_collision_freeze:
            right_status = "COLL"

        stdscr.addstr(4, 0, f"左臂目标: x={left_goal[0]:.3f}  y={left_goal[1]:.3f}  z={left_goal[2]:.3f}  模式:{left_status}")
        stdscr.addstr(5, 0, f"右臂目标: x={right_goal[0]:.3f}  y={right_goal[1]:.3f}  z={right_goal[2]:.3f}  模式:{right_status}")
        
        # 显示当前手掌位置
        try:
            with lock:
                left_hand = env._fk_left()
            stdscr.addstr(6, 0, f"左臂手掌: x={left_hand[0]:.3f}  y={left_hand[1]:.3f}  z={left_hand[2]:.3f}")
        except Exception:
            stdscr.addstr(6, 0, "左臂手掌: (不可用)")
            
        try:
            with lock:
                right_hand = env._fk_right()
            stdscr.addstr(7, 0, f"右臂手掌: x={right_hand[0]:.3f}  y={right_hand[1]:.3f}  z={right_hand[2]:.3f}")
        except Exception:
            stdscr.addstr(7, 0, "右臂手掌: (不可用)")

        stdscr.refresh()

        # 在主循环里渲染（与viewer在同线程，避免GL上下文跨线程）
        with lock:
            env.render()

        time.sleep(speed.dt)

# ---------------------------------------------------------------------------
# 6.  入口点
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="G-1双臂交互式RL控制器")

    ap.add_argument("--left-model", default="models/ppo_left_5M_Sep11/ppo_g1_left_5000k.zip", help="左臂训练模型路径")
    ap.add_argument("--right-model", default="models/ppo_right_5M_Aug28/ppo_g1_right_5000k.zip", help="右臂训练模型路径")
    ap.add_argument("--iface", default="enx00e04c680266", help="DDS网络接口（机器人）")
    ap.add_argument("--domain", type=int, default=0, help="DDS域ID")
    ap.add_argument("--sim-only", action="store_true", help="完全禁用机器人输出")
    ap.add_argument("--rate", type=float, default=0.04, help="渲染/控制循环间隔（秒）")

    args = ap.parse_args()

    # 检查模型文件
    left_model_path = pathlib.Path(args.left_model).expanduser()
    right_model_path = pathlib.Path(args.right_model).expanduser()
    
    if not left_model_path.exists():
        raise SystemExit(f"左臂模型文件未找到: {left_model_path}")
    if not right_model_path.exists():
        raise SystemExit(f"右臂模型文件未找到: {right_model_path}")

    # 加载策略
    print("正在加载策略...")
    left_policy, left_algo = load_policy(left_model_path)
    right_policy, right_algo = load_policy(right_model_path)
    
    print(f"[main] 左臂使用 {left_algo} 策略")
    print(f"[main] 右臂使用 {right_algo} 策略")

    # 创建双臂环境
    env = DualArmEnv(render=True)

    # 初始化机器人桥接器
    robot = RobotBridge(args.iface, args.domain) if not args.sim_only else RobotBridge("", 0)

    out_left_sim = True
    out_left_robot = False
    out_right_sim = True
    out_right_robot = False

    speed = SimpleNamespace(dt=max(0.005, args.rate), mult=0.25)

    # 移除独立渲染线程，避免GL上下文跨线程
    lock = threading.Lock()

    try:
        curses.wrapper(run_curses, env, left_policy, right_policy, robot, 
                      out_left_sim, out_left_robot, out_right_sim, out_right_robot,
                      speed, lock)
    finally:
        pass  # 不再需要 stop_evt

if __name__ == "__main__":
    main()

