#!/usr/bin/env python3
"""g1_dual_arm_policy_controller_ROS2_v3.py – 基于 ROS2 的 G1 双臂 RL 控制器 (Action Server集成版)

本脚本是Version 2的升级版，新增功能：
  • 发布左右臂当前位置到ROS2话题（供Action Server订阅）
  • 订阅左右臂目标位置话题（从Action Server接收）
  • 支持手动键盘控制和Action Server控制双模式
  • 与arm_control_action_server.py配合使用

新增话题:
  发布:
    /g1/left_arm/position  (Float64MultiArray) - 左臂当前位置 [x,y,z]
    /g1/right_arm/position (Float64MultiArray) - 右臂当前位置 [x,y,z]
  
  订阅:
    /g1/left_arm/goal  (Float64MultiArray) - 左臂目标位置 [x,y,z]
    /g1/right_arm/goal (Float64MultiArray) - 右臂目标位置 [x,y,z]

运行模式:
  1. 手动模式: 使用键盘控制（默认）
  2. Action模式: 接收Action Server的目标位置
  3. 混合模式: 键盘按'm'切换控制权

参考实现:
  - g1_dual_arm_policy_controller_ROS2_v2.py (基础版本)
  - arm_control_action_server.py (Action服务器)

作者: York
日期: 2025-11-13
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

import numpy as np

# 距离阈值
DISTANCE_THRESHOLD = 0.03

# ---------------------------------------------------------------------------
# 0. 策略加载
# ---------------------------------------------------------------------------


def load_policy(path: pathlib.Path):
    """加载预训练的 PPO 或 SAC 策略"""
    from stable_baselines3 import PPO, SAC

    try:
        policy = PPO.load(str(path), device="cpu")
        policy.set_parameters(policy.get_parameters())
        policy.policy.set_training_mode(False)
        print(f"[policy] 已加载 PPO 模型: {path}")
        return policy, "PPO"
    except Exception as e:
        print(f"[policy] PPO 加载失败，尝试 SAC... ({e})")
        policy = SAC.load(str(path), device="cpu")
        policy.set_parameters(policy.get_parameters())
        policy.policy.set_training_mode(False)
        print(f"[policy] 已加载 SAC 模型: {path}")
        return policy, "SAC"


# ---------------------------------------------------------------------------
# 1. 双臂环境
# ---------------------------------------------------------------------------


class DualArmEnv:
    """双臂环境包装器"""
    
    def __init__(self, render: bool = True):
        import g1_arm_rl_env as _env
        import mujoco
        
        # 创建基础环境用于配置
        base_env = _env.G1ArmReachEnv(render_mode="none", left_or_right_arm="left")
        
        # 加载双臂 XML（当前在 arm_controller/ 子目录，XML 在上一级）
        script_dir = pathlib.Path(__file__).resolve().parent
        package_dir = script_dir.parent  # 上一级到 arm_controller 包根目录
        dual_xml = package_dir / "unitree_mujoco" / "g1_29dof_with_hand_dual.xml"
        
        if dual_xml.exists():
            self.model = mujoco.MjModel.from_xml_path(str(dual_xml))
            print(f"[dual_env] 已加载双臂 XML: {dual_xml}")
        else:
            self.model = base_env.model
            print(f"[dual_env] 双臂 XML 未找到，使用默认 XML")
        
        self.data = mujoco.MjData(self.model)
        
        # 渲染器
        self.viewer = None
        if render:
            try:
                from mujoco import viewer as mj_viewer
                self.viewer = mj_viewer.launch_passive(self.model, self.data)
            except Exception:
                self.viewer = None
        
        # 配置双臂
        self._setup_arm_configs(base_env)
        
        # 初始目标位置
        self.left_safe_goal = np.array([0.1, 0.25, 0.70], dtype=np.float32)
        self.right_safe_goal = np.array([0.1, -0.25, 0.70], dtype=np.float32)
        self.left_goal = self.left_safe_goal.copy()
        self.right_goal = self.right_safe_goal.copy()
        
        # 设置目标体
        self._setup_goal_bodies()
        
        # 环境状态
        self.max_steps = 500
        self.left_step_count = 0
        self.right_step_count = 0
        
        # 初始化
        self.reset_dual()
    
    def _setup_arm_configs(self, base_env):
        """配置双臂关节信息"""
        self._left_joint_names = base_env._left_joint_names
        self._right_joint_names = base_env._right_joint_names
        
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
        
        self._mujoco = base_env._mujoco
        self._setup_rest_poses()
        self._setup_palm_configs()
        self._setup_collision_groups()
    
    def _setup_rest_poses(self):
        """设置静止姿态"""
        right_rest_deg = [16, -16, 2, 55, -12, 0, 0]
        self._right_rest_q = np.array(right_rest_deg, dtype=np.float32) * (math.pi / 180.0)
        
        left_rest_deg = [16, 16, -2, 55, 12, 0, 0]
        self._left_rest_q = np.array(left_rest_deg, dtype=np.float32) * (math.pi / 180.0)
    
    def _setup_palm_configs(self):
        """配置手掌位置计算"""
        # 左臂
        self._left_use_wrist_offset = True
        self._left_body_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")
        self._left_palm_offset = np.array([0.0415, 0.003, 0.0], dtype=np.float32)
        
        # 右臂
        self._right_use_wrist_offset = True
        self._right_body_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")
        self._right_palm_offset = np.array([0.0415, -0.003, 0.0], dtype=np.float32)
    
    def _setup_collision_groups(self):
        """设置碰撞检测组（双臂版本）"""
        # 受保护的身体部位关键词
        protect_kw = (
            "pelvis", "waist", "torso", "chest", "spine", "abdomen", "body",
            "head", "neck", "hip_", "thigh", "knee", "calf", "shin", "leg",
        )
        
        # 左右臂的肢体关键词（排除 "hand" 以避免手指触发碰撞）
        limb_kw = ("upperarm", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw", "forearm", "shoulder")
        
        self._protect_gids = set()
        self._left_arm_gids = set()
        self._right_arm_gids = set()
        
        for gid in range(self.model.ngeom):
            bid = int(self.model.geom_bodyid[gid])
            body_name = self._mujoco.mj_id2name(self.model, self._mujoco.mjtObj.mjOBJ_BODY, bid)
            if not body_name:
                continue
            
            # 左臂几何体
            if body_name.startswith("left_") and any(k in body_name for k in limb_kw):
                self._left_arm_gids.add(gid)
            
            # 右臂几何体
            elif body_name.startswith("right_") and any(k in body_name for k in limb_kw):
                self._right_arm_gids.add(gid)
            
            # 受保护几何体
            if any(k in body_name for k in protect_kw):
                self._protect_gids.add(gid)
        
        # 启用碰撞层
        for gid in self._protect_gids | self._left_arm_gids | self._right_arm_gids:
            self.model.geom_contype[gid] |= 1
            self.model.geom_conaffinity[gid] |= 1
        
        print(f"[dual_env] 碰撞组配置: 左臂={len(self._left_arm_gids)} 右臂={len(self._right_arm_gids)} 保护={len(self._protect_gids)}")
    
    def _setup_goal_bodies(self):
        """设置左臂和右臂目标体的ID（带兜底机制）"""
        # 尝试标准命名：left_goal_body, right_goal_body
        try:
            self.left_goal_bid = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, "left_goal_body")
            self.right_goal_bid = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, "right_goal_body")
        except Exception:
            self.left_goal_bid = -1
            self.right_goal_bid = -1
        
        self.left_goal_mid = -1
        self.right_goal_mid = -1

        if self.left_goal_bid != -1:
            self.left_goal_mid = int(self.model.body_mocapid[self.left_goal_bid])
        if self.right_goal_bid != -1:
            self.right_goal_mid = int(self.model.body_mocapid[self.right_goal_bid])

        # 兜底1：扫描名字包含"goal"的 body（支持 goal_body_left/goal_body_right 等命名）
        if self.left_goal_mid == -1 or self.right_goal_mid == -1:
            candidates = []
            for bid in range(self.model.nbody):
                name = self._mujoco.mj_id2name(self.model, self._mujoco.mjtObj.mjOBJ_BODY, bid)
                if not name:
                    continue
                if "goal" in name.lower():
                    mid = int(self.model.body_mocapid[bid])
                    if mid != -1:
                        candidates.append((bid, mid, name))
            
            # 优先匹配包含 "left" 的作为左臂目标
            if self.left_goal_mid == -1:
                for bid, mid, name in candidates:
                    if "left" in name.lower():
                        self.left_goal_bid, self.left_goal_mid = bid, mid
                        print(f"[dual_env] 找到左臂目标体: {name}")
                        break
            
            # 优先匹配包含 "right" 的作为右臂目标
            if self.right_goal_mid == -1:
                for bid, mid, name in candidates:
                    if "right" in name.lower():
                        self.right_goal_bid, self.right_goal_mid = bid, mid
                        print(f"[dual_env] 找到右臂目标体: {name}")
                        break
            
            # 如果仍未找到，按顺序分配
            if self.left_goal_mid == -1 and len(candidates) >= 1:
                self.left_goal_bid, self.left_goal_mid, name = candidates[0]
                print(f"[dual_env] 使用第一个目标体作为左臂: {name}")
            if self.right_goal_mid == -1 and len(candidates) >= 2:
                self.right_goal_bid, self.right_goal_mid, name = candidates[1]
                print(f"[dual_env] 使用第二个目标体作为右臂: {name}")

        # 兜底2：如果仍失败但模型里至少有两个 mocap，则用 mocap 0/1
        if (self.left_goal_mid == -1 or self.right_goal_mid == -1) and getattr(self.model, "nmocap", 0) >= 2:
            if self.left_goal_mid == -1:
                self.left_goal_mid = 0
                print(f"[dual_env] 使用 mocap 0 作为左臂目标")
            if self.right_goal_mid == -1:
                self.right_goal_mid = 1
                print(f"[dual_env] 使用 mocap 1 作为右臂目标")

        # 同步目标位置
        self._sync_goals()
        
        print(f"[dual_env] 左臂目标体 ID: {self.left_goal_bid}, mocap: {self.left_goal_mid}")
        print(f"[dual_env] 右臂目标体 ID: {self.right_goal_bid}, mocap: {self.right_goal_mid}")
    
    def _sync_goals(self):
        """同步目标到 mocap"""
        if self.left_goal_mid != -1:
            self.data.mocap_pos[self.left_goal_mid] = self.left_goal
        if self.right_goal_mid != -1:
            self.data.mocap_pos[self.right_goal_mid] = self.right_goal
    
    def set_left_goal(self, goal_pos):
        """设置左臂目标"""
        self.left_goal = np.asarray(goal_pos, dtype=np.float32).copy()
        self._sync_goals()
    
    def set_right_goal(self, goal_pos):
        """设置右臂目标"""
        self.right_goal = np.asarray(goal_pos, dtype=np.float32).copy()
        self._sync_goals()
    
    def get_left_obs(self):
        """获取左臂观测 (24维): [q(7), dq(7), p_hand(3), p_goal(3), delta(3), step_frac(1)]"""
        q = np.array([self.data.qpos[i] for i in self._left_qadr], dtype=np.float32)
        dq = np.array([self.data.qvel[i] for i in self._left_dadr], dtype=np.float32)
        p_hand = self.get_left_palm_pos()
        delta = self.left_goal - p_hand
        step_frac = np.array([self.left_step_count / self.max_steps], dtype=np.float32)
        return np.concatenate([q, dq, p_hand, self.left_goal, delta, step_frac])
    
    def get_right_obs(self):
        """获取右臂观测 (24维): [q(7), dq(7), p_hand(3), p_goal(3), delta(3), step_frac(1)]"""
        q = np.array([self.data.qpos[i] for i in self._right_qadr], dtype=np.float32)
        dq = np.array([self.data.qvel[i] for i in self._right_dadr], dtype=np.float32)
        p_hand = self.get_right_palm_pos()
        delta = self.right_goal - p_hand
        step_frac = np.array([self.right_step_count / self.max_steps], dtype=np.float32)
        return np.concatenate([q, dq, p_hand, self.right_goal, delta, step_frac])
    
    def get_left_palm_pos(self):
        """获取左臂手掌位置"""
        return self._fk_left()
    
    def get_right_palm_pos(self):
        """获取右臂手掌位置"""
        return self._fk_right()
    
    def step_left(self, action):
        """左臂执行动作"""
        # 限制动作范围到 [-0.05, +0.05] 弧度增量
        action = np.clip(action, -0.05, 0.05)
        
        # 应用关节增量，并限制关节范围
        for i, qadr in enumerate(self._left_qadr):
            if i < len(action):
                self.data.qpos[qadr] = np.clip(
                    self.data.qpos[qadr] + float(action[i]), -3.14, 3.14
                )
        
        # 前向动力学
        self._mujoco.mj_forward(self.model, self.data)
        
        # 碰撞检测
        collision_hit = self._check_left_collision()
        
        # 更新步数
        self.left_step_count += 1
        done = self.left_step_count >= self.max_steps or collision_hit
        
        obs = self.get_left_obs()
        reward = -1.0 if collision_hit else 0.0
        info = {
            "dist": np.linalg.norm(self.left_goal - self.get_left_palm_pos()),
            "collision": collision_hit
        }
        
        return obs, reward, done, False, info
    
    def step_right(self, action):
        """右臂执行动作"""
        # 限制动作范围到 [-0.05, +0.05] 弧度增量
        action = np.clip(action, -0.05, 0.05)
        
        # 应用关节增量，并限制关节范围
        for i, qadr in enumerate(self._right_qadr):
            if i < len(action):
                self.data.qpos[qadr] = np.clip(
                    self.data.qpos[qadr] + float(action[i]), -3.14, 3.14
                )
        
        # 前向动力学
        self._mujoco.mj_forward(self.model, self.data)
        
        # 碰撞检测
        collision_hit = self._check_right_collision()
        
        # 更新步数
        self.right_step_count += 1
        done = self.right_step_count >= self.max_steps or collision_hit
        
        obs = self.get_right_obs()
        reward = -1.0 if collision_hit else 0.0
        info = {
            "dist": np.linalg.norm(self.right_goal - self.get_right_palm_pos()),
            "collision": collision_hit
        }
        
        return obs, reward, done, False, info
    
    def reset_dual(self):
        """重置双臂环境（仅在启动或用户要求时调用）"""
        self._reset_to_safe_pose()
        self.left_step_count = 0
        self.right_step_count = 0
        self._mujoco.mj_forward(self.model, self.data)
        self._sync_goals()
        return self.get_left_obs(), self.get_right_obs()
    
    def reset_left_arm(self):
        """重置左臂计数器（不改变姿态）"""
        self.left_step_count = 0
        return self.get_left_obs()
    
    def reset_right_arm(self):
        """重置右臂计数器（不改变姿态）"""
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
        return np.array([0, 0, 0])
    
    def _fk_right(self):
        """计算右臂手掌位置"""
        if self._right_use_wrist_offset:
            wrist_pos = np.array(self.data.xpos[self._right_body_id])
            wrist_rot = np.array(self.data.xmat[self._right_body_id]).reshape(3, 3)
            return wrist_pos + wrist_rot.dot(self._right_palm_offset)
        return np.array([0, 0, 0])
    
    def _check_left_collision(self):
        """检查左臂碰撞"""
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = c.geom1, c.geom2
            
            # 左臂碰到保护区域或右臂
            if (g1 in self._left_arm_gids and (g2 in self._protect_gids or g2 in self._right_arm_gids)) or \
               (g2 in self._left_arm_gids and (g1 in self._protect_gids or g1 in self._right_arm_gids)):
                penetration = max(0.0, -c.dist)
                if penetration >= 0.005:
                    return True
        return False
    
    def _check_right_collision(self):
        """检查右臂碰撞"""
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = c.geom1, c.geom2
            
            # 右臂碰到保护区域或左臂
            if (g1 in self._right_arm_gids and (g2 in self._protect_gids or g2 in self._left_arm_gids)) or \
               (g2 in self._right_arm_gids and (g1 in self._protect_gids or g1 in self._left_arm_gids)):
                penetration = max(0.0, -c.dist)
                if penetration >= 0.005:
                    return True
        return False
    
    def render(self):
        """渲染"""
        if self.viewer is not None:
            self.viewer.sync()
    
    def close(self):
        """关闭"""
        if self.viewer is not None:
            self.viewer.close()


# ---------------------------------------------------------------------------
# 2. ROS2 机器人桥接器 + 位置发布器
# ---------------------------------------------------------------------------


class RobotBridgeROS2:
    """ROS2 话题接口的机器人桥接器（双臂版本 + Action Server集成）"""

    def __init__(self):
        """初始化 ROS2 发布者和订阅者"""
        # === 基础属性（必须先初始化，避免依赖缺失时属性不存在） ===
        self.ok = False
        self._action_mode = False
        
        # === R键本地控制权限管理（新架构：不再通过服务器） ===
        self._manual_control_enabled = False  # R键本地控制开关
        self._manual_lock = threading.Lock()
        
        # === MCB控制权限管理（订阅authority_server） ===
        self._mcb_control_authority = False  # MCB控制权限状态
        self._mcb_lock = threading.Lock()
        
        print("[ROS2] 新架构：R键本地管理 + MCB远程管理")
        print("[ROS2] 控制权限 = R键使能 OR MCB授权")
        print("[ROS2] 急停拥有最高优先级")
        
        # === 急停状态管理 ===
        self.emergency_stop = False  # 急停状态标志
        self._estop_lock = threading.Lock()
        self._last_estop_time = 0.0  # 最后一次急停时间戳
        
        # === 目标和阈值管理（Action模式相关） ===
        self._external_left_goal = None
        self._external_right_goal = None
        self._goal_lock = threading.Lock()
        self._dynamic_threshold = DISTANCE_THRESHOLD
        self._threshold_lock = threading.Lock()
        
        # === ROS2订阅器占位符 ===
        self._left_goal_sub = None
        self._right_goal_sub = None
        
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float64MultiArray, Bool
            from unitree_hg.msg import LowCmd
            
            if not rclpy.ok():
                rclpy.init()
            
            self._rclpy = rclpy
            self._node = Node("g1_dual_arm_controller_v3")
            
            # 创建低层控制发布者
            self._pub = self._node.create_publisher(LowCmd, "/arm_sdk", 10)
            self._LowCmd = LowCmd
            
            # 初始化消息
            self._cmd = LowCmd()
            
            # 设置控制参数
            for i in range(len(self._cmd.motor_cmd)):
                self._cmd.motor_cmd[i].mode = 0
                self._cmd.motor_cmd[i].kp = 50.0
                self._cmd.motor_cmd[i].kd = 1.5
                self._cmd.motor_cmd[i].tau = 0.0
                self._cmd.motor_cmd[i].dq = 0.0
            
            if len(self._cmd.motor_cmd) > 29:
                self._cmd.motor_cmd[29].q = 1.0
            
            # === 新增：位置发布器（供Action Server订阅） ===
            self._left_pos_pub = self._node.create_publisher(
                Float64MultiArray, 
                '/g1/left_arm/position', 
                10
            )
            self._right_pos_pub = self._node.create_publisher(
                Float64MultiArray, 
                '/g1/right_arm/position', 
                10
            )
            
            # === 新增：阈值订阅器（从Action Server同步阈值） ===
            self._threshold_sub = self._node.create_subscription(
                Float64MultiArray,
                '/g1/arm/distance_threshold',
                self._threshold_callback,
                10
            )
            
            # === 订阅MCB权限状态（替代服务调用） ===
            self._mcb_authority_sub = self._node.create_subscription(
                Bool,
                '/g1/arm/mcb_control_authority',
                self._mcb_authority_callback,
                10
            )
            
            # 订阅急停状态
            self._estop_status_sub = self._node.create_subscription(
                Bool,
                '/g1/arm/emergency_stop',
                self._estop_status_callback,
                10
            )
            
            self._node.get_logger().info("[RobotBridge] ✓ 已订阅authority_server状态:")
            self._node.get_logger().info("  - /g1/arm/mcb_control_authority (MCB权限)")
            self._node.get_logger().info("  - /g1/arm/emergency_stop (急停状态)")
            self._node.get_logger().info("[RobotBridge] ✓ R键控制为本地管理，不经过服务器")
            
            # === 不再创建服务客户端（R键已本地化管理） ===
            # 旧代码：self._control_client = self._node.create_client(...)
            # 现在R键直接切换self._manual_control_enabled
            
            self._mode = "ros2"
            self.ok = True
            
            # 启动 spinner 线程
            self._spin_thread = threading.Thread(
                target=self._spin_node,
                daemon=True,
                name="rclpy_spinner"
            )
            self._spin_thread.start()
            
            print(f"[robot][ros2] ROS2 双臂模式激活 (V3 - Action集成版)")
            print(f"[robot][ros2] 发布: /arm_sdk (低层控制)")
            print(f"[robot][ros2] 发布: /g1/left_arm/position, /g1/right_arm/position (位置反馈)")
            print(f"[robot][ros2] 准备订阅: /g1/left_arm/goal, /g1/right_arm/goal (目标位置)")
            
        except ImportError as e:
            print(f"[robot] 缺少 ROS2 依赖: {e}")
            self.ok = False
        except Exception as e:
            print(f"[robot] ROS2 初始化失败: {e}")
            self.ok = False

    def enable_action_mode(self):
        """启用Action控制模式"""
        if not self.ok:
            return
        
        try:
            from std_msgs.msg import Float64MultiArray
            
            if self._left_goal_sub is None:
                self._left_goal_sub = self._node.create_subscription(
                    Float64MultiArray,
                    '/g1/left_arm/goal',
                    self._left_goal_callback,
                    10
                )
            
            if self._right_goal_sub is None:
                self._right_goal_sub = self._node.create_subscription(
                    Float64MultiArray,
                    '/g1/right_arm/goal',
                    self._right_goal_callback,
                    10
                )
            
            self._action_mode = True
            self._node.get_logger().info("[Action] Action控制模式已启用")
        except Exception as e:
            self._node.get_logger().error(f"[Action] 启用Action模式失败: {e}")
    
    def disable_action_mode(self):
        """禁用Action控制模式"""
        self._action_mode = False
        with self._goal_lock:
            self._external_left_goal = None
            self._external_right_goal = None
        self._node.get_logger().info("[Action] Action控制模式已禁用，切换到手动控制")
    
    def _mcb_authority_callback(self, msg):
        """订阅MCB控制权限状态回调"""
        with self._mcb_lock:
            old_state = self._mcb_control_authority
            self._mcb_control_authority = msg.data
            
            # 仅在状态变化时记录
            if old_state != msg.data:
                if msg.data:
                    self._node.get_logger().info("[MCB权限] ✓ MCB已授予控制权限")
                else:
                    self._node.get_logger().info("[MCB权限] ✗ MCB已释放控制权限")
    
    def trigger_manual_emergency_stop(self):
        """手动触发急停（Z键调用）"""
        with self._estop_lock:
            if not self.emergency_stop:
                self.emergency_stop = True
                self._last_estop_time = time.time()
                if self.ok:
                    self._node.get_logger().error("[Z键] 🚨 手动触发急停！")
                    # 发布急停消息到ROS2话题
                    try:
                        from robot_interfaces.msg import Estop
                        estop_msg = Estop()
                        estop_msg.emergency_stop = 'Y'
                        # 创建临时发布者发送急停消息
                        if not hasattr(self, '_manual_estop_pub'):
                            self._manual_estop_pub = self._node.create_publisher(Estop, '/msg_estop', 10)
                        self._manual_estop_pub.publish(estop_msg)
                    except Exception as e:
                        self._node.get_logger().warn(f"[Z键] 急停消息发布失败: {e}")
                else:
                    print("[Z键] 🚨 手动触发急停！")
                return True
        return False
    
    def release_manual_emergency_stop(self):
        """手动解除急停（Z键两次调用）"""
        with self._estop_lock:
            if self.emergency_stop:
                self.emergency_stop = False
                if self.ok:
                    self._node.get_logger().info("[Z键] ✓ 手动解除急停")
                    # 发布解除急停消息
                    try:
                        from robot_interfaces.msg import Estop
                        estop_msg = Estop()
                        estop_msg.emergency_stop = 'N'
                        if hasattr(self, '_manual_estop_pub'):
                            self._manual_estop_pub.publish(estop_msg)
                    except Exception as e:
                        self._node.get_logger().warn(f"[Z键] 解除急停消息发布失败: {e}")
                else:
                    print("[Z键] ✓ 手动解除急停")
                return True
        return False
    
    def _estop_status_callback(self, msg):
        """订阅急停状态回调"""
        with self._estop_lock:
            old_state = self.emergency_stop
            self.emergency_stop = msg.data
            
            # 仅在状态变化时记录
            if old_state != msg.data:
                if msg.data:
                    self._node.get_logger().error("[ESTOP] 🚨 急停激活！")
                    self._last_estop_time = time.time()
                else:
                    self._node.get_logger().info("[ESTOP] ✓ 急停解除")
    
    def toggle_manual_control(self):
        """R键切换手动控制开关（本地管理，不经过服务器）
        
        Returns:
            bool: 切换后的状态
        """
        with self._manual_lock:
            self._manual_control_enabled = not self._manual_control_enabled
            status_str = "启用" if self._manual_control_enabled else "禁用"
            if self.ok:
                self._node.get_logger().info(f"[R键] 手动控制已{status_str}")
            else:
                print(f"[R键] 手动控制已{status_str}")
            return self._manual_control_enabled
    
    def get_manual_control_status(self):
        """获取R键手动控制状态"""
        with self._manual_lock:
            return self._manual_control_enabled
    
    def get_mcb_control_status(self):
        """获取MCB控制权限状态"""
        with self._mcb_lock:
            return self._mcb_control_authority
    
    def has_arm_control_authority(self):
        """检查是否拥有arm控制权限
        
        新架构逻辑：控制权限 = R键使能 OR MCB授权
        两者权限等级相同，任意一个开启即可控制
        
        Returns:
            bool: True=有控制权限, False=无控制权限
        """
        manual_enabled = self.get_manual_control_status()
        mcb_enabled = self.get_mcb_control_status()
        return manual_enabled or mcb_enabled
    
    # ===== 旧的急停回调已删除，现在由权限管理服务器统一处理 =====
    
    def is_emergency_stop(self):
        """检查是否处于急停状态"""
        with self._estop_lock:
            return self.emergency_stop
    
    def get_estop_duration(self):
        """获取急停持续时间（秒）"""
        with self._estop_lock:
            if self.emergency_stop and self._last_estop_time > 0:
                return time.time() - self._last_estop_time
            return 0.0
    
    def _left_goal_callback(self, msg):
        """左臂目标位置回调"""
        if len(msg.data) >= 3:
            with self._goal_lock:
                self._external_left_goal = np.array(msg.data[:3], dtype=np.float32)
    
    def _right_goal_callback(self, msg):
        """右臂目标位置回调"""
        if len(msg.data) >= 3:
            with self._goal_lock:
                self._external_right_goal = np.array(msg.data[:3], dtype=np.float32)
    
    def _threshold_callback(self, msg):
        """距离阈值回调"""
        if len(msg.data) >= 1:
            with self._threshold_lock:
                self._dynamic_threshold = float(msg.data[0])
            self._node.get_logger().info(f"[Threshold] 更新阈值: {self._dynamic_threshold:.4f}m")
    
    def get_dynamic_threshold(self):
        """获取当前动态阈值
        
        Returns:
            float: 当前距离阈值（米）
        """
        with self._threshold_lock:
            return self._dynamic_threshold
    
    def get_external_goals(self):
        """获取外部目标（从Action Server）
        
        Returns:
            (left_goal, right_goal): 外部目标位置，None表示无新目标
        """
        with self._goal_lock:
            left = self._external_left_goal
            right = self._external_right_goal
            # 清空已读取的目标
            self._external_left_goal = None
            self._external_right_goal = None
            return left, right
    
    def publish_position(self, left_pos: np.ndarray, right_pos: np.ndarray):
        """发布当前位置到ROS2话题
        
        Args:
            left_pos: 左臂位置 [x, y, z]
            right_pos: 右臂位置 [x, y, z]
        """
        if not self.ok:
            return
        
        try:
            from std_msgs.msg import Float64MultiArray
            
            # 发布左臂位置
            left_msg = Float64MultiArray()
            left_msg.data = left_pos.tolist()
            self._left_pos_pub.publish(left_msg)
            
            # 发布右臂位置
            right_msg = Float64MultiArray()
            right_msg.data = right_pos.tolist()
            self._right_pos_pub.publish(right_msg)
            
        except Exception as e:
            self._node.get_logger().error(f"[Position] 发布位置失败: {e}")

    def _spin_node(self):
        """后台线程：处理 ROS2 回调"""
        thread_name = threading.current_thread().name
        print(f"[robot][ros2] ROS2回调处理线程已启动: {thread_name}")
        
        try:
            if not hasattr(self, '_rclpy') or not hasattr(self, '_node'):
                print(f"[robot][ros2] 警告: 缺少rclpy或node对象")
                return
            
            spin_count = 0
            while True:
                if not getattr(self, 'ok', False):
                    print(f"[robot][ros2] 回调线程正常退出 (spin_count={spin_count})")
                    break
                
                try:
                    if self._rclpy.ok():
                        # ⭐ 关键优化：减少超时时间到10ms，提高响应速度
                        # 这样可以更及时地处理服务回调，特别是控制权限请求
                        self._rclpy.spin_once(self._node, timeout_sec=0.01)
                        spin_count += 1
                    else:
                        print(f"[robot][ros2] rclpy.ok()返回False，退出回调线程")
                        break
                except self._rclpy.executors.ExternalShutdownException:
                    # 外部关闭信号，正常退出
                    print(f"[robot][ros2] 收到外部关闭信号，退出回调线程 (spin_count={spin_count})")
                    break
                except Exception as e:
                    # 其他异常不应导致线程退出，只记录日志并继续
                    if spin_count % 1000 == 0:  # 减少日志刷屏
                        print(f"[robot][ros2] spin_once异常 (已忽略，继续运行): {e}")
                    continue
                    
        except Exception as e:
            print(f"[robot][ros2] 回调线程异常退出: {e}")
            import traceback
            traceback.print_exc()

    def send_qpos(self, q: Dict[int, float]) -> None:
        """发送关节位置（双臂）
        
        特殊处理：左臂第5、7关节镜像右臂对应关节
        - 左臂第5关节（idx=19, left_wrist_roll）= -右臂第5关节（idx=26, right_wrist_roll）
        - 左臂第7关节（idx=21, left_wrist_yaw） = -右臂第7关节（idx=28, right_wrist_yaw）
        York, 20260210
        """
        if not self.ok:
            return

        # 镜像处理：第5和第7关节
        LEFT_WRIST_ROLL = 19   # 左臂第5关节索引
        RIGHT_WRIST_ROLL = 26  # 右臂第5关节索引
        LEFT_WRIST_YAW = 21    # 左臂第7关节索引
        RIGHT_WRIST_YAW = 28   # 右臂第7关节索引
        
        if RIGHT_WRIST_ROLL in q and LEFT_WRIST_ROLL in q:
            q[LEFT_WRIST_ROLL] = -q[RIGHT_WRIST_ROLL]
        
        if RIGHT_WRIST_YAW in q and LEFT_WRIST_YAW in q:
            q[LEFT_WRIST_YAW] = -q[RIGHT_WRIST_YAW]

        for idx, val in q.items():
            if idx >= 35:
                continue
            if idx < len(self._cmd.motor_cmd) and idx < 29:
                self._cmd.motor_cmd[idx].q = float(val)

        try:
            self._pub.publish(self._cmd)
        except Exception:
            self.ok = False

    def shutdown(self):
        """关闭 ROS2 资源"""
        if not hasattr(self, 'ok'):
            return
            
        self.ok = False
        
        if hasattr(self, "_spin_thread"):
            try:
                if self._spin_thread.is_alive():
                    self._spin_thread.join(timeout=2.0)
            except Exception:
                pass
        
        if hasattr(self, "_node"):
            try:
                self._node.destroy_node()
            except Exception:
                pass
        
        if hasattr(self, "_rclpy"):
            try:
                if self._rclpy.ok():
                    self._rclpy.shutdown()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# 3. 关节映射
# ---------------------------------------------------------------------------

JOINTS: List[Tuple[int, str, str]] = [
    (15, "左肩俯仰", "left_shoulder_pitch"),
    (16, "左肩翻滚", "left_shoulder_roll"),
    (17, "左肩偏航", "left_shoulder_yaw"),
    (18, "左肘", "left_elbow"),
    (19, "左腕翻滚", "left_wrist_roll"),
    (20, "左腕俯仰", "left_wrist_pitch"),
    (21, "左腕偏航", "left_wrist_yaw"),
    (22, "右肩俯仰", "right_shoulder_pitch"),
    (23, "右肩翻滚", "right_shoulder_roll"),
    (24, "右肩偏航", "right_shoulder_yaw"),
    (25, "右肘", "right_elbow"),
    (26, "右腕翻滚", "right_wrist_roll"),
    (27, "右腕俯仰", "right_wrist_pitch"),
    (28, "右腕偏航", "right_wrist_yaw"),
]


# ---------------------------------------------------------------------------
# 4. 渲染线程
# ---------------------------------------------------------------------------


def viewer_sync(env, stop_evt: threading.Event, speed, lock: threading.Lock):
    """渲染同步"""
    while not stop_evt.is_set():
        with lock:
            env.render()
        time.sleep(speed.dt)


# ---------------------------------------------------------------------------
# 5. 主控制循环
# ---------------------------------------------------------------------------


def run_curses(stdscr, env, left_policy, right_policy, robot: RobotBridgeROS2,
               control_state: dict, speed, lock: threading.Lock):
    """双臂交互控制循环（支持Action模式和MCB控制权限）"""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    STEP = 0.015  # 目标位置步进
    
    # 应用初始姿态
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

                # 将目标移动到当前手掌位置
                try:
                    left_hand = env._fk_left()
                    env.set_left_goal(left_hand)

                    right_hand = env._fk_right()
                    env.set_right_goal(right_hand)
                except Exception as e:
                    print(f"[init] 目标位置初始化失败: {e}")

        apply_pose()
        
    except Exception as e:
        print(f"[init] 初始姿态应用失败: {e}")
    
    # 初始观测
    with lock:
        left_obs = env.get_left_obs()
        right_obs = env.get_right_obs()
    
    # 目标位置
    left_goal = env.left_goal.copy()
    right_goal = env.right_goal.copy()
    
    last_robot_send = 0.0
    last_pos_publish = 0.0
    
    # 保持模式标志
    left_hold = False
    right_hold = False
    left_collision_freeze = False
    right_collision_freeze = False
    
    # Action模式标志
    action_mode = False
    
    # MCB状态追踪（用于检测释放事件）
    last_mcb_status = robot.get_mcb_control_status()

    while True:
        # === MCB权限监控：检测MCB授权/释放事件 ===
        current_mcb_status = robot.get_mcb_control_status()
        
        # MCB授权事件：自动启用机器人输出
        if not last_mcb_status and current_mcb_status:
            control_state['out_robot'] = robot.ok
            print("\n[MCB授权] MCB已授权，自动启用机器人输出")
            try:
                stdscr.addstr(10, 0, "✓ MCB已授权，机器人输出已自动启用" + " " * 30)
                stdscr.refresh()
            except Exception:
                pass
        
        # MCB释放事件：强制关闭输出（忽略R键状态）
        elif last_mcb_status and not current_mcb_status:
            control_state['out_robot'] = False
            print("\n[MCB释放] MCB已释放控制权限，强制关闭机器人输出")
            try:
                stdscr.addstr(10, 0, "⚠️  MCB已释放控制权限，机器人输出已强制关闭" + " " * 30)
                stdscr.refresh()
            except Exception:
                pass
        
        last_mcb_status = current_mcb_status
        
        # 键盘输入
        ch = stdscr.getch()
        if ch != -1:
            key = chr(ch & 0xFF)
            if key in ("x", "\x1b"):
                break
            # Z键：手动急停切换
            elif key in ("z", "Z"):
                if robot.is_emergency_stop():
                    # 当前是急停状态，解除急停
                    robot.release_manual_emergency_stop()
                    try:
                        stdscr.addstr(10, 0, "[Z键] ✓ 手动解除急停" + " " * 50)
                        stdscr.refresh()
                    except Exception:
                        pass
                else:
                    # 当前非急停状态，触发急停
                    robot.trigger_manual_emergency_stop()
                    try:
                        stdscr.addstr(10, 0, "[Z键] 🚨 手动触发急停！所有控制已冻结" + " " * 30)
                        stdscr.refresh()
                    except Exception:
                        pass
                time.sleep(0.3)  # 防止连续触发
            # 模式切换
            elif key == "m":
                action_mode = not action_mode
                if action_mode:
                    robot.enable_action_mode()
                else:
                    robot.disable_action_mode()
            # 左臂控制 (W/A/S/D/Q/E) - 仅手动模式
            elif not action_mode:
                if key == "w":
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
                # 右臂控制 (I/K/J/L/U/O)
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
            # R键：独立手动控制（忽略MCB状态）
            if key in ("r", "R"):
                new_status = robot.toggle_manual_control()
                status_msg = "启用" if new_status else "禁用"
                print(f"\n[R键] 手动控制已{status_msg}")
                
                # R键启用/禁用时，自动控制机器人输出
                if new_status:
                    # R键启用：自动开启机器人输出（忽略MCB状态）
                    control_state['out_robot'] = robot.ok
                    print(f"[R键] 已自动开启机器人输出 (忽略MCB状态)")
                else:
                    # R键禁用：自动关闭机器人输出
                    control_state['out_robot'] = False
                    print("[R键] 已关闭机器人输出")
            elif key == "t":
                control_state['out_sim'] = not control_state['out_sim']
            elif key == "]":
                speed.dt = max(0.005, speed.dt * 0.5)
            elif key == "[":
                speed.dt = min(0.5, speed.dt * 2.0)
            elif key in (">", "."):
                speed.mult = min(4.0, speed.mult * 2.0)
            elif key in ("<", ","):
                speed.mult = max(0.25, speed.mult * 0.5)

        # === 新增：急停检查（最高优先级）===
        if robot.is_emergency_stop():
            # 急停状态：强制关闭所有输出，清除目标
            control_state['out_robot'] = False
            control_state['out_sim'] = False  # 连仿真also停止
            # 发送零速度命令锁定位置
            if robot.ok:
                try:
                    # 发送当前位置作为目标（保持不动）
                    with lock:
                        left_pos = env.get_left_palm_pos()
                        right_pos = env.get_right_palm_pos()
                    env.set_left_goal(left_pos)
                    env.set_right_goal(right_pos)
                except Exception:
                    pass
        
        # === 权限检查（仅用于UI显示，不影响输出控制） ===
        # 注意：out_robot的控制已由MCB授权/释放事件和R键直接管理
        # 这里不再强制修改out_robot状态
        
        # 使用 control_state 字典中的值
        out_sim = control_state['out_sim']
        out_robot = control_state['out_robot']
        
        # === Action模式：接收外部目标（需要MCB权限且无急停） ===
        if action_mode and robot._action_mode:
            if robot.is_emergency_stop():
                # 急停状态：拒绝所有外部目标
                try:
                    estop_duration = robot.get_estop_duration()
                    stdscr.addstr(8, 0, f"🚨 急停状态: 已持续 {estop_duration:.1f}秒，所有控制已冻结                    ")
                except Exception:
                    pass
            elif not robot.has_arm_control_authority():
                # MCB未授权时，拒绝接收外部目标
                try:
                    stdscr.addstr(8, 0, "⚠ 警告: Action模式需要MCB授权，当前目标被忽略                    ")
                except Exception:
                    pass
            else:
                # 有授权且无急停，正常接收目标
                ext_left, ext_right = robot.get_external_goals()
                if ext_left is not None:
                    left_goal = ext_left.copy()
                    left_hold = False
                    left_collision_freeze = False
                if ext_right is not None:
                    right_goal = ext_right.copy()
                    right_hold = False
                    right_collision_freeze = False

        # 限制目标范围
        left_goal = np.clip(left_goal, [0.0, 0.0, 0.55], [0.6, 0.6, 1.45])
        right_goal = np.clip(right_goal, [0.0, -0.6, 0.55], [0.6, 0.0, 1.45])

        # 更新目标
        env.set_left_goal(left_goal)
        env.set_right_goal(right_goal)

        # === 发布当前位置（10Hz） ===
        if time.time() - last_pos_publish > 0.1:
            with lock:
                left_pos = env.get_left_palm_pos()
                right_pos = env.get_right_palm_pos()
            robot.publish_position(left_pos, right_pos)
            last_pos_publish = time.time()

        if out_sim or out_robot:
            # 左臂控制
            with lock:
                left_dist = np.linalg.norm(env.left_goal - env.get_left_palm_pos())
            
            if left_collision_freeze:
                left_hold = True
            else:
                current_threshold = robot.get_dynamic_threshold() if robot.ok else DISTANCE_THRESHOLD
                if not left_hold and left_dist < current_threshold:
                    left_hold = True
                elif left_hold and left_dist >= DISTANCE_THRESHOLD:
                    left_hold = False
                    left_collision_freeze = False
            
            if left_hold:
                with lock:
                    env.step_left(np.zeros(7, dtype=np.float32))
                env.left_step_count = 0
            else:
                with lock:
                    left_obs = env.get_left_obs()
                left_action, _ = left_policy.predict(left_obs, deterministic=True)
                left_action = np.clip(left_action * speed.mult, -0.05, 0.05)
                with lock:
                    left_obs, _, left_done, _, left_info = env.step_left(left_action)
                    
                    if left_info.get("collision", False):
                        left_collision_freeze = True
                    
                    if left_done and not left_collision_freeze:
                        left_obs = env.reset_left_arm()
                        env.set_left_goal(left_goal)
            
            # 右臂控制
            with lock:
                right_dist = np.linalg.norm(env.right_goal - env.get_right_palm_pos())
            
            if right_collision_freeze:
                right_hold = True
            else:
                current_threshold = robot.get_dynamic_threshold() if robot.ok else DISTANCE_THRESHOLD
                if not right_hold and right_dist < current_threshold:
                    right_hold = True
                elif right_hold and right_dist >= DISTANCE_THRESHOLD:
                    right_hold = False
                    right_collision_freeze = False
            
            if right_hold:
                with lock:
                    env.step_right(np.zeros(7, dtype=np.float32))
                env.right_step_count = 0
            else:
                with lock:
                    right_obs = env.get_right_obs()
                right_action, _ = right_policy.predict(right_obs, deterministic=True)
                right_action = np.clip(right_action * speed.mult, -0.05, 0.05)
                with lock:
                    right_obs, _, right_done, _, right_info = env.step_right(right_action)
                    
                    if right_info.get("collision", False):
                        right_collision_freeze = True
                    
                    if right_done and not right_collision_freeze:
                        right_obs = env.reset_right_arm()
                        env.set_right_goal(right_goal)

        # 发送到机器人
        if out_robot and robot.ok and (time.time() - last_robot_send) > 0.02:
            if not hasattr(env, "_motor_qadr"):
                import mujoco as _mj
                qadr = {}
                for idx, _lbl, mj_short in JOINTS:
                    jname = mj_short + "_joint"
                    jid = _mj.mj_name2id(env.model, _mj.mjtObj.mjOBJ_JOINT, jname)
                    if jid != -1:
                        qadr[idx] = int(env.model.jnt_qposadr[jid])
                env._motor_qadr = qadr

            qpos = {idx: float(env.data.qpos[adr]) for idx, adr in env._motor_qadr.items()}
            robot.send_qpos(qpos)
            last_robot_send = time.time()

        # UI
        stdscr.erase()
        
        # 标题栏（如果急停则高亮显示）
        if robot.is_emergency_stop():
            try:
                stdscr.addstr(0, 0, "🚨 急停状态 🚨 G-1 双臂 RL 控制器 V3 (MCB集成) – 所有控制已冻结！", curses.A_REVERSE | curses.A_BOLD)
            except Exception:
                stdscr.addstr(0, 0, "[ESTOP] G-1 双臂 RL 控制器 V3 (MCB集成) – 所有控制已冻结！")
        else:
            stdscr.addstr(0, 0, "G-1 双臂 RL 控制器 V3 (MCB集成) – [R]手动控制 [Z]急停 [M]Action模式 [T]仿真 [Esc]退出")
        
        # 显示权限和急停状态
        manual_status = '✓' if robot.get_manual_control_status() else ' '
        mcb_status = '✓' if robot.get_mcb_control_status() else ' '
        final_auth = '✓' if robot.has_arm_control_authority() else ' '
        estop_status = '🚨' if robot.is_emergency_stop() else ' '
        
        # 第一行：基本状态
        stdscr.addstr(1, 0,
                     f"控制模式:[{'ACTION' if action_mode else '手动'}]  "
                     f"急停:[{estop_status}]  "
                     f"R键:[{manual_status}]  "
                     f"MCB:[{mcb_status}]  "
                     f"最终权限:[{final_auth}]  "
                     f"输出→仿真:[{'X' if out_sim else ' '}]  "
                     f"机器人:[{'X' if out_robot else '-'}]  "
                     f"速率:{speed.dt*1000:.0f}ms  倍数:{speed.mult:.2f}")
        
        # 第二行：权限提示
        if not robot.has_arm_control_authority():
            stdscr.addstr(2, 0, 
                         "⚠️  无控制权限！按R键启用手动控制，或等待MCB授权" + " " * 30,
                         curses.A_BOLD | curses.color_pair(3) if hasattr(curses, 'color_pair') else 0)
        elif robot.is_emergency_stop():
            stdscr.addstr(2, 0, 
                         "⚠️  急停状态！所有输出已冻结" + " " * 40,
                         curses.A_BOLD | curses.color_pair(1) if hasattr(curses, 'color_pair') else 0)
        elif not out_robot and robot.has_arm_control_authority():
            stdscr.addstr(2, 0, 
                         "✓ 已有控制权限，但机器人输出未开启（按R启用）" + " " * 30)
        else:
            manual_src = "R键" if robot.get_manual_control_status() else ""
            mcb_src = "MCB" if robot.get_mcb_control_status() else ""
            auth_sources = "+".join(filter(None, [manual_src, mcb_src]))
            stdscr.addstr(2, 0, 
                         f"✓ 控制权限正常 (来源:{auth_sources})" + " " * 50)
        
        try:
            with lock:
                cur_left_dist = np.linalg.norm(env.left_goal - env.get_left_palm_pos())
                cur_right_dist = np.linalg.norm(env.right_goal - env.get_right_palm_pos())
            # 将距离显示移到第4行，避免与权限提示冲突
            stdscr.addstr(4, 0, f"距离 → 左:{cur_left_dist:.4f}m 右:{cur_right_dist:.4f}m  阈值:{DISTANCE_THRESHOLD}m")
        except Exception:
            pass

        # 臂状态（调整到第5-6行）
        left_status = "保持" if left_hold else "运行"
        if left_collision_freeze:
            left_status = "碰撞"
        
        right_status = "保持" if right_hold else "运行"
        if right_collision_freeze:
            right_status = "碰撞"

        stdscr.addstr(5, 0, f"左臂目标: x={left_goal[0]:.3f} y={left_goal[1]:.3f} z={left_goal[2]:.3f}  模式:{left_status}")
        stdscr.addstr(6, 0, f"右臂目标: x={right_goal[0]:.3f} y={right_goal[1]:.3f} z={right_goal[2]:.3f}  模式:{right_status}")

        try:
            left_palm = env.get_left_palm_pos()
            right_palm = env.get_right_palm_pos()
            # 将手掌位置显示移到第7-8行
            stdscr.addstr(7, 0, f"左手掌: x={left_palm[0]:.3f} y={left_palm[1]:.3f} z={left_palm[2]:.3f}")
            stdscr.addstr(8, 0, f"右手掌: x={right_palm[0]:.3f} y={right_palm[1]:.3f} z={right_palm[2]:.3f}")
        except Exception:
            pass
        
        if action_mode:
            # Action模式提示移到第9行
            stdscr.addstr(9, 0, "提示: Action模式下，目标由Action Server控制，键盘WASD等无效")

        stdscr.refresh()
        time.sleep(speed.dt)


# ---------------------------------------------------------------------------
# 6. 主函数
# ---------------------------------------------------------------------------


def main() -> None:
    """主函数"""
    ap = argparse.ArgumentParser(description="G-1 双臂 RL 控制器 V3 (Action集成版)")

    # 使用ROS2包查找机制定位模型文件（支持多路径搜索）
    script_dir = pathlib.Path(__file__).resolve().parent
    source_package_dir = script_dir.parent  # 源码目录
    
    # 候选路径列表（优先源码目录，因为模型通常不会被install复制）
    candidate_dirs = [source_package_dir]
    
    try:
        from ament_index_python.packages import get_package_share_directory
        install_package_dir = pathlib.Path(get_package_share_directory('arm_controller'))
        candidate_dirs.append(install_package_dir)  # 添加安装目录作为备选
    except Exception:
        pass
    
    # 搜索左臂模型
    # left_model_subpath = "models/ppo_left_5M_Feb10_Cons01/ppo_g1_left_5000k.zip"
    left_model_subpath = "models/ppo_left_5M_Feb09/20260209-12310-lArm/ppo_g1_left_5000k.zip"

    default_left_model = None
    for base_dir in candidate_dirs:
        left_path = base_dir / left_model_subpath
        if left_path.exists():
            default_left_model = str(left_path)
            print(f"[main] 找到左臂模型: {left_path}")
            break
    
    if default_left_model is None:
        # 兜底：尝试旧模型路径
        old_left_subpath = "models/ppo_left_5M_Feb09/20260209-12310-lArm/ppo_g1_left_5000k.zip"
        for base_dir in candidate_dirs:
            left_path = base_dir / old_left_subpath
            if left_path.exists():
                default_left_model = str(left_path)
                print(f"[main] 使用旧版左臂模型: {left_path}")
                break
    
    # 搜索右臂模型
    right_model_subpath = "models/ppo_right_5M_Aug28/ppo_g1_right_5000k.zip"
    default_right_model = None
    for base_dir in candidate_dirs:
        right_path = base_dir / right_model_subpath
        if right_path.exists():
            default_right_model = str(right_path)
            print(f"[main] 找到右臂模型: {right_path}")
            break
    
    # 如果仍未找到，使用默认路径（会在后续检查中报错）
    if default_left_model is None:
        default_left_model = str(source_package_dir / left_model_subpath)
        print(f"[main] 警告: 未找到左臂模型，使用默认路径: {default_left_model}")
    if default_right_model is None:
        default_right_model = str(source_package_dir / right_model_subpath)
        print(f"[main] 警告: 未找到右臂模型，使用默认路径: {default_right_model}")
    
    ap.add_argument(
        "--left-model",
        default=default_left_model,
        help="左臂模型路径")
    ap.add_argument(
        "--right-model",
        default=default_right_model,
        help="右臂模型路径")
    ap.add_argument(
        "--rate",
        type=float,
        default=0.04,
        help="控制循环间隔 (秒)")

    args = ap.parse_args()

    # 加载策略
    left_model_path = pathlib.Path(args.left_model).expanduser()
    right_model_path = pathlib.Path(args.right_model).expanduser()
    
    if not left_model_path.exists():
        raise SystemExit(f"左臂模型未找到: {left_model_path}")
    if not right_model_path.exists():
        raise SystemExit(f"右臂模型未找到: {right_model_path}")

    left_policy, _ = load_policy(left_model_path)
    right_policy, _ = load_policy(right_model_path)
    
    print(f"[main] 已加载左臂策略")
    print(f"[main] 已加载右臂策略")

    # 创建环境
    env = DualArmEnv(render=True)

    # 创建 ROS2 桥接器
    robot = RobotBridgeROS2()

    # 创建控制状态字典（使用可变对象在函数间共享状态）
    control_state = {
        'out_sim': True,
        'out_robot': False
    }

    speed = SimpleNamespace(dt=max(0.01, args.rate), mult=0.5)

    lock = threading.Lock()
    stop_evt = threading.Event()
    threading.Thread(target=viewer_sync, args=(env, stop_evt, speed, lock), daemon=True).start()

    try:
        curses.wrapper(run_curses, env, left_policy, right_policy, robot, control_state, speed, lock)
    except KeyboardInterrupt:
        print("\n[main] 键盘中断")
    except Exception as e:
        print(f"\n[main] 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("[main] 清理资源...")
        try:
            stop_evt.set()
        except Exception:
            pass

        try:
            robot.shutdown()
        except Exception:
            pass

        try:
            env.close()
        except Exception:
            pass

        print("[main] 清理完成")


if __name__ == "__main__":
    main()

