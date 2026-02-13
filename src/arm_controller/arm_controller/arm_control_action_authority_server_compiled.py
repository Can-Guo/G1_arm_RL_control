#!/usr/bin/env python3
"""arm_control_action_authority_server_compiled.py - G1双臂控制统一服务器

功能描述：
  整合了权限管理和Action服务功能的统一服务器，减少终端启动数量。
  
核心功能：
  1. MCB控制权限管理（通过ControlPower服务）
  2. 急停监听与状态广播
  3. 双臂控制Action服务（带权限验证）
  4. 实时位置反馈与目标设置

权限逻辑：
  - 仅当MCB有授权且无急停时，才接受Action请求
  - 急停时自动中止所有正在执行的Action
  - 急停触发时自动释放MCB权限

服务接口：
  - /srv_control_power_transport (ControlPower) - MCB控制权限请求

Action接口：
  - action_arm (ArmControl) - 双臂控制Action

订阅话题：
  - /msg_estop (Estop) - 急停信号
  - /g1/left_arm/position (Float64MultiArray) - 左臂位置反馈
  - /g1/right_arm/position (Float64MultiArray) - 右臂位置反馈

发布话题：
  - /g1/arm/mcb_control_authority (Bool) - MCB控制权限状态
  - /g1/arm/emergency_stop (Bool) - 急停状态
  - /g1/left_arm/goal (Float64MultiArray) - 左臂目标位置
  - /g1/right_arm/goal (Float64MultiArray) - 右臂目标位置
  - /g1/arm/distance_threshold (Float64MultiArray) - 距离阈值

运行方式：
  ros2 run arm_controller arm_control_action_authority_server_compiled.py

作者: York
日期: 2026-02-13
版本: 1.0 (统一服务器)
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

# 导入距离阈值
try:
    from arm_controller import DISTANCE_THRESHOLD
except (ImportError, AttributeError):
    DISTANCE_THRESHOLD = 0.03
    import sys
    print(f"[提示] 使用默认距离阈值: {DISTANCE_THRESHOLD}m", file=sys.stderr)

# 导入消息和服务接口
try:
    from robot_interfaces.action import ArmControl
    from robot_interfaces.srv import ControlPower
    from robot_interfaces.msg import Estop
    INTERFACES_AVAILABLE = True
except ImportError as e:
    print(f"[错误] 无法导入接口: {e}")
    print("[提示] 请先编译 robot_interfaces 包:")
    print("       cd /home/york/RL_arm_control")
    print("       colcon build --packages-select robot_interfaces")
    print("       source install/setup.bash")
    INTERFACES_AVAILABLE = False


# ---------------------------------------------------------------------------
# 双臂控制器接口
# ---------------------------------------------------------------------------
class DualArmController:
    """双臂控制器接口 - 与g1_dual_arm_policy_controller_ROS2_v3通信"""
    
    def __init__(self, node: Node):
        """初始化控制器接口
        
        Args:
            node: ROS2节点，用于创建发布者/订阅者
        """
        self.node = node
        self._lock = threading.Lock()
        
        # 当前目标位置
        self.left_goal = np.array([0.1, 0.25, 0.70], dtype=np.float32)
        self.right_goal = np.array([0.1, -0.25, 0.70], dtype=np.float32)
        
        # 当前实际位置（从反馈获取）
        self.left_pos = self.left_goal.copy()
        self.right_pos = self.right_goal.copy()
        
        # 到达标志
        self.left_reached = False
        self.right_reached = False
        
        # 距离阈值（米）
        self.distance_threshold = DISTANCE_THRESHOLD
        
        # 创建发布者（向控制器发送目标）
        self._left_goal_pub = node.create_publisher(
            Float64MultiArray, 
            '/g1/left_arm/goal', 
            10
        )
        self._right_goal_pub = node.create_publisher(
            Float64MultiArray, 
            '/g1/right_arm/goal', 
            10
        )
        
        # 创建订阅者（接收当前位置反馈）
        self._left_pos_sub = node.create_subscription(
            Float64MultiArray,
            '/g1/left_arm/position',
            self._left_pos_callback,
            10
        )
        self._right_pos_sub = node.create_subscription(
            Float64MultiArray,
            '/g1/right_arm/position',
            self._right_pos_callback,
            10
        )
        
        # 创建阈值发布者（向控制器同步阈值）
        self._threshold_pub = node.create_publisher(
            Float64MultiArray,
            '/g1/arm/distance_threshold',
            10
        )
        
        self.node.get_logger().info("[DualArmController] 初始化完成")
    
    def _left_pos_callback(self, msg):
        """左臂位置反馈回调"""
        with self._lock:
            if len(msg.data) >= 3:
                self.left_pos = np.array(msg.data[:3], dtype=np.float32)
    
    def _right_pos_callback(self, msg):
        """右臂位置反馈回调"""
        with self._lock:
            if len(msg.data) >= 3:
                self.right_pos = np.array(msg.data[:3], dtype=np.float32)
    
    def set_goal(self, left_xyz: np.ndarray, right_xyz: np.ndarray):
        """设置双臂目标位置
        
        Args:
            left_xyz: 左臂目标 [x, y, z]
            right_xyz: 右臂目标 [x, y, z]
        """
        with self._lock:
            self.left_goal = np.array(left_xyz, dtype=np.float32)
            self.right_goal = np.array(right_xyz, dtype=np.float32)
            self.left_reached = False
            self.right_reached = False
        
        # 发布目标
        left_msg = Float64MultiArray()
        left_msg.data = self.left_goal.tolist()
        self._left_goal_pub.publish(left_msg)
        
        right_msg = Float64MultiArray()
        right_msg.data = self.right_goal.tolist()
        self._right_goal_pub.publish(right_msg)
        
        self.node.get_logger().info(
            f"[DualArmController] 目标设置 - 左臂:{self.left_goal} 右臂:{self.right_goal}"
        )
    
    def get_current_position(self):
        """获取当前位置
        
        Returns:
            (left_pos, right_pos): 左右臂当前位置
        """
        with self._lock:
            return self.left_pos.copy(), self.right_pos.copy()
    
    def stop_at_current_position(self):
        """停止机械臂：发送当前位置作为目标（保持不动）"""
        with self._lock:
            current_left = self.left_pos.copy()
            current_right = self.right_pos.copy()
        
        # 发布当前位置作为新目标，使机械臂停止移动
        left_msg = Float64MultiArray()
        left_msg.data = current_left.tolist()
        self._left_goal_pub.publish(left_msg)
        
        right_msg = Float64MultiArray()
        right_msg.data = current_right.tolist()
        self._right_goal_pub.publish(right_msg)
        
        self.node.get_logger().info(
            f"[DualArmController] 停止命令 - 保持当前位置 "
            f"左臂:{current_left} 右臂:{current_right}"
        )
    
    def check_reached(self):
        """检查是否到达目标
        
        Returns:
            (left_reached, right_reached, both_reached): 到达状态
        """
        with self._lock:
            left_dist = np.linalg.norm(self.left_goal - self.left_pos)
            right_dist = np.linalg.norm(self.right_goal - self.right_pos)
            
            self.left_reached = left_dist < self.distance_threshold
            self.right_reached = right_dist < self.distance_threshold
            both_reached = self.left_reached and self.right_reached
            
            return self.left_reached, self.right_reached, both_reached
    
    def set_distance_threshold(self, threshold: float):
        """动态设置距离阈值
        
        Args:
            threshold: 新的距离阈值（米）
        """
        with self._lock:
            self.distance_threshold = threshold
        
        # 发布阈值到控制器
        msg = Float64MultiArray()
        msg.data = [float(threshold)]
        self._threshold_pub.publish(msg)
        
        self.node.get_logger().info(
            f"[DualArmController] 距离阈值已更新并发布: {threshold:.4f}m"
        )


# ---------------------------------------------------------------------------
# 统一服务器节点
# ---------------------------------------------------------------------------
class UnifiedArmControlServer(Node):
    """统一的机械臂控制服务器（权限管理 + Action服务）"""
    
    def __init__(self):
        super().__init__('unified_arm_control_server')
        
        if not INTERFACES_AVAILABLE:
            raise RuntimeError("接口未编译，请先编译 robot_interfaces 包")
        
        # ==================== 状态管理（线程安全） ====================
        self._mcb_control_authority = False  # MCB控制权限
        self._emergency_stop = False         # 急停状态
        self._authority_lock = threading.Lock()
        self._estop_lock = threading.Lock()
        
        # 当前正在执行的Action目标句柄（用于急停时中止）
        self._current_goal_handle = None
        self._goal_handle_lock = threading.Lock()
        
        # ==================== 初始化双臂控制器 ====================
        self.controller = DualArmController(self)
        
        # ==================== 创建权限管理服务 ====================
        self._mcb_control_srv = self.create_service(
            ControlPower,
            '/srv_control_power_transport',
            self._handle_mcb_control_request
        )
        
        # ==================== 订阅急停话题 ====================
        self._estop_sub = self.create_subscription(
            Estop,
            '/msg_estop',
            self._estop_callback,
            10
        )
        
        # ==================== 发布状态话题 ====================
        self._mcb_authority_pub = self.create_publisher(
            Bool,
            '/g1/arm/mcb_control_authority',
            10
        )
        self._estop_pub = self.create_publisher(
            Bool,
            '/g1/arm/emergency_stop',
            10
        )
        
        # 定时发布状态（10Hz）
        self._status_timer = self.create_timer(0.1, self._publish_status)
        
        # ==================== 创建Action Server ====================
        self._action_server = ActionServer(
            self,
            ArmControl,
            'action_arm',
            execute_callback=self._execute_action_callback,
            goal_callback=self._action_goal_callback,
            cancel_callback=self._action_cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # ==================== 启动完成 ====================
        self.get_logger().info("=" * 70)
        self.get_logger().info("统一机械臂控制服务器已启动")
        self.get_logger().info("-" * 70)
        self.get_logger().info("权限管理服务:")
        self.get_logger().info("  - /srv_control_power_transport (ControlPower) - MCB权限请求")
        self.get_logger().info("Action服务:")
        self.get_logger().info("  - action_arm (ArmControl) - 双臂控制Action")
        self.get_logger().info("订阅话题:")
        self.get_logger().info("  - /msg_estop (Estop) - 急停信号")
        self.get_logger().info("  - /g1/left_arm/position, /g1/right_arm/position - 位置反馈")
        self.get_logger().info("发布话题:")
        self.get_logger().info("  - /g1/arm/mcb_control_authority (Bool) - MCB权限状态")
        self.get_logger().info("  - /g1/arm/emergency_stop (Bool) - 急停状态")
        self.get_logger().info("  - /g1/left_arm/goal, /g1/right_arm/goal - 目标位置")
        self.get_logger().info("-" * 70)
        self.get_logger().info("初始状态: MCB权限=False, 急停=False")
        self.get_logger().info("=" * 70)
    
    # ========================================================================
    # 权限管理相关方法
    # ========================================================================
    
    def _handle_mcb_control_request(self, request, response):
        """处理MCB控制权限请求
        
        请求参数：
          - task_id: 任务ID
          - object: 控制对象 (arm/hand)
          - type: 操作类型 (get/release)
        
        返回值：
          - result: success/failed
          - error: 错误信息
        """
        self.get_logger().info(
            f"[MCB权限] 收到请求: task_id={request.task_id}, "
            f"object={request.object}, type={request.type}"
        )
        
        try:
            # 1. 验证对象类型
            if request.object.lower() == "hand":
                response.result = "failed"
                response.error = "hand control ignored (not supported)"
                self.get_logger().info(f"[MCB权限] 忽略hand控制请求")
                return response
            
            if request.object.lower() != "arm":
                response.result = "failed"
                response.error = f"invalid object: {request.object}"
                self.get_logger().warn(f"[MCB权限] 拒绝无效对象: {request.object}")
                return response
            
            # 2. 处理get/release请求
            with self._authority_lock:
                if request.type.lower() == "get":
                    # MCB请求获取控制权限
                    self._mcb_control_authority = True
                    response.result = "success"
                    response.error = ""
                    self.get_logger().info(
                        f"[MCB权限] ✓ 授予控制权限 (task_id={request.task_id})"
                    )
                    
                elif request.type.lower() == "release":
                    # MCB请求释放控制权限
                    self._mcb_control_authority = False
                    response.result = "success"
                    response.error = ""
                    self.get_logger().info(
                        f"[MCB权限] ✗ 释放控制权限 (task_id={request.task_id})"
                    )
                    
                else:
                    # 无效的操作类型
                    response.result = "failed"
                    response.error = f"invalid type: {request.type}"
                    self.get_logger().warn(f"[MCB权限] 拒绝无效类型: {request.type}")
            
            # 3. 记录到文件（调试用）
            self._log_to_file("MCB", request, response)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"[MCB权限] 处理请求时出错: {e}")
            import traceback
            traceback.print_exc()
            
            response.result = "failed"
            response.error = f"internal error: {str(e)}"
            return response
    
    def _estop_callback(self, msg):
        """急停消息回调
        
        消息格式：
          - emergency_stop: "Y"（急停）/ "N"（解除）
        """
        with self._estop_lock:
            if msg.emergency_stop.upper() == "Y":
                if not self._emergency_stop:
                    # 触发急停
                    self._emergency_stop = True
                    self.get_logger().error("[急停] 🚨 收到急停信号！所有控制已冻结！")
                    
                    # 急停时自动释放MCB控制权限（安全措施）
                    with self._authority_lock:
                        if self._mcb_control_authority:
                            self._mcb_control_authority = False
                            self.get_logger().warn("[急停] ⚠️  自动释放MCB控制权限")
                    
                    # 停止机械臂
                    try:
                        self.controller.stop_at_current_position()
                    except Exception as e:
                        self.get_logger().error(f"[急停] 停止机械臂失败: {e}")
                    
                    # 中止当前正在执行的Action
                    with self._goal_handle_lock:
                        if self._current_goal_handle is not None:
                            try:
                                self._current_goal_handle.abort()
                                self.get_logger().warn("[急停] 已中止当前Action任务")
                            except Exception as e:
                                self.get_logger().error(f"[急停] 中止Action失败: {e}")
                            
            elif msg.emergency_stop.upper() == "N":
                if self._emergency_stop:
                    # 解除急停
                    self._emergency_stop = False
                    self.get_logger().info("[急停] ✓ 急停解除，系统恢复正常")
                    self.get_logger().info("[急停]   （MCB需重新请求权限）")
            else:
                self.get_logger().warn(f"[急停] 收到未知急停信号: {msg.emergency_stop}")
    
    def _publish_status(self):
        """定时发布MCB权限和急停状态（10Hz）"""
        # 发布MCB控制权限状态
        mcb_authority_msg = Bool()
        with self._authority_lock:
            mcb_authority_msg.data = self._mcb_control_authority
        self._mcb_authority_pub.publish(mcb_authority_msg)
        
        # 发布急停状态
        estop_msg = Bool()
        with self._estop_lock:
            estop_msg.data = self._emergency_stop
        self._estop_pub.publish(estop_msg)
    
    def _log_to_file(self, source, request, response):
        """记录控制权限变更到文件（调试用）"""
        try:
            import datetime
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            log_msg = (
                f"[{timestamp}] [{source}] "
                f"task_id={request.task_id}, object={request.object}, type={request.type} "
                f"-> result={response.result}, error={response.error}\n"
            )
            with open("/tmp/arm_control_authority.log", "a") as f:
                f.write(log_msg)
        except Exception:
            pass
    
    def _has_control_authority(self):
        """检查是否拥有控制权限（MCB授权且无急停）
        
        Returns:
            (bool, str): (是否有权限, 错误信息)
        """
        with self._estop_lock:
            if self._emergency_stop:
                return False, "emergency stop is active"
        
        with self._authority_lock:
            if not self._mcb_control_authority:
                return False, "no MCB control authority"
        
        return True, ""
    
    # ========================================================================
    # Action服务相关方法
    # ========================================================================
    
    def _action_goal_callback(self, goal_request):
        """接收新Action目标时的回调"""
        # 构造友好的动作描述
        action_desc = {
            'pick': '抓取',
            'place': '放置',
            'move': '移动'
        }.get(goal_request.action, goal_request.action)
        
        target_desc = {
            'empty_box': '空箱',
            'product_box': '产品箱'
        }.get(goal_request.target, goal_request.target or '自定义位置')
        
        # 精度模式描述
        resolution_desc = {
            1: '高精度(1x)',
            2: '中精度(2x)',
            3: '低精度(3x)'
        }.get(goal_request.high_resolution, f'自定义({goal_request.high_resolution}x)')
        
        self.get_logger().info(
            f"[Action目标] 收到新请求: {action_desc} {target_desc}\n"
            f"  Target: {goal_request.target or '(无)'}\n"
            f"  Action: {goal_request.action}\n"
            f"  精度模式: {resolution_desc}\n"
            f"  左臂目标: ({goal_request.x_l:.3f}, {goal_request.y_l:.3f}, {goal_request.z_l:.3f})\n"
            f"  右臂目标: ({goal_request.x_r:.3f}, {goal_request.y_r:.3f}, {goal_request.z_r:.3f})"
        )
        
        # 检查控制权限
        has_authority, error_msg = self._has_control_authority()
        if not has_authority:
            self.get_logger().error(f"[Action目标] 拒绝请求: {error_msg}")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def _action_cancel_callback(self, goal_handle):
        """取消Action目标时的回调"""
        self.get_logger().info("[Action取消] 收到取消请求")
        return CancelResponse.ACCEPT
    
    def _execute_action_callback(self, goal_handle):
        """执行Action的主回调函数"""
        self.get_logger().info("[Action执行] 开始执行任务")
        
        # 注册当前goal_handle（用于急停时中止）
        with self._goal_handle_lock:
            self._current_goal_handle = goal_handle
        
        try:
            # 获取目标参数
            goal = goal_handle.request
            
            # 再次检查权限（防止在等待期间权限被撤销）
            has_authority, error_msg = self._has_control_authority()
            if not has_authority:
                self.get_logger().error(f"[Action执行] 权限验证失败: {error_msg}")
                result = ArmControl.Result()
                result.result = "failed"
                result.error = error_msg
                goal_handle.abort()
                return result
            
            # 验证目标参数
            error_msg = self._validate_goal(goal)
            if error_msg:
                self.get_logger().error(f"[Action执行] 参数验证失败: {error_msg}")
                result = ArmControl.Result()
                result.result = "failed"
                result.error = error_msg
                goal_handle.abort()
                return result
            
            # 根据high_resolution参数动态设置距离阈值
            base_threshold = DISTANCE_THRESHOLD
            multiplier = max(1, goal.high_resolution)
            dynamic_threshold = base_threshold * multiplier
            
            self.controller.set_distance_threshold(dynamic_threshold)
            self.get_logger().info(
                f"[Action执行] 精度模式={goal.high_resolution}, "
                f"容差阈值={dynamic_threshold:.4f}m ({multiplier}x基准)"
            )
            
            # 设置目标位置
            left_xyz = np.array([goal.x_l, goal.y_l, goal.z_l], dtype=np.float32)
            right_xyz = np.array([goal.x_r, goal.y_r, goal.z_r], dtype=np.float32)
            self.controller.set_goal(left_xyz, right_xyz)
            
            # 反馈频率（Hz）
            feedback_rate = 10.0
            feedback_period = 1.0 / feedback_rate
            
            # 超时设置（秒）
            timeout = 10.0
            start_time = time.time()
            
            # 执行循环
            feedback_msg = ArmControl.Feedback()
            
            while rclpy.ok():
                # 检查急停状态
                with self._estop_lock:
                    if self._emergency_stop:
                        # 急停触发，中止任务
                        self.controller.stop_at_current_position()
                        self.get_logger().error("[Action执行] 🚨 急停触发，任务中止")
                        
                        result = ArmControl.Result()
                        result.result = "failed"
                        result.error = "emergency stop triggered"
                        left_pos, right_pos = self.controller.get_current_position()
                        result.x_l = float(left_pos[0])
                        result.y_l = float(left_pos[1])
                        result.z_l = float(left_pos[2])
                        result.x_r = float(right_pos[0])
                        result.y_r = float(right_pos[1])
                        result.z_r = float(right_pos[2])
                        goal_handle.abort()
                        return result
                
                # 检查MCB权限状态
                with self._authority_lock:
                    if not self._mcb_control_authority:
                        # MCB权限被撤销，中止任务
                        self.controller.stop_at_current_position()
                        self.get_logger().warn("[Action执行] ⚠️  MCB权限已撤销，任务中止")
                        
                        result = ArmControl.Result()
                        result.result = "failed"
                        result.error = "MCB authority revoked"
                        left_pos, right_pos = self.controller.get_current_position()
                        result.x_l = float(left_pos[0])
                        result.y_l = float(left_pos[1])
                        result.z_l = float(left_pos[2])
                        result.x_r = float(right_pos[0])
                        result.y_r = float(right_pos[1])
                        result.z_r = float(right_pos[2])
                        goal_handle.abort()
                        return result
                
                # 检查是否被取消
                if goal_handle.is_cancel_requested:
                    # 停止机械臂
                    self.controller.stop_at_current_position()
                    
                    goal_handle.canceled()
                    self.get_logger().info("[Action执行] 任务已取消，机械臂已停止")
                    
                    result = ArmControl.Result()
                    result.result = "failed"
                    result.error = "cancelled by user"
                    left_pos, right_pos = self.controller.get_current_position()
                    result.x_l = float(left_pos[0])
                    result.y_l = float(left_pos[1])
                    result.z_l = float(left_pos[2])
                    result.x_r = float(right_pos[0])
                    result.y_r = float(right_pos[1])
                    result.z_r = float(right_pos[2])
                    return result
                
                # 获取当前位置
                left_pos, right_pos = self.controller.get_current_position()
                
                # 发送反馈（转换为Python原生float类型）
                feedback_msg.x_l = float(left_pos[0])
                feedback_msg.y_l = float(left_pos[1])
                feedback_msg.z_l = float(left_pos[2])
                feedback_msg.x_r = float(right_pos[0])
                feedback_msg.y_r = float(right_pos[1])
                feedback_msg.z_r = float(right_pos[2])
                goal_handle.publish_feedback(feedback_msg)
                
                # 检查是否到达
                left_reached, right_reached, both_reached = self.controller.check_reached()
                
                if both_reached:
                    self.get_logger().info("[Action执行] ✓ 双臂均已到达目标")
                    goal_handle.succeed()
                    
                    result = ArmControl.Result()
                    result.result = "success"
                    result.error = ""
                    result.x_l = float(left_pos[0])
                    result.y_l = float(left_pos[1])
                    result.z_l = float(left_pos[2])
                    result.x_r = float(right_pos[0])
                    result.y_r = float(right_pos[1])
                    result.z_r = float(right_pos[2])
                    return result
                
                # 检查超时
                elapsed = time.time() - start_time
                if elapsed > timeout:
                    # 停止机械臂
                    self.controller.stop_at_current_position()
                    
                    self.get_logger().warn(
                        f"[Action执行] ⏱️  超时 ({timeout}秒) - "
                        f"左臂{'已到达' if left_reached else '未到达'}, "
                        f"右臂{'已到达' if right_reached else '未到达'}, 机械臂已停止"
                    )
                    goal_handle.abort()
                    
                    result = ArmControl.Result()
                    result.result = "failed"
                    result.error = f"timeout after {timeout}s"
                    result.x_l = float(left_pos[0])
                    result.y_l = float(left_pos[1])
                    result.z_l = float(left_pos[2])
                    result.x_r = float(right_pos[0])
                    result.y_r = float(right_pos[1])
                    result.z_r = float(right_pos[2])
                    return result
                
                # 休眠
                time.sleep(feedback_period)
            
            # 节点关闭 - 停止机械臂
            self.controller.stop_at_current_position()
            self.get_logger().info("[Action执行] 节点关闭，机械臂已停止")
            result = ArmControl.Result()
            result.result = "failed"
            result.error = "node shutdown"
            goal_handle.abort()
            return result
        
        finally:
            # 清除goal_handle注册
            with self._goal_handle_lock:
                self._current_goal_handle = None
    
    def _validate_goal(self, goal):
        """验证目标参数
        
        Args:
            goal: Action目标
            
        Returns:
            error_msg: 错误信息（空字符串表示验证通过）
        """
        # 检查target类型
        valid_targets = ["empty_box", "product_box", ""]
        if goal.target not in valid_targets:
            return f"invalid target: {goal.target}, must be one of {valid_targets}"
        
        # 检查action类型
        valid_actions = ["pick", "place", "move", ""]
        if goal.action not in valid_actions:
            return f"invalid action: '{goal.action}', must be one of {valid_actions}"
        
        # 检查精度模式参数
        if not isinstance(goal.high_resolution, int) or goal.high_resolution < 1:
            return f"invalid high_resolution: {goal.high_resolution}, must be integer >= 1"
        
        # 检查位置范围（基于G1工作空间）
        if not (0.0 <= goal.x_l <= 0.6):
            return f"left x out of range: {goal.x_l}, must be in [0.0, 0.6]"
        if not (0.0 <= goal.y_l <= 0.6):
            return f"left y out of range: {goal.y_l}, must be in [0.0, 0.6]"
        if not (0.55 <= goal.z_l <= 1.45):
            return f"left z out of range: {goal.z_l}, must be in [0.55, 1.45]"
        
        if not (0.0 <= goal.x_r <= 0.6):
            return f"right x out of range: {goal.x_r}, must be in [0.0, 0.6]"
        if not (-0.6 <= goal.y_r <= 0.0):
            return f"right y out of range: {goal.y_r}, must be in [-0.6, 0.0]"
        if not (0.55 <= goal.z_r <= 1.45):
            return f"right z out of range: {goal.z_r}, must be in [0.55, 1.45]"
        
        return ""  # 验证通过


# ---------------------------------------------------------------------------
# 主函数
# ---------------------------------------------------------------------------
def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    server = None
    executor = None
    
    try:
        # 创建统一服务器节点
        server = UnifiedArmControlServer()
        
        # 使用多线程执行器
        executor = MultiThreadedExecutor()
        executor.add_node(server)
        
        print("\n" + "=" * 70)
        print("统一机械臂控制服务器正在运行...")
        print("  - 权限管理服务: /srv_control_power_transport")
        print("  - Action服务: action_arm")
        print("  - R键控制由policy_controller本地管理")
        print("按 Ctrl+C 停止")
        print("=" * 70 + "\n")
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            server.get_logger().info("收到中断信号，正在关闭...")
    
    finally:
        # 优雅关闭
        if server is not None:
            try:
                # 1. 先停止机械臂
                server.controller.stop_at_current_position()
                server.get_logger().info("机械臂已停止")
            except Exception:
                pass
        
        if executor is not None:
            try:
                # 2. 移除节点并关闭executor
                executor.remove_node(server)
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        
        if server is not None:
            try:
                # 3. 销毁Action Server
                if hasattr(server, '_action_server'):
                    server._action_server.destroy()
            except Exception:
                pass
            
            try:
                # 4. 销毁节点
                server.destroy_node()
            except Exception:
                pass
        
        # 5. 最后关闭rclpy
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        
        print("[INFO] 统一服务器已完全关闭")


if __name__ == '__main__':
    main()
