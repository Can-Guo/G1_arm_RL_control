#!/usr/bin/env python3
"""arm_control_action_server.py - G1双臂控制ROS2 Action服务器（生产版）

功能：
  • 接收MCB的双臂控制指令（通过ROS2 Action）
  • 控制G1机器人左右臂移动到指定位置
  • 实时反馈当前位置，返回执行结果

依赖：
  • 需要先编译 arm_control_interfaces 包
  • 运行前执行: source ~/RL_arm_control/ros2_ws/install/setup.bash

作者: York
日期: 2025-11-13
协议: 参见 Arm_control_protocol.md
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
from std_msgs.msg import Float64MultiArray

# 导入距离阈值
# 优先从 arm_controller 包的 __init__.py 导入，避免触发 gymnasium 依赖
try:
    from arm_controller import DISTANCE_THRESHOLD
except (ImportError, AttributeError):
    # 兜底：使用默认值
    DISTANCE_THRESHOLD = 0.03
    import sys
    print(f"[提示] 使用默认距离阈值: {DISTANCE_THRESHOLD}m", file=sys.stderr)

# 导入编译后的Action接口
try:
    from robot_interfaces.action import ArmControl
    ACTION_AVAILABLE = True
except ImportError as e:
    print(f"[错误] 无法导入Action接口: {e}")
    print("[提示] 请先编译接口包:")
    print("       方法1: 使用便捷脚本")
    print("       cd ~/sunny/hr_ros2_ws/src/arm_controller")
    print("       ./build_action_interface.sh")
    print("")
    print("       方法2: 手动编译")
    print("       cd ~/sunny/hr_ros2_ws")
    print("       colcon build --packages-select robot_interfaces")
    print("       source ~/sunny/hr_ros2_ws/install/setup.bash")
    ACTION_AVAILABLE = False


# ---------------------------------------------------------------------------
# 双臂控制接口（连接到控制器）
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
# ROS2 Action Server
# ---------------------------------------------------------------------------
class ArmControlActionServer(Node):
    """G1双臂控制Action服务器"""
    
    def __init__(self):
        super().__init__('arm_control_action_server')
        
        if not ACTION_AVAILABLE:
            raise RuntimeError("Action接口未编译，请先运行 build_action_interface.sh")
        
        # 初始化双臂控制器
        self.controller = DualArmController(self)
        
        # 创建Action Server
        self._action_server = ActionServer(
            self,
            ArmControl,
            'action_arm',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("G1双臂控制Action服务器已启动")
        self.get_logger().info("Action名称: /action_arm")
        self.get_logger().info("Action类型: arm_control_interfaces/action/ArmControl")
        self.get_logger().info("等待MCB指令...")
        self.get_logger().info("=" * 60)
    
    def goal_callback(self, goal_request):
        """接收新目标时的回调"""
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
            f"[Goal] 收到新目标: {action_desc} {target_desc}\n"
            f"  Target: {goal_request.target or '(无)'}\n"
            f"  Action: {goal_request.action}\n"
            f"  精度模式: {resolution_desc}\n"
            f"  左臂目标: ({goal_request.x_l:.3f}, {goal_request.y_l:.3f}, {goal_request.z_l:.3f})\n"
            f"  右臂目标: ({goal_request.x_r:.3f}, {goal_request.y_r:.3f}, {goal_request.z_r:.3f})"
        )
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """取消目标时的回调"""
        self.get_logger().info("[cancel_callback] 收到取消请求")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """执行Action的主回调函数"""
        self.get_logger().info("[execute_callback] 开始执行任务")
        
        # 获取目标参数
        goal = goal_handle.request
        
        # 验证目标参数
        error_msg = self._validate_goal(goal)
        if error_msg:
            self.get_logger().error(f"[execute_callback] 参数验证失败: {error_msg}")
            result = ArmControl.Result()
            result.result = "failed"
            result.error = error_msg
            goal_handle.abort()
            return result
        
        # 根据high_resolution参数动态设置距离阈值
        # high_resolution = 1: 使用基准阈值 (0.03m)
        # high_resolution = 2: 使用2倍基准阈值 (0.06m)
        # high_resolution = 3: 使用3倍基准阈值 (0.09m)
        base_threshold = DISTANCE_THRESHOLD  # 基准阈值(米)
        multiplier = max(1, goal.high_resolution)  # 确保至少为1
        dynamic_threshold = base_threshold * multiplier
        
        self.controller.set_distance_threshold(dynamic_threshold)
        self.get_logger().info(
            f"[execute_callback] 精度模式={goal.high_resolution}, "
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
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                # 停止机械臂
                self.controller.stop_at_current_position()
                
                goal_handle.canceled()
                self.get_logger().info("[execute_callback] 任务已取消，机械臂已停止")
                
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
                self.get_logger().info("[execute_callback] 双臂均已到达目标")
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
                    f"[execute_callback] 超时 ({timeout}秒) - "
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
        self.get_logger().info("[execute_callback] 节点关闭，机械臂已停止")
        result = ArmControl.Result()
        result.result = "failed"
        result.error = "node shutdown"
        goal_handle.abort()
        return result
    
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
        
        # 检查action类型（根据 Arm_control_protocol.md）
        valid_actions = ["pick", "place", "move", ""]
        if goal.action not in valid_actions:
            return f"invalid action: '{goal.action}', must be one of ['pick', 'place', 'move', ''] " \
                   f"(pick=抓取, place=放置, move=移动)"
        
        # 检查精度模式参数
        if not isinstance(goal.high_resolution, int) or goal.high_resolution < 1:
            return f"invalid high_resolution: {goal.high_resolution}, must be integer >= 1 (1=高精度, 2=中精度, 3=低精度)"
        
        # 检查位置范围（基于G1工作空间）
        # X: 0.0 ~ 0.6m (前方)
        # Y: -0.6 ~ 0.6m (左右)
        # Z: 0.55 ~ 1.45m (高度)
        
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


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    action_server = None
    executor = None
    
    try:
        # 创建Action Server节点
        action_server = ArmControlActionServer()
        
        # 使用多线程执行器
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            action_server.get_logger().info("收到中断信号，正在关闭...")
    
    finally:
        # 优雅关闭：改进的顺序
        if action_server is not None:
            try:
                # 1. 先停止机械臂
                action_server.controller.stop_at_current_position()
                action_server.get_logger().info("机械臂已停止")
            except Exception:
                pass
        
        if executor is not None:
            try:
                # 2. 移除节点并关闭executor
                executor.remove_node(action_server)
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        
        if action_server is not None:
            try:
                # 3. 销毁Action Server（避免"Destroyable"错误）
                if hasattr(action_server, '_action_server'):
                    action_server._action_server.destroy()
            except Exception:
                pass
            
            try:
                # 4. 销毁节点
                action_server.destroy_node()
            except Exception:
                pass
        
        # 5. 最后关闭rclpy
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        
        print("[INFO] 服务器已完全关闭")


if __name__ == '__main__':
    main()
