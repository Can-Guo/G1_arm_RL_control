#!/usr/bin/env python3
"""test_arm_client_simple.py - G1双臂控制专业测试客户端

功能：
  • 使用 actionpoints.yaml 中的预设动作点
  • 支持用户自定义输入坐标
  • 超时停止机制测试（服务器端30秒超时）
  • 批量测试模式
  • 状态查询功能
  • 参数调整（动作类型、目标类型）
  • 测试报告生成
  • 可配置 Action 名称（默认 /arm_control）

命令列表：
  [基本控制]
  1-4 - 移动到预设点
  P   - 自定义位置
  
  [测试功能]
  T   - 超时停止测试
  B   - 批量测试所有预设点
  S   - 查询当前状态
  
  [参数设置]
  C   - 配置参数（动作/目标类型）
  
  [其他]
  H   - 帮助信息
  R   - 显示测试报告
  Q   - 退出

环境变量：
  ARM_CONTROL_ACTION_NAME - Action名称（默认: /arm_control）

作者: York
版本: 2.0 - 专业测试版
日期: 2025-11-14
"""

from __future__ import annotations

import sys
import time
from datetime import datetime
from typing import Optional, Tuple, List, Dict

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import os

# 导入编译后的Action接口
try:
    from robot_interfaces.action import ArmControl
    ACTION_AVAILABLE = True
except ImportError as e:
    print(f"[错误] 无法导入Action接口: {e}")
    print("[提示] 请先编译接口包并加载环境")
    print("       cd ~/RL_arm_control")
    print("       ./src/arm_controller/build_action_interface.sh")
    print("       source ~/RL_arm_control/install/setup.bash")
    sys.exit(1)

# 导入动作点配置
try:
    from arm_controller.action_points_config import get_config, list_action_points
    CONFIG_AVAILABLE = True
except ImportError:
    try:
        # 兜底：开发模式直接导入
        from action_points_config import get_config, list_action_points
        CONFIG_AVAILABLE = True
    except ImportError:
        import sys
        print("[警告] 无法导入 action_points_config", file=sys.stderr)
        print("[提示] 请确保配置文件存在并重新构建包", file=sys.stderr)
        CONFIG_AVAILABLE = False


# ============================================================================
# 测试结果记录
# ============================================================================

class TestRecord:
    """单次测试记录"""
    def __init__(self, test_type: str, description: str):
        self.test_type = test_type
        self.description = description
        self.start_time = datetime.now()
        self.end_time = None
        self.duration = 0.0
        self.success = False
        self.error_msg = ""
        self.target_pos = None
        self.final_pos = None
    
    def finish(self, success: bool, error_msg: str = ""):
        """完成测试"""
        self.end_time = datetime.now()
        self.duration = (self.end_time - self.start_time).total_seconds()
        self.success = success
        self.error_msg = error_msg
    
    def __str__(self):
        status = "✅ 成功" if self.success else "❌ 失败"
        return (f"{self.test_type} | {self.description} | "
                f"{status} | {self.duration:.2f}s | {self.error_msg}")


class TestReport:
    """测试报告管理器"""
    def __init__(self):
        self.records: List[TestRecord] = []
        self.session_start = datetime.now()
    
    def add_record(self, record: TestRecord):
        """添加测试记录"""
        self.records.append(record)
    
    def get_summary(self) -> str:
        """生成摘要报告"""
        if not self.records:
            return "暂无测试记录"
        
        total = len(self.records)
        success = sum(1 for r in self.records if r.success)
        failed = total - success
        avg_duration = sum(r.duration for r in self.records) / total
        
        lines = [
            "=" * 70,
            "测试报告摘要",
            "=" * 70,
            f"会话开始时间: {self.session_start.strftime('%Y-%m-%d %H:%M:%S')}",
            f"测试总数: {total}",
            f"成功: {success} ({success/total*100:.1f}%)",
            f"失败: {failed} ({failed/total*100:.1f}%)",
            f"平均耗时: {avg_duration:.2f}秒",
            "",
            "详细记录:",
            "-" * 70,
        ]
        
        for i, record in enumerate(self.records, 1):
            lines.append(f"{i}. {record}")
        
        lines.append("=" * 70)
        return "\n".join(lines)
    
    def save_to_file(self, filename: str = None):
        """保存报告到文件"""
        if filename is None:
            filename = f"test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(self.get_summary())
        
        return filename


class ArmClient(Node):
    """G1双臂控制Action客户端"""
    
    def __init__(self):
        super().__init__('arm_control_client')
        
        # Action name（可通过环境变量 ARM_CONTROL_ACTION_NAME 覆盖）
        # 默认与快速启动脚本一致：/arm_control
        self.action_name = os.environ.get('ARM_CONTROL_ACTION_NAME', '/action_arm')
        
        # 创建Action Client
        self._action_client = ActionClient(self, ArmControl, self.action_name)
        
        # 加载动作点配置
        if CONFIG_AVAILABLE:
            self.config = get_config()
            available_points = list_action_points()
            self.get_logger().info(f"已加载 {len(available_points)} 个预设动作点")
        else:
            self.config = None
            self.get_logger().warn("未加载配置")
        
        # 参数设置
        self.default_action = "move"  # 动作类型: move/pick/place
        self.default_target = "empty_box"  # 目标类型: empty_box/product_box
        self.default_resolution = 2  # 默认精度模式: 1(高)/2(中)/3(低)
        
        # 测试报告
        self.report = TestReport()
        
        self.get_logger().info("客户端已创建")
    
    def send_goal_by_name(self, point_name: str, record: TestRecord = None):
        """使用预设动作点名称发送目标"""
        if not CONFIG_AVAILABLE or self.config is None:
            self.get_logger().error("配置未加载")
            if record:
                record.finish(False, "config_not_loaded")
            return False
        
        # 获取完整配置(包括high_resolution)
        config = self.config.get_full_config(point_name)
        if config is None:
            self.get_logger().error(f"动作点 '{point_name}' 不存在")
            if record:
                record.finish(False, "point_not_found")
            return False
        
        left_xyz = config['left']
        right_xyz = config['right']
        high_resolution = config.get('high_resolution')  # 可能为None
        
        # 如果配置中指定了精度,显示提示
        if high_resolution is not None:
            resolution_desc = {1: '高精度(3cm)', 2: '中精度(6cm)', 3: '低精度(9cm)'}.get(
                high_resolution, f'自定义({high_resolution}x)'
            )
            self.get_logger().info(f"动作点 '{point_name}' 配置了精度: {resolution_desc}")
        
        return self.send_goal(left_xyz, right_xyz, high_resolution=high_resolution, record=record)
    
    def send_goal(self, left_xyz: tuple, right_xyz: tuple, 
                  action: str = None, target: str = None,
                  high_resolution: int = None,
                  record: TestRecord = None):
        """发送目标指令
        
        Args:
            left_xyz: 左臂目标位置
            right_xyz: 右臂目标位置
            action: 动作类型 (move/pick/place)
            target: 目标类型 (empty_box/product_box)
            high_resolution: 精度模式 (1=高/2=中/3=低)
            record: 测试记录对象
        
        Returns:
            bool: 是否成功
        """
        # 使用默认参数
        if action is None:
            action = self.default_action
        if target is None:
            target = self.default_target
        if high_resolution is None:
            high_resolution = self.default_resolution
        
        # 等待服务器
        self.get_logger().info(f"等待Action Server ({self.action_name})...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"❌ Action Server {self.action_name} 连接超时！")
            if record:
                record.finish(False, "server_timeout")
            return False
        
        # 构造目标
        goal_msg = ArmControl.Goal()
        goal_msg.target = target
        goal_msg.action = action
        goal_msg.x_l, goal_msg.y_l, goal_msg.z_l = left_xyz
        goal_msg.x_r, goal_msg.y_r, goal_msg.z_r = right_xyz
        goal_msg.high_resolution = high_resolution
        
        # 精度模式描述
        resolution_desc = {1: '高精度(3cm)', 2: '中精度(6cm)', 3: '低精度(9cm)'}.get(
            high_resolution, f'自定义({high_resolution}x)'
        )
        
        print("=" * 60)
        print(f"📤 发送目标:")
        print(f"   目标类型: {target}")
        print(f"   动作类型: {action}")
        print(f"   精度模式: {resolution_desc}")
        print(f"   左臂目标: ({left_xyz[0]:.3f}, {left_xyz[1]:.3f}, {left_xyz[2]:.3f})")
        print(f"   右臂目标: ({right_xyz[0]:.3f}, {right_xyz[1]:.3f}, {right_xyz[2]:.3f})")
        print("=" * 60)
        
        if record:
            record.target_pos = (left_xyz, right_xyz)
        
        # 发送目标
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # 等待接受
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ 目标被服务器拒绝！")
            if record:
                record.finish(False, "goal_rejected")
            return False
        
        self.get_logger().info("✅ 目标已接受，执行中...")
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        success = self._display_result(result)
        
        if record:
            record.final_pos = (
                (result.x_l, result.y_l, result.z_l),
                (result.x_r, result.y_r, result.z_r)
            )
            record.finish(success, result.error if not success else "")
        
        return success
    
    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[反馈] 左:({fb.x_l:.2f},{fb.y_l:.2f},{fb.z_l:.2f}) "
            f"右:({fb.x_r:.2f},{fb.y_r:.2f},{fb.z_r:.2f})",
            throttle_duration_sec=1.0
        )
    
    def _display_result(self, result) -> bool:
        """显示结果
        
        Returns:
            bool: 是否成功
        """
        print("\n" + "=" * 60)
        if result.result == "success":
            print("✅ 任务执行成功！")
            success = True
        else:
            print(f"❌ 任务执行失败")
            print(f"   错误信息: {result.error}")
            success = False
        
        print(f"📍 最终位置:")
        print(f"   左臂: ({result.x_l:.3f}, {result.y_l:.3f}, {result.z_l:.3f})")
        print(f"   右臂: ({result.x_r:.3f}, {result.y_r:.3f}, {result.z_r:.3f})")
        print("=" * 60)
        
        return success
    
    def test_timeout_stop(self):
        """测试超时停止机制"""
        print("\n" + "=" * 70)
        print(" " * 20 + "超时停止机制测试")
        print("=" * 70)
        print("\n测试说明:")
        print("  • 发送一个较远的目标位置")
        print("  • 服务器端有30秒超时限制")
        print("  • 观察超时后机械臂是否立即停止")
        print("\n预期结果:")
        print("  ✅ 服务器超时后发送停止命令")
        print("  ✅ 机械臂立即停止在当前位置")
        print("  ✅ 机械臂不再继续移动")
        print("=" * 70)
        
        confirm = input("\n开始测试? (Y/n): ").strip().lower()
        if confirm and confirm not in ['y', 'yes', '']:
            print("测试已取消")
            return
        
        # 创建测试记录
        record = TestRecord("timeout_test", "超时停止机制测试")
        
        # 发送远距离目标
        far_left = (0.5, 0.6, 1.0)
        far_right = (0.5, -0.6, 1.0)
        
        print("\n🚀 发送测试目标...")
        print("⚠️  注意：服务器端有30秒超时机制")
        success = self.send_goal(far_left, far_right, record=record)
        
        if not success:
            print("\n" + "=" * 70)
            print("⚠️  测试完成 - 请观察机械臂行为：")
            print("   1. 机械臂应该在超时位置停止")
            print("   2. 机械臂不应该继续移动向原目标")
            print("\n⏱️  等待5秒观察...")
            print("=" * 70)
            time.sleep(5)
            
            # 添加到报告
            self.report.add_record(record)
            
            print("\n✅ 如果机械臂已停止 → 停止机制正常")
            print("❌ 如果机械臂仍在移动 → 停止机制失败")
        else:
            print("\n⚠️  目标在超时前完成，无法测试超时停止")
            self.report.add_record(record)
    
    def batch_test_presets(self):
        """批量测试所有预设点"""
        if not CONFIG_AVAILABLE or self.config is None:
            print("❌ 配置未加载，无法执行批量测试")
            return
        
        preset_points = {
            'arm_safety': '安全点',
            'arm_home': '初始位置',
            'arm_open_prepare': '开箱准备',
            'arm_open_prepare_for_box': '箱体开箱准备',
        }
        
        print("\n" + "=" * 70)
        print(" " * 25 + "批量测试模式")
        print("=" * 70)
        print(f"\n将依次测试 {len(preset_points)} 个预设点:")
        for i, (key, desc) in enumerate(preset_points.items(), 1):
            print(f"  {i}. {desc} ({key})")
        print("=" * 70)
        
        confirm = input("\n开始批量测试? (Y/n): ").strip().lower()
        if confirm and confirm not in ['y', 'yes', '']:
            print("测试已取消")
            return
        
        results = []
        for i, (point_name, desc) in enumerate(preset_points.items(), 1):
            print(f"\n{'─'*70}")
            print(f"测试 {i}/{len(preset_points)}: {desc}")
            print(f"{'─'*70}")
            
            record = TestRecord("preset_test", f"{desc} ({point_name})")
            success = self.send_goal_by_name(point_name, record=record)
            results.append((desc, success))
            self.report.add_record(record)
            
            if i < len(preset_points):
                print("\n⏱️  等待3秒后继续...")
                time.sleep(3)
        
        # 显示总结
        print("\n" + "=" * 70)
        print(" " * 25 + "批量测试完成")
        print("=" * 70)
        success_count = sum(1 for _, s in results if s)
        print(f"\n总测试数: {len(results)}")
        print(f"成功: {success_count}")
        print(f"失败: {len(results) - success_count}")
        print("\n详细结果:")
        for desc, success in results:
            status = "✅" if success else "❌"
            print(f"  {status} {desc}")
        print("=" * 70)


def parse_custom_position(input_str: str):
    """解析用户输入的自定义位置
    
    格式: "P x_l y_l z_l x_r y_r z_r [resolution]"
    返回: ((x_l, y_l, z_l), (x_r, y_r, z_r), resolution) 或 None
    """
    try:
        # 移除 'P' 前缀
        input_str = input_str.strip()
        if input_str.upper().startswith('P'):
            input_str = input_str[1:].strip()
        
        # 分割并转换
        parts = input_str.split()
        if len(parts) not in [6, 7]:
            print(f"[错误] 需要6个坐标值(+可选精度)，您输入了 {len(parts)} 个")
            return None
        
        coords = [float(x) for x in parts[:6]]
        left_xyz = (coords[0], coords[1], coords[2])
        right_xyz = (coords[3], coords[4], coords[5])
        
        # 解析可选的精度参数
        resolution = None
        if len(parts) == 7:
            resolution = int(parts[6])
            if resolution < 1:
                print(f"[错误] 精度参数必须 >= 1，您输入了 {resolution}")
                return None
        
        return left_xyz, right_xyz, resolution
    
    except ValueError as e:
        print(f"[错误] 参数解析失败: {e}")
        return None


def configure_parameters(client: ArmClient):
    """配置参数界面"""
    print("\n" + "=" * 70)
    print(" " * 25 + "参数配置")
    print("=" * 70)
    
    # 精度描述
    resolution_desc = {1: '高精度(3cm)', 2: '中精度(6cm)', 3: '低精度(9cm)'}.get(
        client.default_resolution, f'{client.default_resolution}x'
    )
    
    print(f"\n当前设置:")
    print(f"  默认动作类型: {client.default_action}")
    print(f"  默认目标类型: {client.default_target}")
    print(f"  默认精度模式: {client.default_resolution} ({resolution_desc})")
    print("\n可配置项:")
    print("  1. 修改动作类型 (move/pick/place)")
    print("  2. 修改目标类型 (empty_box/product_box)")
    print("  3. 修改精度模式 (1=高/2=中/3=低)")
    print("  0. 返回主菜单")
    print("=" * 70)
    
    choice = input("\n请选择 (0-3): ").strip()
    
    if choice == '1':
        print("\n动作类型:")
        print("  1. move  - 移动")
        print("  2. pick  - 抓取")
        print("  3. place - 放置")
        action_choice = input("选择 (1-3): ").strip()
        action_map = {'1': 'move', '2': 'pick', '3': 'place'}
        if action_choice in action_map:
            client.default_action = action_map[action_choice]
            print(f"✅ 动作类型已设置为 {client.default_action}")
        else:
            print("❌ 无效选择")
    
    elif choice == '2':
        print("\n目标类型:")
        print("  1. empty_box    - 空箱")
        print("  2. product_box  - 产品箱")
        target_choice = input("选择 (1-2): ").strip()
        target_map = {'1': 'empty_box', '2': 'product_box'}
        if target_choice in target_map:
            client.default_target = target_map[target_choice]
            print(f"✅ 目标类型已设置为 {client.default_target}")
        else:
            print("❌ 无效选择")
    
    elif choice == '3':
        print("\n精度模式:")
        print("  1. 高精度 (容差 3cm)  - 适合精密操作")
        print("  2. 中精度 (容差 6cm)  - 适合常规任务")
        print("  3. 低精度 (容差 9cm)  - 适合快速移动")
        resolution_choice = input("选择 (1-3): ").strip()
        if resolution_choice in ['1', '2', '3']:
            client.default_resolution = int(resolution_choice)
            desc = {1: '高精度(3cm)', 2: '中精度(6cm)', 3: '低精度(9cm)'}[client.default_resolution]
            print(f"✅ 精度模式已设置为 {client.default_resolution} ({desc})")
        else:
            print("❌ 无效选择")
    
    elif choice == '0':
        return
    else:
        print("❌ 无效选择")


def query_status(client: ArmClient):
    """查询当前状态（模拟功能）"""
    print("\n" + "=" * 70)
    print(" " * 25 + "系统状态查询")
    print("=" * 70)
    print("\n⚠️  注意：当前版本仅显示配置信息")
    print("   （实时位置需要订阅 /g1/left_arm/position 等话题）")
    
    # 精度描述
    resolution_desc = {1: '高精度(3cm)', 2: '中精度(6cm)', 3: '低精度(9cm)'}.get(
        client.default_resolution, f'{client.default_resolution}x'
    )
    
    print("\n当前配置:")
    print(f"  默认动作类型: {client.default_action}")
    print(f"  默认目标类型: {client.default_target}")
    print(f"  默认精度模式: {client.default_resolution} ({resolution_desc})")
    print(f"  Action Server: {client.action_name}")
    
    # 检查服务器状态
    print("\n检查服务器连接...")
    if client._action_client.wait_for_server(timeout_sec=3.0):
        print("  ✅ Action Server 已连接")
    else:
        print("  ❌ Action Server 未响应")
    
    # 显示测试统计
    if client.report.records:
        total = len(client.report.records)
        success = sum(1 for r in client.report.records if r.success)
        print(f"\n本次会话测试统计:")
        print(f"  总测试次数: {total}")
        print(f"  成功: {success}")
        print(f"  失败: {total - success}")
    
    print("=" * 70)


def print_menu():
    """打印命令菜单"""
    print("\n" + "=" * 70)
    print(" " * 20 + "G1双臂控制 - 命令菜单")
    print("=" * 70)
    print("\n【基本控制】")
    print("  1 - 移动到安全点 (arm_safety)")
    print("  2 - 移动到初始位置 (arm_home)")
    print("  3 - 移动到开箱准备 (arm_open_prepare)")
    print("  4 - 移动到箱体开箱准备 (arm_open_prepare_for_box)")
    print("  P - 自定义位置")
    print("      格式: P <x_l> <y_l> <z_l> <x_r> <y_r> <z_r> [精度]")
    print("      示例: P 0.3 0.2 0.8 0.3 -0.2 0.8     (默认精度)")
    print("            P 0.3 0.2 0.8 0.3 -0.2 0.8 1   (高精度)")
    
    print("\n【测试功能】")
    print("  T - 超时停止测试（测试超时后机械臂是否停止）")
    print("  B - 批量测试（依次测试所有预设点）")
    print("  S - 状态查询（查看系统状态和统计信息）")
    
    print("\n【参数设置】")
    print("  C - 配置参数（修改动作类型、目标类型）")
    
    print("\n【其他】")
    print("  H - 帮助信息（查看详细使用说明）")
    print("  R - 显示测试报告")
    print("  M - 显示此菜单")
    print("  Q - 退出程序")
    print("=" * 70)


def print_help():
    """打印帮助信息"""
    print("\n" + "=" * 70)
    print(" " * 25 + "详细帮助信息")
    print("=" * 70)
    
    print("\n【坐标系说明】")
    print("  X轴: 前后方向（机器人正前方为正）")
    print("  Y轴: 左右方向（左臂为正方向，右臂为负方向）")
    print("  Z轴: 垂直方向（向上为正）")
    print("  单位: 米 (m)")
    
    print("\n【工作空间范围】")
    print("  左臂: X [0.0 - 0.6]  Y [0.0 - 0.6]  Z [0.55 - 1.45]")
    print("  右臂: X [0.0 - 0.6]  Y [-0.6 - 0.0]  Z [0.55 - 1.45]")
    
    print("\n【预设动作点】")
    print("  1. arm_safety - 安全点（碰撞风险最低）")
    print("  2. arm_home - 初始位置（标准待机姿态）")
    print("  3. arm_open_prepare - 开箱准备（较高位置）")
    print("  4. arm_open_prepare_for_box - 箱体开箱准备（特定任务）")
    
    print("\n【基本操作】")
    print("  • 输入 1-4 直接移动到预设点")
    print("  • 输入 P 加6个坐标值(+可选精度)自定义位置")
    print("    格式: P x_left y_left z_left x_right y_right z_right [精度]")
    print("    示例: P 0.3 0.2 0.8 0.3 -0.2 0.8      (使用默认精度)")
    print("          P 0.3 0.2 0.8 0.3 -0.2 0.8 1    (高精度)")
    print("          P 0.3 0.2 0.8 0.3 -0.2 0.8 3    (低精度)")
    
    print("\n【测试功能】")
    print("  T - 超时停止测试")
    print("      测试Action超时后机械臂是否正确停止")
    print("      用于验证安全机制是否工作")
    
    print("  B - 批量测试")
    print("      依次测试所有预设点，自动生成测试报告")
    print("      适合验证系统稳定性")
    
    print("  S - 状态查询")
    print("      查看当前配置、服务器连接状态、测试统计")
    
    print("\n【参数配置】")
    print("  C - 配置参数")
    print("      • 修改默认动作类型（move/pick/place）")
    print("      • 修改默认目标类型（empty_box/product_box）")
    print("      • 服务器端超时：30秒（在服务器端配置）")
    
    print("\n【测试报告】")
    print("  R - 显示测试报告")
    print("      查看本次会话所有测试记录")
    print("      包含成功率、平均耗时等统计信息")
    
    print("\n【安全提示】")
    print("  ⚠️  首次运行建议先测试安全点（命令1）")
    print("  ⚠️  自定义位置请确保在工作空间范围内")
    print("  ⚠️  观察机械臂运动，如有异常立即停止")
    
    print("=" * 70)


def main():
    """主函数"""
    print("\n" + "=" * 70)
    print(" " * 15 + "G1双臂控制专业测试客户端 v2.0")
    print("=" * 70)
    print(f"\n会话开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建客户端
    client = ArmClient()
    
    # 预设点映射
    preset_points = {
        '1': ('arm_safety', '安全点'),
        '2': ('arm_home', '初始位置'),
        '3': ('arm_open_prepare', '开箱准备'),
        '4': ('arm_open_prepare_for_box', '箱体开箱准备'),
    }
    
    # 显示菜单
    print_menu()
    
    try:
        while True:
            try:
                # 读取输入
                user_input = input("\n💬 请输入命令: ").strip()
                
                if not user_input:
                    continue
                
                cmd = user_input.upper()
                
                # ============ 退出 ============
                if cmd in ['Q', 'QUIT', 'EXIT']:
                    # 显示会话报告
                    if client.report.records:
                        print("\n" + client.report.get_summary())
                        save = input("\n保存测试报告? (Y/n): ").strip().lower()
                        if not save or save in ['y', 'yes']:
                            filename = client.report.save_to_file()
                            print(f"✅ 报告已保存至: {filename}")
                    print("\n退出程序...")
                    break
                
                # ============ 帮助 ============
                elif cmd in ['H', 'HELP', '?']:
                    print_help()
                    continue
                
                # ============ 菜单 ============
                elif cmd in ['M', 'MENU']:
                    print_menu()
                    continue
                
                # ============ 预设点 (1-4) ============
                elif user_input in preset_points:
                    point_name, desc = preset_points[user_input]
                    print(f"\n▶ 执行: 移动到{desc}")
                    
                    record = TestRecord("preset_move", f"{desc} ({point_name})")
                    success = client.send_goal_by_name(point_name, record=record)
                    client.report.add_record(record)
                    print("✅ 完成" if success else "❌ 失败")
                
                # ============ 自定义位置 (P) ============
                elif cmd.startswith('P'):
                    print("\n▶ 执行: 移动到自定义位置")
                    
                    positions = parse_custom_position(user_input)
                    if positions is None:
                        print("❌ 格式错误")
                        print("正确格式: P <x_l> <y_l> <z_l> <x_r> <y_r> <z_r> [精度]")
                        print("示例: P 0.3 0.2 0.8 0.3 -0.2 0.8      (使用默认精度)")
                        print("      P 0.3 0.2 0.8 0.3 -0.2 0.8 1    (使用高精度)")
                        continue
                    
                    left_xyz, right_xyz, resolution = positions
                    
                    # 显示目标信息
                    print(f"   左臂目标: ({left_xyz[0]:.3f}, {left_xyz[1]:.3f}, {left_xyz[2]:.3f})")
                    print(f"   右臂目标: ({right_xyz[0]:.3f}, {right_xyz[1]:.3f}, {right_xyz[2]:.3f})")
                    
                    # 显示精度信息
                    if resolution is not None:
                        res_desc = {1: '高精度(3cm)', 2: '中精度(6cm)', 3: '低精度(9cm)'}.get(
                            resolution, f'自定义({resolution}x)'
                        )
                        print(f"   精度模式: {resolution} ({res_desc})")
                    else:
                        print(f"   精度模式: 使用默认值 ({client.default_resolution})")
                    
                    # 确认
                    confirm = input("\n确认执行? (Y/n): ").strip().lower()
                    if confirm and confirm not in ['y', 'yes', '']:
                        print("❌ 已取消")
                        continue
                    
                    record = TestRecord("custom_move", f"自定义位置")
                    success = client.send_goal(left_xyz, right_xyz, high_resolution=resolution, record=record)
                    client.report.add_record(record)
                    print("✅ 完成" if success else "❌ 失败")
                
                # ============ 超时测试 (T) ============
                elif cmd in ['T', 'TEST']:
                    client.test_timeout_stop()
                
                # ============ 批量测试 (B) ============
                elif cmd in ['B', 'BATCH']:
                    client.batch_test_presets()
                
                # ============ 状态查询 (S) ============
                elif cmd in ['S', 'STATUS']:
                    query_status(client)
                
                # ============ 参数配置 (C) ============
                elif cmd in ['C', 'CONFIG']:
                    configure_parameters(client)
                
                # ============ 显示报告 (R) ============
                elif cmd in ['R', 'REPORT']:
                    if client.report.records:
                        print("\n" + client.report.get_summary())
                    else:
                        print("\n暂无测试记录")
                
                # ============ 无效命令 ============
                else:
                    print(f"❌ 无效命令: {user_input}")
                    print("💡 输入 'H' 查看帮助，'M' 查看菜单")
            
            except EOFError:
                print("\n检测到EOF，退出...")
                break
            
            except KeyboardInterrupt:
                print("\n检测到中断信号...")
                confirm = input("确认退出? (Y/n): ").strip().lower()
                if not confirm or confirm in ['y', 'yes']:
                    break
    
    except KeyboardInterrupt:
        print("\n正在退出...")
    
    finally:
        # 清理
        print("\n" + "─" * 70)
        print("清理资源...")
        
        if client is not None:
            client.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        print("✅ 资源已清理")
        print("=" * 70)
        print(" " * 20 + "感谢使用 G1双臂控制测试客户端")
        print("=" * 70)


if __name__ == '__main__':
    main()
