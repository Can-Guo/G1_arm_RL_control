#!/usr/bin/env python3
"""arm_control_authority_server.py - 机械臂MCB控制权限管理服务器

功能描述：
  专门管理MCB（上位机）的机械臂控制权限，不再管理手动控制（R键）。
  R键控制已移至policy_controller本地管理。
  
  新架构：
    - MCB控制：通过本服务器管理（/srv_control_power_transport）
    - R键控制：在policy_controller本地管理（不经过服务器）
    - 最终控制权限 = R键使能 OR MCB授权（两者权限等级相同）
    - 急停拥有最高优先级，可强制禁用机器人输出

服务接口：
  - /srv_control_power_transport (ControlPower) - MCB控制权限请求

订阅话题：
  - /msg_estop (Estop) - 急停信号

发布话题：
  - /g1/arm/mcb_control_authority (Bool) - MCB控制权限状态
  - /g1/arm/emergency_stop (Bool) - 当前急停状态

运行方式：
  ros2 run arm_controller arm_control_authority_server.py

作者: York
日期: 2026-02-06
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading
import time


class ArmControlAuthorityServer(Node):
    """机械臂MCB控制权限管理服务器（仅管理MCB，不管理R键）"""
    
    def __init__(self):
        super().__init__('arm_control_authority_server')
        
        # ==================== 状态管理 ====================
        self._mcb_control_authority = False  # MCB控制权限（默认无权限，安全第一）
        self._emergency_stop = False          # 急停状态
        self._authority_lock = threading.Lock()
        self._estop_lock = threading.Lock()
        
        # ==================== 导入消息和服务接口 ====================
        try:
            from robot_interfaces.srv import ControlPower
            from robot_interfaces.msg import Estop
            self._ControlPower = ControlPower
            self._Estop = Estop
        except ImportError as e:
            self.get_logger().error(f"无法导入接口: {e}")
            self.get_logger().error("请先编译 robot_interfaces 包:")
            self.get_logger().error("  cd /home/york/RL_arm_control")
            self.get_logger().error("  colcon build --packages-select robot_interfaces")
            raise
        
        # ==================== 创建服务 ====================
        # 服务: MCB控制权限请求（兼容现有协议）
        self._mcb_control_srv = self.create_service(
            ControlPower,
            '/srv_control_power_transport',
            self._handle_mcb_control_request
        )
        
        self.get_logger().info("✓ MCB控制权限管理服务已启动:")
        self.get_logger().info("  - /srv_control_power_transport (MCB专用)")
        self.get_logger().info("  注意：R键控制已移至policy_controller本地管理")
        
        # ==================== 订阅急停话题 ====================
        self._estop_sub = self.create_subscription(
            Estop,
            '/msg_estop',
            self._estop_callback,
            10
        )
        self.get_logger().info("✓ 订阅急停话题: /msg_estop")
        
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
        self.get_logger().info("✓ 状态发布话题:")
        self.get_logger().info("  - /g1/arm/mcb_control_authority (MCB权限状态)")
        self.get_logger().info("  - /g1/arm/emergency_stop (急停状态)")
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("机械臂MCB控制权限管理服务器已就绪！")
        self.get_logger().info("初始状态: MCB权限=False, 急停=False")
        self.get_logger().info("架构：R键控制由policy_controller本地管理")
        self.get_logger().info("=" * 60)
    
    def _handle_mcb_control_request(self, request, response):
        """
        处理MCB控制权限请求（仅处理MCB，不再处理R键）
        
        请求参数：
          - task_id: 任务ID
          - object: 控制对象 (arm/hand)
          - type: 操作类型 (get/release)
        
        返回值：
          - result: success/failed
          - error: 错误信息
        """
        self.get_logger().info(
            f"收到MCB控制权限请求: "
            f"task_id={request.task_id}, object={request.object}, type={request.type}"
        )
        
        try:
            # 1. 验证对象类型
            if request.object.lower() == "hand":
                response.result = "success"
                response.error = "hand control ignored (not supported)"
                self.get_logger().info(f"忽略hand控制请求 (task_id={request.task_id})")
                return response
            
            if request.object.lower() != "arm":
                response.result = "failed"
                response.error = f"invalid object: {request.object}, must be 'arm' or 'hand'"
                self.get_logger().warn(f"拒绝无效对象: {request.object}")
                return response
            
            # 2. 处理get/release请求
            with self._authority_lock:
                if request.type.lower() == "get":
                    # MCB请求获取控制权限
                    self._mcb_control_authority = True
                    response.result = "success"
                    response.error = ""
                    self.get_logger().info(
                        f"✓ 授予MCB控制权限 (task_id={request.task_id})"
                    )
                    
                elif request.type.lower() == "release":
                    # MCB请求释放控制权限
                    self._mcb_control_authority = False
                    response.result = "success"
                    response.error = ""
                    self.get_logger().info(
                        f"✗ 释放MCB控制权限 (task_id={request.task_id})"
                    )
                    
                else:
                    # 无效的操作类型
                    response.result = "failed"
                    response.error = f"invalid type: {request.type}, must be 'get' or 'release'"
                    self.get_logger().warn(f"拒绝无效类型: {request.type}")
            
            # 3. 记录到文件（调试用）
            self._log_to_file("MCB", request, response)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"处理控制权限请求时出错: {e}")
            import traceback
            traceback.print_exc()
            
            response.result = "failed"
            response.error = f"internal error: {str(e)}"
            return response
    
    def _estop_callback(self, msg):
        """
        急停消息回调
        
        消息格式：
          - emergency_stop: "Y"（急停）/ "N"（解除）
        """
        with self._estop_lock:
            if msg.emergency_stop.upper() == "Y":
                if not self._emergency_stop:
                    # 触发急停
                    self._emergency_stop = True
                    self.get_logger().error("🚨 收到急停信号！机械臂控制已冻结！")
                    
                    # 急停时自动释放MCB控制权限（安全措施）
                    with self._authority_lock:
                        if self._mcb_control_authority:
                            self._mcb_control_authority = False
                            self.get_logger().warn("⚠️  急停触发，自动释放MCB控制权限")
                            
            elif msg.emergency_stop.upper() == "N":
                if self._emergency_stop:
                    # 解除急停
                    self._emergency_stop = False
                    self.get_logger().info("✓ 急停解除，系统恢复正常")
                    self.get_logger().info("  （MCB需重新请求权限，R键需重新按下）")
            else:
                self.get_logger().warn(f"收到未知急停信号: {msg.emergency_stop}")
    
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
    
    def get_mcb_authority_status(self):
        """获取当前MCB控制权限状态（供外部查询）"""
        with self._authority_lock:
            return self._mcb_control_authority
    
    def get_estop_status(self):
        """获取当前急停状态（供外部查询）"""
        with self._estop_lock:
            return self._emergency_stop


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        server = ArmControlAuthorityServer()
        
        # 使用多线程执行器（支持并发服务调用）
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(server)
        
        print("\n" + "=" * 60)
        print("机械臂MCB控制权限管理服务器正在运行...")
        print("（R键控制由policy_controller本地管理）")
        print("按 Ctrl+C 停止")
        print("=" * 60 + "\n")
        
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n收到停止信号，关闭服务器...")
    except Exception as e:
        print(f"\n服务器异常退出: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'server' in locals():
            server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
