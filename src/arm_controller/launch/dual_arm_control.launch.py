#!/usr/bin/env python3
"""dual_arm_control.launch.py - G1双臂控制器启动文件（仅控制器，保留curses交互界面）

功能：
  • 启动双臂控制器，保留curses交互界面用于调试
  • Action服务器需单独通过ros2 run启动

使用方法：
  终端1 - 启动控制器（本launch文件）：
    conda activate g1_vibe
    cd ~/RL_arm_control
    source install/setup.bash
    ros2 launch arm_controller dual_arm_control.launch.py
  
  终端2 - 启动Action服务器：
    conda activate g1_vibe
    cd ~/RL_arm_control
    source install/setup.bash
    ros2 run arm_controller arm_action_server_compiled

作者: York
日期: 2025-12-18
"""

import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成Launch描述"""
    
    # 获取源码目录（用于直接运行Python脚本）
    try:
        package_share = get_package_share_directory('arm_controller')
        # 从install目录回溯到src目录
        src_dir = str(Path(package_share).parent.parent.parent.parent / 'src' / 'arm_controller' / 'arm_controller')
    except:
        # 备用方案
        src_dir = str(Path.home() / 'RL_arm_control' / 'src' / 'arm_controller' / 'arm_controller')
    
    controller_script = str(Path(src_dir) / 'g1_dual_arm_policy_controller_ROS2_v3.py')
    action_server_script = str(Path(src_dir) / 'arm_control_action_server_compiled.py')
    
    # 检测Python解释器：优先使用conda环境，否则使用系统Python
    # 方法1: 检查CONDA_PREFIX环境变量
    conda_prefix = os.environ.get('CONDA_PREFIX', None)
    if conda_prefix:
        python_exec = str(Path(conda_prefix) / 'bin' / 'python3')
        print(f"[launch] 使用conda环境Python: {python_exec}")
    else:
        python_exec = sys.executable
        print(f"[launch] 使用系统Python: {python_exec}")
        print("[launch] ⚠️  警告: 未检测到conda环境，请先运行 'conda activate g1_vibe'")
    
    # 仅启动双臂控制器（保留curses交互界面）
    # emulate_tty=True 尝试提供伪终端支持curses
    dual_arm_controller_node = ExecuteProcess(
        cmd=[python_exec, controller_script],
        output='screen',
        name='g1_dual_arm_controller_v3',
        emulate_tty=True,  # 启用伪终端支持
    )
    
    return LaunchDescription([
        dual_arm_controller_node,
    ])
