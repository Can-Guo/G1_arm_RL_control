"""arm_controller - G1双臂RL控制器包

提供基于ROS2的机械臂控制功能，包括：
  • Action Server: 异步动作控制服务
  • RL Controller: 基于强化学习的双臂策略控制器
  • Configuration: 动作点配置管理
  • Test Tools: 测试客户端

作者: York 
日期: 2025-11-21
"""

import os

__version__ = '1.0.0'

def get_config_path(filename='actionpoints.yaml'):
    """获取配置文件路径
    
    优先级:
      1. ROS2 包安装路径 (colcon build 后)
      2. 开发环境相对路径
      3. 抛出异常
    
    Args:
        filename: 配置文件名
        
    Returns:
        配置文件的绝对路径
        
    Raises:
        FileNotFoundError: 找不到配置文件
    """
    # 尝试从 ROS2 包获取
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('arm_controller')
        config_path = os.path.join(pkg_share, 'config', filename)
        if os.path.exists(config_path):
            return config_path
    except Exception:
        pass
    
    # 尝试开发环境路径
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    dev_paths = [
        os.path.join(pkg_dir, '..', 'config', filename),
        os.path.join(pkg_dir, '..', '..', 'config', filename),
    ]
    
    for path in dev_paths:
        if os.path.exists(path):
            return os.path.abspath(path)
    
    raise FileNotFoundError(
        f"找不到配置文件: {filename}\n"
        f"请确保已运行 'colcon build' 或配置文件存在于开发目录"
    )


def get_model_path(model_name):
    """获取模型文件路径
    
    Args:
        model_name: 模型相对路径 (例如: 'ppo_left_10M_Oct09/...')
        
    Returns:
        模型文件的绝对路径
    """
    # 尝试从 ROS2 包获取
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('arm_controller')
        model_path = os.path.join(pkg_share, 'models', model_name)
        if os.path.exists(model_path):
            return model_path
    except Exception:
        pass
    
    # 尝试开发环境路径
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(pkg_dir, '..', 'models', model_name)
    if os.path.exists(model_path):
        return os.path.abspath(model_path)
    
    raise FileNotFoundError(f"找不到模型文件: {model_name}")


def get_mujoco_xml_path(xml_name='g1_29dof_with_hand_dual.xml'):
    """获取 MuJoCo XML 路径
    
    Args:
        xml_name: XML 文件名
        
    Returns:
        XML 文件的绝对路径
    """
    # 尝试从 ROS2 包获取
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('arm_controller')
        xml_path = os.path.join(pkg_share, 'unitree_mujoco', xml_name)
        if os.path.exists(xml_path):
            return xml_path
    except Exception:
        pass
    
    # 尝试开发环境路径
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(pkg_dir, '..', 'unitree_mujoco', xml_name)
    if os.path.exists(xml_path):
        return os.path.abspath(xml_path)
    
    raise FileNotFoundError(f"找不到 MuJoCo XML: {xml_name}")


# 导入子模块和常量
try:
    from . import g1_arm_rl_env
    from .g1_arm_rl_env import DISTANCE_THRESHOLD
except ImportError as e:
    # 开发模式兜底
    import warnings
    warnings.warn(f"无法导入 g1_arm_rl_env: {e}", ImportWarning)
    DISTANCE_THRESHOLD = 0.03


__all__ = [
    'get_config_path',
    'get_model_path',
    'get_mujoco_xml_path',
    'g1_arm_rl_env',
    'DISTANCE_THRESHOLD',
]
