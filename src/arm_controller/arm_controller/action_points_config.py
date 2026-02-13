#!/usr/bin/env python3
"""action_points_config.py - 动作点配置加载器

功能：
  • 加载 actionpoints.yaml 配置文件
  • 提供预设动作点查询接口
  • 验证动作点参数有效性

作者: York & Sunny Zhu
日期: 2025-11-14
"""

import os
from typing import Dict, Optional, Tuple

try:
    import yaml
except ImportError:
    yaml = None


class ActionPointsConfig:
    """动作点配置管理器"""
    
    def __init__(self, config_path: Optional[str] = None):
        """初始化配置管理器
        
        Args:
            config_path: YAML 配置文件路径（可选）
            
        Raises:
            FileNotFoundError: 当找不到配置文件时
            RuntimeError: 当 PyYAML 未安装时
        """
        self.action_points = {}
        
        # 检查 PyYAML 是否可用
        if yaml is None:
            raise RuntimeError(
                "[错误] PyYAML 未安装，无法加载配置文件\n"
                "请运行: pip install pyyaml"
            )
        
        # 尝试加载 YAML 配置
        if config_path and os.path.exists(config_path):
            self._load_from_yaml(config_path)
        else:
            # 获取当前脚本所在目录
            script_dir = os.path.dirname(os.path.abspath(__file__))
            
            # 尝试从默认位置加载（使用相对路径）
            default_paths = [
                # MCB标准路径: ~/sunny/hr_ros2_ws/src/config/
                os.path.join(script_dir, '../config/actionpoints.yaml'),
                
                # 向上两级到 hr_ros2_ws/src/config/
                os.path.join(script_dir, '../../config/actionpoints.yaml'),
                
                # 兼容旧路径 (开发环境备用)
                os.path.join(script_dir, 'ros2_ws/src/arm_control_interfaces/config/actionpoints.yaml'),
                os.path.join(script_dir, 'arm_control_interfaces/config/actionpoints.yaml'),
                os.path.join(script_dir, 'config/actionpoints.yaml'),
                
                # 绝对路径 (MCB环境)
                '/home/sunny/hr_ros2_ws/src/config/actionpoints.yaml',
                
                # 当前目录 (最后备用)
                os.path.join(script_dir, 'actionpoints.yaml'),
                'actionpoints.yaml'
            ]
            
            loaded = False
            for path in default_paths:
                if os.path.exists(path):
                    self._load_from_yaml(path)
                    loaded = True
                    break
            
            if not loaded:
                error_msg = (
                    "[错误] 未找到动作点配置文件 actionpoints.yaml\n"
                    f"已搜索以下路径:\n" +
                    "\n".join(f"  • {path}" for path in default_paths) +
                    "\n\n请确保配置文件存在于上述任一位置"
                )
                raise FileNotFoundError(error_msg)
    
    def _load_from_yaml(self, path: str):
        """从 YAML 文件加载配置
        
        Args:
            path: YAML 文件路径
            
        Raises:
            ValueError: 当配置文件格式错误时
            Exception: 当加载失败时
        """
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                
            if 'actionpoints' in data:
                self.action_points = data['actionpoints']
                print(f"[配置] 已加载 {len(self.action_points)} 个动作点: {path}")
            else:
                raise ValueError(
                    f"[错误] 配置文件格式错误: {path}\n"
                    "配置文件必须包含 'actionpoints' 根键"
                )
        
        except ValueError:
            raise
        except Exception as e:
            raise Exception(f"[错误] 加载配置文件失败: {path}\n原因: {e}")
    
    def get_action_point(self, name: str) -> Optional[Dict[str, float]]:
        """获取指定动作点配置
        
        Args:
            name: 动作点名称
            
        Returns:
            动作点配置字典，如果不存在返回 None
        """
        return self.action_points.get(name)
    
    def get_position(self, name: str) -> Optional[Tuple[Tuple[float, float, float], 
                                                         Tuple[float, float, float]]]:
        """获取指定动作点的位置坐标
        
        Args:
            name: 动作点名称
            
        Returns:
            ((x_l, y_l, z_l), (x_r, y_r, z_r)) 或 None
        """
        point = self.get_action_point(name)
        if point is None:
            return None
        
        left = (point['x_l'], point['y_l'], point['z_l'])
        right = (point['x_r'], point['y_r'], point['z_r'])
        return left, right
    
    def get_full_config(self, name: str) -> Optional[Dict]:
        """获取指定动作点的完整配置(包括位置和精度设置)
        
        Args:
            name: 动作点名称
            
        Returns:
            {
                'left': (x_l, y_l, z_l),
                'right': (x_r, y_r, z_r),
                'high_resolution': int  # 可选,如果配置中没有则为None
            }
            或 None (如果动作点不存在)
        """
        point = self.get_action_point(name)
        if point is None:
            return None
        
        config = {
            'left': (point['x_l'], point['y_l'], point['z_l']),
            'right': (point['x_r'], point['y_r'], point['z_r']),
            'high_resolution': point.get('high_resolution')  # 可选字段
        }
        
        return config
    
    def list_action_points(self) -> list:
        """列出所有可用的动作点名称"""
        return list(self.action_points.keys())
    
    def validate_action_point(self, name: str) -> bool:
        """验证动作点是否存在且有效
        
        Args:
            name: 动作点名称
            
        Returns:
            是否有效
        """
        point = self.get_action_point(name)
        if point is None:
            return False
        
        # 检查必需字段
        required_fields = ['x_l', 'y_l', 'z_l', 'x_r', 'y_r', 'z_r']
        return all(field in point for field in required_fields)
    
    def get_action_for_target(self, target: str, action: str) -> Optional[str]:
        """根据 target 和 action 获取对应的动作点名称
        
        Args:
            target: 目标类型（empty_box, product_box）
            action: 动作类型（pick, place）
            
        Returns:
            动作点名称，如果不存在返回 None
        """
        # 构造动作点名称
        if target and action:
            point_name = f"{action}_{target}"
            if point_name in self.action_points:
                return point_name
        
        return None


# 全局单例实例
_config_instance = None

def get_config(reload: bool = False) -> ActionPointsConfig:
    """获取配置单例实例
    
    Args:
        reload: 是否重新加载配置
        
    Returns:
        配置实例
    """
    global _config_instance
    
    if _config_instance is None or reload:
        _config_instance = ActionPointsConfig()
    
    return _config_instance


# 快捷函数
def get_action_point(name: str) -> Optional[Dict[str, float]]:
    """获取指定动作点配置（快捷函数）"""
    return get_config().get_action_point(name)


def get_position(name: str) -> Optional[Tuple[Tuple[float, float, float], 
                                                Tuple[float, float, float]]]:
    """获取指定动作点位置（快捷函数）"""
    return get_config().get_position(name)


def list_action_points() -> list:
    """列出所有动作点（快捷函数）"""
    return get_config().list_action_points()


if __name__ == '__main__':
    """测试配置加载"""
    print("=" * 60)
    print("动作点配置测试")
    print("=" * 60)
    
    # 加载配置
    config = get_config()
    
    # 列出所有动作点
    print(f"\n可用动作点 ({len(list_action_points())} 个):")
    for name in list_action_points():
        print(f"  • {name}")
    
    # 测试获取动作点
    print("\n详细配置:")
    for name in ['arm_home', 'pick_empty_box', 'pick_product_box']:
        point = get_action_point(name)
        if point:
            print(f"\n{name}:")
            print(f"  左臂: ({point['x_l']}, {point['y_l']}, {point['z_l']})")
            print(f"  右臂: ({point['x_r']}, {point['y_r']}, {point['z_r']})")
    
    # 测试根据 target/action 查找
    print("\n动作映射测试:")
    test_cases = [
        ('empty_box', 'pick'),
        ('empty_box', 'place'),
        ('product_box', 'pick'),
        ('product_box', 'place'),
    ]
    
    for target, action in test_cases:
        point_name = config.get_action_for_target(target, action)
        if point_name:
            print(f"  {target} + {action} → {point_name}")
        else:
            print(f"  {target} + {action} → [未找到]")
    
    print("\n" + "=" * 60)
