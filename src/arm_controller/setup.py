from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'arm_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 配置文件
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml') if os.path.exists('config') else []),
        # Launch 文件
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py') if os.path.exists('launch') else []),
        # MuJoCo 模型
        (os.path.join('share', package_name, 'unitree_mujoco'), 
            glob('unitree_mujoco/*.xml') if os.path.exists('unitree_mujoco') else []),
        # RL模型文件（左臂）
        (os.path.join('share', package_name, 'models/ppo_left_10M_Oct09/20251010-100658-lArm'),
            glob('models/ppo_left_10M_Oct09/20251010-100658-lArm/*.zip') if os.path.exists('models/ppo_left_10M_Oct09/20251010-100658-lArm') else []),
        # RL模型文件（右臂）
        (os.path.join('share', package_name, 'models/ppo_right_5M_Aug28'),
            glob('models/ppo_right_5M_Aug28/*.zip') if os.path.exists('models/ppo_right_5M_Aug28') else []),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        'numpy',
        'stable-baselines3>=2.7.0',
        'mujoco>=3.0.0',
        'gymnasium>=1.0.0',
    ],
    zip_safe=True,
    maintainer='York',
    maintainer_email='1739950387@qq.com',
    description='G1双臂RL控制器',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            # unified control authority server (合并MCB和R键权限管理)
            'arm_action_authority_server = arm_controller.arm_control_action_authority_server_compiled:main',
            # Action Server
            # 'arm_action_server = arm_controller.arm_control_action_server:main',
            'arm_action_server_compiled = arm_controller.arm_control_action_server_compiled:main',
            
            # Controller
            'dual_arm_controller = arm_controller.g1_dual_arm_policy_controller_ROS2_v3:main',
            
            # Authority Server (新增: 控制权限管理服务器)
            'arm_authority_server = arm_controller.arm_control_authority_server:main',
            
            # Test Client
            'arm_test_client = arm_controller.test_arm_client_simple:main',
        ],
    },
)
