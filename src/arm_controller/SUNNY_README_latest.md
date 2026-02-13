# G1 双臂控制系统

基于 ROS2 + MuJoCo + PPO 的 G1 机器人双臂强化学习控制系统  
**版本**: v3.1 (ROS2 Standard)  
**更新日期**: 2025-12-19

---

## 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS2**: Humble
- **Python**: 3.10+
- **Conda 环境**: g1_vibe (推荐)
- **依赖包**: stable-baselines3, mujoco, gymnasium

---

## 首次部署

### 1. 构建 ROS2 包

```bash
cd ~/RL_arm_control
colcon build --packages-select robot_interfaces arm_controller --symlink-install
source install/setup.bash
```

### 2. 安装 Python 依赖（Conda 环境）

```bash
conda activate g1_vibe
pip install stable-baselines3==2.7.0 mujoco>=3.0.0 gymnasium>=1.0.0
```

### 3. 验证安装

```bash
# 检查 ROS2 包
ros2 pkg list | grep arm_controller

# 检查可执行文件
ros2 pkg executables arm_controller

# 检查 Action 接口
ros2 interface list | grep ArmControl

# 检查 Python 导入
python3 -c "from arm_controller import DISTANCE_THRESHOLD; print('✓ OK')"
```

---

## 运行系统

系统需要 **3 个终端** 同时运行：

### 终端 1 - 双臂控制器（仿真/实机）

```bash
conda activate g1_vibe
cd ~/RL_arm_control
source install/setup.bash
python3 src/arm_controller/arm_controller/g1_dual_arm_policy_controller_ROS2_v3.py
```

**功能**:
- 加载 PPO 策略模型（左臂 + 右臂）
- 启动 MuJoCo 仿真渲染器
- 订阅目标位置话题（从 Action Server）
- 发布当前位置话题（反馈给 Action Server）
- 发送关节控制指令到实机（按 `r` 启用）

**键盘控制**:
- `m` - 切换手动/Action 模式
- `r` - 切换机器人输出（仿真→实机）
- `t` - 切换仿真输出
- `W/S/A/D/Q/E` - 左臂控制
- `I/K/J/L/U/O` - 右臂控制

---

### 终端 2 - Action Server

```bash
conda activate g1_vibe
cd ~/RL_arm_control
source install/setup.bash
ros2 run arm_controller arm_action_server_compiled
```

**功能**:
- 提供 ROS2 Action 接口 (`/action_arm`)
- 接收外部控制指令（MCB/测试客户端）
- 转发目标位置到控制器
- 接收位置反馈并返回执行结果
- 超时保护（30秒）

---

### 终端 3 - 测试客户端（可选）

```bash
conda activate g1_vibe
cd ~/RL_arm_control
source install/setup.bash
ros2 run arm_controller arm_test_client
```

**功能**:
- 发送测试指令到 Action Server
- 使用预设动作点（安全点、初始位置等）
- 自定义目标位置测试
- 批量测试和报告生成

**常用命令**:
- `1-4` - 移动到预设点
- `P x_l y_l z_l x_r y_r z_r [精度]` - 自定义位置
- `T` - 超时停止测试
- `B` - 批量测试所有预设点
- `H` - 帮助信息

---

## 快速启动命令

### 完整系统启动（3 终端）

```bash
# 终端 1: 双臂控制器
conda activate g1_vibe && cd ~/RL_arm_control && source install/setup.bash && \
python3 src/arm_controller/arm_controller/g1_dual_arm_policy_controller_ROS2_v3.py

# 终端 2: Action Server
conda activate g1_vibe && cd ~/RL_arm_control && source install/setup.bash && \
ros2 run arm_controller arm_action_server_compiled

# 终端 3: 测试客户端
conda activate g1_vibe && cd ~/RL_arm_control && source install/setup.bash && \
ros2 run arm_controller arm_test_client
```

---

## 首次部署

### 1. 构建 ROS2 包

```bash
cd ~/RL_arm_control
colcon build --packages-select robot_interfaces arm_controller --symlink-install
source install/setup.bash
```

### 2. 安装 Python 依赖（Conda 环境）

```bash
conda activate g1_vibe
pip install stable-baselines3==2.7.0 mujoco>=3.0.0 gymnasium>=1.0.0
```

### 3. 验证安装

```bash
# 检查 ROS2 包
ros2 pkg list | grep arm_controller

# 检查可执行文件
ros2 pkg executables arm_controller

# 检查 Action 接口
ros2 interface list | grep ArmControl

# 检查 Python 导入
python3 -c "from arm_controller import DISTANCE_THRESHOLD; print('✓ OK')"
```

---

## 核心特性

- ✅ **ROS2 Action Server**: 接收外部控制指令（使用 `ros2 run` 启动）
- ✅ **双臂 RL 控制器**: 独立左右臂 PPO 策略（使用 Python 脚本启动）
- ✅ **测试客户端**: 预设动作点和自定义测试（使用 `ros2 run` 启动）
- ✅ **实时位置反馈**: 10Hz 发布当前手掌位置
- ✅ **双控制模式**: 手动键盘控制 + Action 自动模式
- ✅ **安全机制**: 碰撞检测、超时保护、优雅关闭
- ✅ **动态精度**: 支持高/中/低精度模式（容差 3/6/9cm）

---

## 项目结构

```
RL_arm_control/                              # ROS2 工作空间根目录
├── src/                                     # 源码目录
│   ├── arm_controller/                      # arm_controller ROS2 包
│   │   ├── arm_controller/                  # Python 模块
│   │   │   ├── __init__.py                  # 包初始化（导出 DISTANCE_THRESHOLD）
│   │   │   ├── g1_dual_arm_policy_controller_ROS2_v3.py  # 双臂控制器（Python脚本）
│   │   │   ├── arm_control_action_server_compiled.py     # Action Server
│   │   │   ├── test_arm_client_simple.py                 # 测试客户端
│   │   │   ├── action_points_config.py      # 预设动作点配置
│   │   │   ├── g1_arm_rl_env.py             # RL 环境定义
│   │   │   └── dual_arm_controller_wrapper.py # ROS2 运行包装器
│   │   │
│   │   ├── models/                          # RL 策略模型
│   │   │   ├── ppo_left_10M_Oct09/          # 左臂 PPO 模型
│   │   │   │   └── 20251010-100658-lArm/
│   │   │   │       └── ppo_g1_left_final.zip
│   │   │   └── ppo_right_5M_Aug28/          # 右臂 PPO 模型
│   │   │       └── ppo_g1_right_5000k.zip
│   │   │
│   │   ├── unitree_mujoco/                  # MuJoCo 仿真模型
│   │   │   ├── g1_29dof_with_hand.xml       # 单臂模型
│   │   │   ├── g1_29dof_with_hand_dual.xml  # 双臂模型
│   │   │   └── meshes/                      # 3D 网格文件
│   │   │
│   │   ├── config/                          # 配置文件
│   │   │   └── actionpoints.yaml            # 预设动作点
│   │   │
│   │   ├── launch/                          # 启动脚本（可选）
│   │   │   ├── start_action_server.sh
│   │   │   └── start_dual_arm_controller.sh
│   │   │
│   │   ├── package.xml                      # ROS2 包配置
│   │   ├── setup.py                         # Python 包安装配置
│   │   ├── setup.cfg                        # Python 包配置
│   │   └── SUNNY_README_latest.md           # 本文档
│   │
│   ├── robot_interfaces/                    # Action 接口定义包
│   │   ├── action/
│   │   │   └── ArmControl.action            # 双臂控制 Action 定义
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── pkg/unitree_ros2/                    # Unitree DDS 配置（可选）
│
├── install/                                 # ROS2 安装目录（编译后）
├── build/                                   # 编译临时文件
├── log/                                     # 日志文件
└── run_dual_arm_controller.sh               # 启动辅助脚本（可选）
```

---

## ROS2 话题与接口

### Action 接口
- **名称**: `/action_arm`
- **类型**: `robot_interfaces/action/ArmControl`
- **定义**:
  ```
  # Goal
  string target             # 目标类型: empty_box/product_box/''
  string action             # 动作类型: move/pick/place/''
  int32 high_resolution     # 精度模式: 1(高3cm)/2(中6cm)/3(低9cm)
  float64 x_l, y_l, z_l     # 左臂目标位置 [m]
  float64 x_r, y_r, z_r     # 右臂目标位置 [m]
  ---
  # Result
  string result             # 执行结果: success/failed
  string error              # 错误信息
  float64 x_l, y_l, z_l     # 左臂最终位置 [m]
  float64 x_r, y_r, z_r     # 右臂最终位置 [m]
  ---
  # Feedback
  float64 x_l, y_l, z_l     # 左臂当前位置 [m]
  float64 x_r, y_r, z_r     # 右臂当前位置 [m]
  ```

### 发布话题（控制器 → Action Server）
- `/g1/left_arm/position` (Float64MultiArray) - 左臂当前位置 [x, y, z]
- `/g1/right_arm/position` (Float64MultiArray) - 右臂当前位置 [x, y, z]

### 订阅话题（控制器 ← Action Server）
- `/g1/left_arm/goal` (Float64MultiArray) - 左臂目标位置 [x, y, z]
- `/g1/right_arm/goal` (Float64MultiArray) - 右臂目标位置 [x, y, z]
- `/g1/arm/distance_threshold` (Float64MultiArray) - 动态距离阈值 [threshold]

### 控制话题（控制器 → 机器人）
- `/arm_sdk` (unitree_hg/msg/LowCmd) - 机器人关节控制指令

---

## 控制器操作

### 键盘控制（手动模式）
```
左臂控制:
  W/S - 前进/后退 (X)     Q/E - 上升/下降 (Z)
  A/D - 左移/右移 (Y)

右臂控制:
  I/K - 前进/后退 (X)     U/O - 上升/下降 (Z)
  J/L - 左移/右移 (Y)

模式切换:
  M - 切换手动/Action模式
  R - 切换机器人输出（启用后发送到 /arm_sdk）
  T - 切换仿真输出
  
速度调节:
  [/] - 降低/提高控制速率
  ,/. - 降低/提高速率倍数
```

---

## 预设动作点

配置文件: `config/actionpoints.yaml`

| 编号 | 名称 | 描述 | 左臂位置 | 右臂位置 | 精度 |
|------|------|------|----------|----------|------|
| 1 | arm_safety | 安全点 | (0.1, 0.25, 0.70) | (0.1, -0.25, 0.70) | 中 |
| 2 | arm_home | 初始位置 | (0.2, 0.3, 0.75) | (0.2, -0.3, 0.75) | 中 |
| 3 | arm_open_prepare | 开箱准备 | (0.3, 0.25, 0.85) | (0.3, -0.25, 0.85) | 高 |
| 4 | arm_open_prepare_for_box | 箱体开箱准备 | (0.25, 0.2, 0.80) | (0.25, -0.2, 0.80) | 高 |

---

## 常见问题排查

### 1. Action Server 启动失败

**问题**: 提示缺少 Action 接口
```bash
# 解决方案：重新编译接口
cd ~/RL_arm_control
colcon build --packages-select robot_interfaces arm_controller --symlink-install
source install/setup.bash
```

### 2. 控制器无法连接

**问题**: 无法找到话题
```bash
# 检查话题是否发布
ros2 topic list | grep arm

# 检查 DDS 配置
echo $RMW_IMPLEMENTATION  # 应为: rmw_cyclonedds_cpp
echo $ROS_DOMAIN_ID       # 应为: 0
```

### 3. Python 模块导入失败

**问题**: `ModuleNotFoundError: No module named 'stable_baselines3'`
```bash
# 解决方案：安装依赖
conda activate g1_vibe
pip install stable-baselines3==2.7.0 mujoco>=3.0.0 gymnasium>=1.0.0
```

### 4. 导入警告

**问题**: `[警告] 无法导入 g1_arm_rl_env` 或 `[警告] 无法导入 action_points_config`
```bash
# 解决方案：确保使用正确的导入路径并重新构建
colcon build --packages-select arm_controller --symlink-install
source install/setup.bash
```

### 5. 模型文件未找到

**问题**: `找不到模型文件`
```bash
# 检查模型路径
ls -la src/arm_controller/models/ppo_left_10M_Oct09/20251010-100658-lArm/
ls -la src/arm_controller/models/ppo_right_5M_Aug28/

# 确保使用 ROS2 包路径（自动处理）
python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('arm_controller'))"
```

---

## MCB 部署指南

### 1. 迁移工作空间

```bash
# 从开发电脑复制到 MCB（排除临时文件）
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' \
  ~/RL_arm_control/ mcb@MCB_IP:/path/to/workspace/
```

### 2. MCB 端编译

```bash
# 在 MCB 上
cd /path/to/workspace
colcon build --packages-select robot_interfaces arm_controller --symlink-install
source install/setup.bash
```

### 3. MCB 端运行

```bash
# 终端 1: 双臂控制器
cd /path/to/workspace
source install/setup.bash
python3 src/arm_controller/arm_controller/g1_dual_arm_policy_controller_ROS2_v3.py

# 终端 2: Action Server
cd /path/to/workspace
source install/setup.bash
ros2 run arm_controller arm_action_server_compiled
```

> **注意**: MCB 端通常不需要测试客户端，直接接收 MCB 主控的 Action 指令

---

## 技术栈

- **ROS2**: Humble
- **DDS**: CycloneDDS (rmw_cyclonedds_cpp)
- **Python**: 3.10+
- **RL Framework**: stable-baselines3 2.7.0 (PPO)
- **仿真引擎**: MuJoCo 3.3.5
- **RL 环境**: gymnasium 1.2.0
- **Conda 环境**: g1_vibe

---

## 版本历史

- **v3.1** (2025-12-19) - ROS2 标准化
  - ✅ 双臂控制器使用 Python 脚本启动（支持 Conda 环境）
  - ✅ Action Server 使用 `ros2 run` 启动（标准 ROS2 方式）
  - ✅ 测试客户端使用 `ros2 run` 启动（独立运行）
  - ✅ 修复导入警告（g1_arm_rl_env, action_points_config）
  - ✅ 优化 `__init__.py` 导出机制
  - ✅ 完善文档和快速启动命令

- **v3.0** (2025-11-24) - Production Ready
  - ✅ 相对路径启动脚本（零配置迁移）
  - ✅ 优化关闭流程（避免 ROS2 异常）
  - ✅ 双环境支持（Conda + MCB 原生）
  - ✅ 完整部署文档

- **v2.0** (2025-11-17) - ROS2 标准化
  - ✅ 标准 ament_python 包结构
  - ✅ robot_interfaces Action 接口
  - ✅ 路径修复和依赖管理

- **v1.0** (2025-11-13) - 初始版本
  - 基础 ROS2 Action 接口
  - PPO 双臂控制器

---

## 相关文档

- **[Arm_control_protocol_v1.0.md](Arm_control_protocol_v1.0.md)** - 通信协议规范（MCB 集成）
- **ROS2 Humble 文档**: https://docs.ros.org/en/humble/
- **stable-baselines3 文档**: https://stable-baselines3.readthedocs.io/

---

## 作者与维护

**作者**: York  
**更新**: 2025-12-19  
**联系**: 1739950387@qq.com

---

