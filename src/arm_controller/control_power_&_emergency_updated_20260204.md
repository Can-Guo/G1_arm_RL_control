以下是测试急停和MCB控制权限功能的命令列表：

## 1. 编译接口包
```bash
cd /home/york/RL_arm_control
# 清理旧的编译产物
rm -rf build/robot_interfaces install/robot_interfaces
# 重新编译接口
colcon build --packages-select robot_interfaces --symlink-install
# 重新编译控制器（依赖接口）
colcon build --packages-select arm_controller --symlink-install
# Source 环境（每个新终端都需要）
source install/setup.bash

# source unitree_ros2's setup.sh for real robot control.
source src/pkg/unitree_ros2/setup.sh
```

## 2. 启动服务

### 2.1 启动权限管理服务器（必需）
```bash
# 在单独的终端中启动
ros2 run arm_controller arm_authority_server
```

### 2.2 启动控制器
```bash
python3 src/arm_controller/arm_controller/g1_dual_arm_policy_controller_ROS2_v3.py
```

### 2.3 run the arm_action_server_complied for target position and feedback

```bash
ros2 run arm_controller arm_action_server_complied
```

**注意：**
- arm_control_authority_server 必须先启动，用于管理MCB控制权限
- R键控制是本地管理的，不需要authority_server
- Z键急停是本地触发的，会发布到 `/msg_estop` 话题

## 3. MCB控制权限测试, Use a new terminal for temperary testing if you don't have a full function MCB for this test

**获取控制权限：**
```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower "{task_id: 'test_001', object: 'arm', type: 'get'}"
```

**释放控制权限：**
```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower "{task_id: 'test_002', object: 'arm', type: 'release'}"
```

**查询当前权限状态（无直接命令，通过UI观察MCB权限标志）**

## 4. 急停功能测试

**触发急停：**
```bash
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'Y'}" --once
```

**解除急停：**
```bash
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'N'}" --once
```

## 5. 综合测试场景

**场景1：初始状态（无权限）**
```bash
# 控制器启动后默认无权限
# 观察UI显示：R键:[ ] MCB:[ ] 最终权限:[ ] 机器人:[-]
# 提示："⚠️  无控制权限！按R键启用手动控制，或等待MCB授权"
```

**场景2：MCB授权自动启用输出**
```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower "{task_id: 'test_001', object: 'arm', type: 'get'}"
# 观察：机器人输出自动开启
# UI显示：R键:[ ] MCB:[✓] 最终权限:[✓] 机器人:[X]
# 提示："[MCB授权] MCB已授权，自动启用机器人输出"
```

**场景3：MCB释放强制关闭输出**
```bash
# 在机器人输出开启状态下，MCB释放控制权限
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower "{task_id: 'test_002', object: 'arm', type: 'release'}"
# 观察：机器人输出立即关闭（强制）
# UI显示：R键:[ ] MCB:[ ] 最终权限:[ ] 机器人:[-]
# 提示："[MCB释放] MCB已释放控制权限，强制关闭机器人输出"
```

**场景4：R键独立控制（忽略MCB状态）**
```bash
# 在MCB无授权的状态下，按R键启用手动控制
# 观察：机器人输出立即开启，不需要MCB授权
# UI显示：R键:[✓] MCB:[ ] 最终权限:[✓] 机器人:[X]
# 提示："[R键] 手动控制已启用" + "已自动开启机器人输出（忽略MCB状态）"
# **关键：R键独立工作，不受MCB状态影响**
```

**场景5：MCB释放时忽略R键状态**
```bash
# 前提：R键已启用，机器人正在运行
# UI显示：R键:[✓] MCB:[✓] 最终权限:[✓] 机器人:[X]

# MCB释放控制权限
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower "{task_id: 'test_003', object: 'arm', type: 'release'}"

# 观察：机器人输出立即关闭（即使R键仍启用）
# UI显示：R键:[✓] MCB:[ ] 最终权限:[✓] 机器人:[-]
# 提示："[MCB释放] MCB已释放控制权限，强制关闭机器人输出"
# **关键：MCB释放忽略R键状态，强制关闭输出（安全优先）**
```

**场景6：R键重启恢复控制**
```bash
# 前提：场景5之后，R键仍启用但输出已关闭
# UI显示：R键:[✓] MCB:[ ] 最终权限:[✓] 机器人:[-]

# 步骤1：按R键禁用
# UI显示：R键:[ ] MCB:[ ] 最终权限:[ ] 机器人:[-]

# 步骤2：按R键重新启用
# UI显示：R键:[✓] MCB:[ ] 最终权限:[✓] 机器人:[X]
# 提示："[R键] 手动控制已启用" + "已自动开启机器人输出（忽略MCB状态）"
# **关键：R键重启后可独立恢复控制**
```

**场景7：Z键手动急停**
```bash
# 在机器人运行状态下，按Z键触发急停
# UI显示：急停:[🚨] 机器人:[-]
# 提示："[Z键] 🚨 手动触发急停！所有控制已冻结"
# 观察：
#   - 所有输出立即停止
#   - 仿真也停止
#   - 位置锁定在当前位置

# 再次按Z键解除急停
# 提示："[Z键] ✓ 手动解除急停"
# 观察：系统恢复正常，需要重新启用控制
```

**场景8：ROS2话题急停**
```bash
# 触发急停
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'Y'}" --once
# 观察：与Z键效果相同，所有控制冻结
# UI显示：急停:[🚨] 机器人:[-]

# 解除急停
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'N'}" --once
# 观察：急停解除，系统恢复正常
```

## 6. 监控话题
```bash
# 监控位置发布
ros2 topic echo /g1/left_arm/position
ros2 topic echo /g1/right_arm/position

# 监控急停状态
ros2 topic echo /msg_estop

# 查看服务列表
ros2 service list | grep control_power
```

## 7. 控制权限逻辑说明（重要！）

### 7.1 权限架构（更新版）
```
R键控制：本地手动控制开关（独立，忽略MCB状态）
MCB控制：远程上位机授权（通过服务器管理）
最终权限 = R键使能 OR MCB授权（两者权限等级相同）
机器人输出 = 由MCB授权/释放事件 或 R键切换 直接控制
```

### 7.2 控制逻辑表（新逻辑）

| 事件 | R键状态 | MCB状态 | 最终权限 | 机器人输出 | 说明 |
|------|---------|---------|----------|-----------|------|
| 1. 初始状态 | ❌ | ❌ | ❌ | ❌ | 无控制权限 |
| 2. MCB授权(get) | - | ✅ | ✅ | **✅** | **自动启用输出** |
| 3. 按R键启用 | ✅ | - | ✅ | **✅** | **独立启用，忽略MCB** |
| 4. MCB释放(release) | - | ❌ | ❌ | **❌** | **强制关闭，忽略R键** |
| 5. R键重启 | ✅→❌→✅ | - | ✅ | **✅** | **独立控制，忽略MCB** |

**关键行为：**
- **MCB授权(get)** → 立即启用机器人输出
- **R键启用** → 立即启用机器人输出（不检查MCB状态）
- **MCB释放(release)** → 强制关闭机器人输出（即使R键仍启用）
- **R键重启** → 需先禁用再启用，重新控制输出

### 7.3 新功能：Z键手动急停

**按Z键：**
- 未急停时：触发急停，所有输出冻结
- 急停状态时：解除急停，恢复正常
- 自动发布急停消息到 `/msg_estop` 话题

**急停行为：**
1. 强制关闭所有机器人输出
2. 关闭仿真输出
3. 锁定当前位置（不接受新目标）
4. UI显示急停警告

### 7.4 键盘控制说明

| 按键 | 功能 | 说明 |
|------|------|------|
| R | 手动控制切换 | 独立控制，忽略MCB状态 |
| Z | 急停切换 | 手动触发/解除急停 |
| M | Action模式 | 切换Action Server控制 |
| T | 仿真输出 | 切换仿真显示 |
| WASD/QE | 左臂控制 | 手动模式有效 |
| IJKL/UO | 右臂控制 | 手动模式有效 |
| Esc/X | 退出 | 安全退出程序 |

### 7.5 测试验证序列

```bash
# 完整测试序列：
1. 启动控制器（初始状态）
   → UI: R键:[ ] MCB:[ ] 最终权限:[ ] 机器人:[-]

2. MCB授权（测试自动启用）
   $ ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
     "{task_id: 'test_001', object: 'arm', type: 'get'}"
   → UI: R键:[ ] MCB:[✓] 最终权限:[✓] 机器人:[X]
   → 提示："[MCB授权] MCB已授权，自动启用机器人输出"

3. MCB释放（测试强制关闭）
   $ ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
     "{task_id: 'test_002', object: 'arm', type: 'release'}"
   → UI: R键:[ ] MCB:[ ] 最终权限:[ ] 机器人:[-]
   → 提示："[MCB释放] MCB已释放控制权限，强制关闭机器人输出"

4. 按R键启用（测试独立控制）
   → UI: R键:[✓] MCB:[ ] 最终权限:[✓] 机器人:[X]
   → 提示："[R键] 手动控制已启用" + "已自动开启机器人输出"
   → **关键：即使MCB无授权，R键仍可独立控制**

5. MCB再次授权（测试双权限共存）
   $ ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
     "{task_id: 'test_003', object: 'arm', type: 'get'}"
   → UI: R键:[✓] MCB:[✓] 最终权限:[✓] 机器人:[X]
   → 提示："[MCB授权] MCB已授权，自动启用机器人输出"

6. 按Z键测试急停
   → UI: 急停:[🚨] 机器人:[-]
   → 提示："[Z键] 🚨 手动触发急停！所有控制已冻结"
   → 再次按Z解除：提示"[Z键] ✓ 手动解除急停"

7. MCB释放（测试强制关闭，忽略R键）
   $ ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
     "{task_id: 'test_004', object: 'arm', type: 'release'}"
   → UI: R键:[✓] MCB:[ ] 最终权限:[✓] 机器人:[-]
   → **关键：虽然R键仍启用，但机器人输出已关闭**

8. R键重启（测试重新启用）
   a) 按R禁用 → UI: R键:[ ] 机器人:[-]
   b) 按R启用 → UI: R键:[✓] 机器人:[X]
   → 机器人输出恢复
```

### 7.6 安全机制总结

**优先级排序：**
```
急停 (最高) > MCB释放 > R键控制 > MCB授权
```

**安全保障：**
1. ✅ 急停可通过Z键或ROS2话题触发
2. ✅ MCB释放立即强制关闭输出（安全第一）
3. ✅ R键提供应急独立控制
4. ✅ 所有状态变化都有日志和UI提示