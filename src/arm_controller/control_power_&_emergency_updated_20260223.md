# Unitree G1 机械臂控制系统 - 用户手册与测试指南

**版本**: 2.0 (统一服务器架构)  
**更新日期**: 2026-02-23  
**适用机型**: Unitree G1 双臂机器人

---

## 📋 目录

1. [系统架构](#系统架构)
2. [编译接口包](#编译接口包)
3. [启动服务](#启动服务)
4. [控制权限逻辑](#控制权限逻辑)
5. [MCB控制权限测试](#mcb控制权限测试)
6. [急停功能测试](#急停功能测试)
7. [综合测试场景](#综合测试场景)
8. [监控与调试](#监控与调试)
9. [故障排除](#故障排除)

---

## 系统架构

### 架构升级说明

**旧架构（3终端模式 - 已废弃）**:
- 终端1: `arm_control_authority_server` (权限管理)
- 终端2: `arm_control_action_server_compiled` (Action服务)
- 终端3: `g1_dual_arm_policy_controller_ROS2_v3` (控制器)

**新架构（2终端模式 - 当前版本）**:
- **终端1**: `arm_control_action_authority_server_compiled` (统一服务器 = 权限管理 + Action服务)
- **终端2**: `g1_dual_arm_policy_controller_ROS2_v3` (Policy控制器)

### 核心组件

#### 1️⃣ 统一服务器 (`arm_control_action_authority_server_compiled`)

**功能整合**:
- ✅ MCB控制权限管理（ControlPower服务）
- ✅ 急停监听与状态广播
- ✅ 双臂控制Action服务（带权限验证）
- ✅ 实时位置反馈与目标设置

**服务接口**:
- `/srv_control_power_transport` (ControlPower) - MCB权限请求

**Action接口**:
- `action_arm` (ArmControl) - 双臂控制Action

**订阅话题**:
- `/msg_estop` (Estop) - 急停信号
- `/g1/left_arm/position` (Float64MultiArray) - 左臂位置反馈
- `/g1/right_arm/position` (Float64MultiArray) - 右臂位置反馈

**发布话题**:
- `/g1/arm/mcb_control_authority` (Bool) - MCB权限状态
- `/g1/arm/emergency_stop` (Bool) - 急停状态
- `/g1/left_arm/goal` (Float64MultiArray) - 左臂目标位置
- `/g1/right_arm/goal` (Float64MultiArray) - 右臂目标位置
- `/g1/arm/distance_threshold` (Float64MultiArray) - 距离阈值

#### 2️⃣ Policy控制器 (`g1_dual_arm_policy_controller_ROS2_v3`)

**核心功能**:
- ✅ RL策略推理与执行
- ✅ R键本地控制管理（独立于MCB）
- ✅ MCB远程控制订阅
- ✅ Z键本地急停触发/解除
- ✅ 机械臂低层关节控制

**发布话题**:
- `/arm_sdk` (LowCmd) - 低层关节控制
- `/g1/left_arm/position` (Float64MultiArray) - 左臂位置反馈
- `/g1/right_arm/position` (Float64MultiArray) - 右臂位置反馈

**订阅话题**:
- `/g1/arm/mcb_control_authority` (Bool) - MCB权限状态
- `/g1/arm/emergency_stop` (Bool) - 急停状态
- `/g1/left_arm/goal` (Float64MultiArray) - 左臂目标（Action模式）
- `/g1/right_arm/goal` (Float64MultiArray) - 右臂目标（Action模式）
- `/g1/arm/distance_threshold` (Float64MultiArray) - 距离阈值

---

## 编译接口包

### 步骤1：清理旧编译产物（推荐）

```bash
cd /home/york/RL_arm_control

# 清理旧的接口编译产物
rm -rf build/robot_interfaces install/robot_interfaces

# 清理旧的控制器编译产物（可选）
rm -rf build/arm_controller install/arm_controller
```

### 步骤2：编译接口包

```bash
# 编译 robot_interfaces 接口包（必需）
colcon build --packages-select robot_interfaces --symlink-install

# 编译 arm_controller 控制器包
colcon build --packages-select arm_controller --symlink-install
```

### 步骤3：配置环境变量

```bash
# Source ROS2 环境（每个新终端都需要）
source install/setup.bash

# Source unitree_ros2 环境（实机控制必需，仿真可选）
source src/pkg/unitree_ros2/setup.sh
```

**⚠️ 重要提示**:
- 每次打开新终端都需要执行 `source install/setup.bash`
- 实机控制必须执行 `source src/pkg/unitree_ros2/setup.sh`
- 编译成功后会在 `install/` 目录下看到对应的包

---

## 启动服务

### 🚀 标准启动流程（2终端模式）

#### 终端1：启动统一服务器（必需）

```bash
cd /home/york/RL_arm_control
source install/setup.bash

# 启动统一服务器（权限管理 + Action服务）
ros2 run arm_controller arm_control_action_authority_server_compiled
```

**预期输出**:
```
======================================================================
统一机械臂控制服务器已启动
----------------------------------------------------------------------
权限管理服务:
  - /srv_control_power_transport (ControlPower) - MCB权限请求
Action服务:
  - action_arm (ArmControl) - 双臂控制Action
订阅话题:
  - /msg_estop (Estop) - 急停信号
  - /g1/left_arm/position, /g1/right_arm/position - 位置反馈
发布话题:
  - /g1/arm/mcb_control_authority (Bool) - MCB权限状态
  - /g1/arm/emergency_stop (Bool) - 急停状态
  - /g1/left_arm/goal, /g1/right_arm/goal - 目标位置
----------------------------------------------------------------------
初始状态: MCB权限=False, 急停=False
======================================================================
```

#### 终端2：启动Policy控制器（必需）

```bash
cd /home/york/RL_arm_control
source install/setup.bash

# 实机控制需要source unitree环境
source src/pkg/unitree_ros2/setup.sh

# 启动Policy控制器
python3 src/arm_controller/arm_controller/g1_dual_arm_policy_controller_ROS2_v3.py
```

**预期输出**:
```
[policy] 已加载 PPO 模型: ...
[dual_env] 已加载双臂 XML: ...
[dual_env] 碰撞组配置: 左臂=XX 右臂=XX 保护=XX
[ROS2] 新架构：R键本地管理 + MCB远程管理
[ROS2] 控制权限 = R键使能 OR MCB授权
[ROS2] 急停拥有最高优先级
[robot][ros2] ROS2 双臂模式激活 (V3 - Action集成版)
```

**界面说明**:
```
G-1 双臂 RL 控制器 V3 (MCB集成) – [R]手动控制 [Z]急停 [M]Action模式 [T]仿真 [Esc]退出
控制模式:[手动]  急停:[ ]  R键:[ ]  MCB:[ ]  最终权限:[ ]  输出→仿真:[X]  机器人:[-]
⚠️  无控制权限！按R键启用手动控制，或等待MCB授权
```

---

## 控制权限逻辑

### 权限架构（双通道设计）

```
┌─────────────────────────────────────────────────────────────┐
│                     控制权限来源                              │
├─────────────────────────────────────────────────────────────┤
│  1. R键控制（本地）：Policy控制器本地管理，独立工作           │
│  2. MCB控制（远程）：统一服务器管理，通过服务请求             │
│                                                              │
│  最终控制权限 = R键使能 OR MCB授权                           │
│  （两者权限等级相同，任意一个开启即可控制）                   │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                   安全保护机制                               │
├─────────────────────────────────────────────────────────────┤
│  优先级排序：急停 > MCB释放 > R键控制 > MCB授权              │
│                                                              │
│  ⚠️ 急停状态（最高优先级）：                                 │
│     - Z键触发或ROS2话题触发                                  │
│     - 强制关闭所有机器人输出                                 │
│     - 关闭仿真输出                                           │
│     - 锁定当前位置（不接受新目标）                           │
│     - 自动释放MCB权限                                        │
└─────────────────────────────────────────────────────────────┘
```

### 控制逻辑详解

#### 状态变量说明

| 状态变量 | 管理位置 | 说明 |
|---------|---------|------|
| `R键使能` | Policy控制器本地 | 按R键切换，独立工作 |
| `MCB授权` | 统一服务器管理 | 通过服务请求获取/释放 |
| `最终权限` | 逻辑OR运算 | R键使能 OR MCB授权 |
| `急停状态` | 双向同步 | Z键本地 + ROS2话题远程 |
| `机器人输出` | 自动控制 | 由MCB事件/R键切换自动管理 |

#### 机器人输出控制逻辑

**自动启用输出的情况**:
1. ✅ MCB授权（`get`）→ 自动开启机器人输出
2. ✅ R键启用 → 自动开启机器人输出（忽略MCB状态）

**强制关闭输出的情况**:
1. ❌ 急停触发 → 立即关闭所有输出（最高优先级）
2. ❌ MCB释放（`release`）→ 强制关闭机器人输出（忽略R键状态）

**恢复输出的方法**:
- 急停后：先解除急停，再按R键或请求MCB授权
- MCB释放后（R键仍启用）：需要R键重启（禁用→启用）才能恢复输出

### 控制权限状态表

| 事件 | R键状态 | MCB状态 | 最终权限 | 机器人输出 | 急停状态 | 说明 |
|------|---------|---------|----------|-----------|---------|------|
| **初始状态** | ❌ | ❌ | ❌ | ❌ | ❌ | 无控制权限 |
| **MCB授权(get)** | - | ✅ | ✅ | **✅** | ❌ | 自动启用输出 |
| **按R键启用** | ✅ | - | ✅ | **✅** | ❌ | 独立启用，忽略MCB |
| **MCB释放(release)** | - | ❌ | ❌ | **❌** | ❌ | 强制关闭，忽略R键 |
| **R键重启** | ✅→❌→✅ | - | ✅ | **✅** | ❌ | 独立控制，忽略MCB |
| **Z键急停** | - | - | - | **❌** | **✅** | 最高优先级，冻结所有 |
| **Z键解除急停** | - | - | - | ❌ | ❌ | 需重新启用控制 |
| **急停+MCB授权** | - | ✅ | ✅ | **❌** | **✅** | 急停优先，自动释放MCB |

**图例说明**:
- ✅ = 启用/成功
- ❌ = 禁用/失败
- `-` = 不变/不影响

---

## MCB控制权限测试

### 测试前提

确保统一服务器已启动（终端1）

### 测试命令

#### 1. 获取控制权限

```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'test_001', object: 'arm', type: 'get'}"
```

**预期响应**:
```yaml
result: 'success'
error: ''
```

**统一服务器日志**:
```
[MCB权限] ✓ 授予控制权限 (task_id=test_001)
```

**Policy控制器UI**:
```
MCB:[✓]  最终权限:[✓]  机器人:[X]
[MCB授权] MCB已授权，自动启用机器人输出
```

#### 2. 释放控制权限

```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'test_002', object: 'arm', type: 'release'}"
```

**预期响应**:
```yaml
result: 'success'
error: ''
```

**统一服务器日志**:
```
[MCB权限] ✗ 释放控制权限 (task_id=test_002)
```

**Policy控制器UI**:
```
MCB:[ ]  最终权限:[ ]  机器人:[-]
[MCB释放] MCB已释放控制权限，强制关闭机器人输出
```

#### 3. 测试Hand控制（忽略）

```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'test_003', object: 'hand', type: 'get'}"
```

**预期响应**:
```yaml
result: 'success'
error: 'hand control ignored (not supported)'
```

#### 4. 测试无效参数

```bash
# 无效对象
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'test_004', object: 'leg', type: 'get'}"

# 预期响应: result: 'failed', error: 'invalid object: leg'
```

```bash
# 无效类型
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'test_005', object: 'arm', type: 'reset'}"

# 预期响应: result: 'failed', error: 'invalid type: reset'
```

---

## 急停功能测试

### Z键本地急停（推荐）

在Policy控制器界面中：

1. **触发急停**：按 `Z` 键
   - 预期显示：`急停:[🚨]  机器人:[-]`
   - 提示：`[Z键] 🚨 手动触发急停！所有控制已冻结`

2. **解除急停**：再次按 `Z` 键
   - 预期显示：`急停:[ ]  机器人:[-]`
   - 提示：`[Z键] ✓ 手动解除急停`

### ROS2话题急停（远程）

#### 触发急停

```bash
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'Y'}" --once
```

**预期效果**:
- 统一服务器日志：`[急停] 🚨 收到急停信号！所有控制已冻结！`
- Policy控制器UI：`急停:[🚨]  机器人:[-]`
- 自动释放MCB权限
- 所有机器人输出关闭
- 仿真输出关闭
- 位置锁定在当前位置

#### 解除急停

```bash
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'N'}" --once
```

**预期效果**:
- 统一服务器日志：`[急停] ✓ 急停解除，系统恢复正常`
- Policy控制器UI：`急停:[ ]  机器人:[-]`
- 需要重新启用控制（R键或MCB授权）

### 急停行为验证

**急停状态下的行为**:
1. ❌ 拒绝所有Action请求
2. ❌ 拒绝所有外部目标设置
3. ✅ 发送当前位置作为目标（保持不动）
4. ✅ 自动释放MCB权限（如果之前有授权）
5. ✅ 强制关闭机器人输出（忽略R键状态）

---

## 综合测试场景

### 场景1：初始状态验证

**操作步骤**:
1. 启动统一服务器（终端1）
2. 启动Policy控制器（终端2）

**预期UI显示**:
```
控制模式:[手动]  急停:[ ]  R键:[ ]  MCB:[ ]  最终权限:[ ]  机器人:[-]
⚠️  无控制权限！按R键启用手动控制，或等待MCB授权
```

**验证点**:
- ✅ 所有权限标志为空
- ✅ 机器人输出关闭
- ✅ 显示无权限提示

---

### 场景2：MCB授权自动启用输出

**操作步骤**:
```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'test_001', object: 'arm', type: 'get'}"
```

**预期UI显示**:
```
R键:[ ]  MCB:[✓]  最终权限:[✓]  机器人:[X]
[MCB授权] MCB已授权，自动启用机器人输出
```

**验证点**:
- ✅ MCB权限标志显示 `✓`
- ✅ 最终权限标志显示 `✓`
- ✅ 机器人输出自动开启 `[X]`
- ✅ 可以通过键盘控制机械臂移动

---

### 场景3：MCB释放强制关闭输出

**前提**：场景2完成，MCB已授权，机器人输出开启

**操作步骤**:
```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'test_002', object: 'arm', type: 'release'}"
```

**预期UI显示**:
```
R键:[ ]  MCB:[ ]  最终权限:[ ]  机器人:[-]
[MCB释放] MCB已释放控制权限，强制关闭机器人输出
```

**验证点**:
- ✅ MCB权限标志清空
- ✅ 最终权限标志清空
- ✅ 机器人输出强制关闭 `[-]`
- ✅ 机械臂停止响应控制命令

---

### 场景4：R键独立控制（忽略MCB状态）

**前提**：初始状态或MCB已释放

**操作步骤**:
1. 在Policy控制器界面按 `R` 键

**预期UI显示**:
```
R键:[✓]  MCB:[ ]  最终权限:[✓]  机器人:[X]
[R键] 手动控制已启用
[R键] 已自动开启机器人输出（忽略MCB状态）
```

**验证点**:
- ✅ R键标志显示 `✓`
- ✅ MCB标志仍为空（独立工作）
- ✅ 最终权限标志显示 `✓`
- ✅ 机器人输出自动开启 `[X]`
- ✅ 可以通过键盘控制机械臂移动

**重要说明**：
> R键控制完全独立于MCB状态，即使MCB无授权，R键仍可独立控制机械臂。这是为了提供应急手动控制能力。

---

### 场景5：MCB释放忽略R键状态（安全优先）

**前提**：R键已启用，MCB也已授权，机器人正在运行

**初始UI显示**:
```
R键:[✓]  MCB:[✓]  最终权限:[✓]  机器人:[X]
```

**操作步骤**:
```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'test_003', object: 'arm', type: 'release'}"
```

**预期UI显示**:
```
R键:[✓]  MCB:[ ]  最终权限:[✓]  机器人:[-]
[MCB释放] MCB已释放控制权限，强制关闭机器人输出
```

**验证点**:
- ✅ R键标志仍为 `✓`（未改变）
- ✅ MCB标志清空
- ✅ 最终权限仍为 `✓`（因为R键仍启用）
- ⚠️ **机器人输出被强制关闭** `[-]`（安全优先）
- ✅ 机械臂停止响应控制命令

**重要说明**：
> MCB释放时会强制关闭机器人输出，即使R键仍启用。这是安全机制：MCB作为远程上位机，其释放操作拥有更高的安全优先级。

---

### 场景6：R键重启恢复控制

**前提**：场景5完成，R键启用但输出已关闭

**初始UI显示**:
```
R键:[✓]  MCB:[ ]  最终权限:[✓]  机器人:[-]
```

**操作步骤**:
1. 按 `R` 键禁用手动控制
   - 预期UI：`R键:[ ]  MCB:[ ]  最终权限:[ ]  机器人:[-]`

2. 再按 `R` 键重新启用手动控制
   - 预期UI：`R键:[✓]  MCB:[ ]  最终权限:[✓]  机器人:[X]`
   - 提示：`[R键] 手动控制已启用` + `已自动开启机器人输出（忽略MCB状态）`

**验证点**:
- ✅ R键重启成功
- ✅ 机器人输出恢复 `[X]`
- ✅ 可以通过键盘控制机械臂移动
- ✅ 不需要MCB重新授权

**重要说明**：
> R键重启（禁用→启用）可以独立恢复控制，无需等待MCB重新授权。

---

### 场景7：Z键手动急停

**前提**：机器人正在运行（任意权限模式）

**操作步骤**:
1. 在Policy控制器界面按 `Z` 键触发急停
2. 再次按 `Z` 键解除急停

**触发急停**:
```
UI显示：急停:[🚨]  机器人:[-]
提示：[Z键] 🚨 手动触发急停！所有控制已冻结
```

**观察现象**:
- ✅ 所有机器人输出立即停止
- ✅ 仿真输出也停止
- ✅ 位置锁定在当前位置
- ✅ 拒绝接受新的目标位置
- ✅ 如果MCB已授权，自动释放MCB权限

**解除急停**:
```
UI显示：急停:[ ]  机器人:[-]
提示：[Z键] ✓ 手动解除急停
```

**观察现象**:
- ✅ 急停标志清除
- ⚠️ 机器人输出仍为关闭状态 `[-]`
- ⚠️ 需要重新启用控制（按R键或请求MCB授权）

---

### 场景8：ROS2话题远程急停

**前提**：机器人正在运行

**触发急停**:
```bash
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'Y'}" --once
```

**预期效果**:
- 统一服务器：`[急停] 🚨 收到急停信号！所有控制已冻结！`
- Policy控制器UI：`急停:[🚨]  机器人:[-]`
- 与Z键效果相同

**解除急停**:
```bash
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'N'}" --once
```

**预期效果**:
- 统一服务器：`[急停] ✓ 急停解除，系统恢复正常`
- Policy控制器UI：`急停:[ ]  机器人:[-]`
- 需要重新启用控制

---

### 场景9：MCB双权限共存测试

**操作步骤**:
1. 按 `R` 键启用手动控制
   - UI：`R键:[✓]  MCB:[ ]  最终权限:[✓]  机器人:[X]`

2. MCB请求授权
   ```bash
   ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
     "{task_id: 'test_004', object: 'arm', type: 'get'}"
   ```
   - UI：`R键:[✓]  MCB:[✓]  最终权限:[✓]  机器人:[X]`

3. 按 `Z` 键触发急停
   - UI：`急停:[🚨]  R键:[✓]  MCB:[ ]  最终权限:[✓]  机器人:[-]`
   - 注意：MCB权限被自动释放

4. 按 `Z` 键解除急停
   - UI：`急停:[ ]  R键:[✓]  MCB:[ ]  最终权限:[✓]  机器人:[-]`

5. R键重启恢复控制
   - 按R禁用 → 按R启用
   - UI：`R键:[✓]  MCB:[ ]  最终权限:[✓]  机器人:[X]`

**验证点**:
- ✅ R键和MCB可以同时持有权限
- ✅ 急停触发时自动释放MCB权限
- ✅ 急停不影响R键状态
- ✅ 急停解除后需重新启用控制

---

### 场景10：Action模式测试（完整流程）

**前提**：统一服务器和Policy控制器已启动

**步骤1：MCB获取控制权限**
```bash
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'action_test', object: 'arm', type: 'get'}"
```

**步骤2：在Policy控制器按 `M` 键启用Action模式**
```
UI显示：控制模式:[ACTION]
提示：[Action] Action控制模式已启用
```

**步骤3：发送Action请求（需要单独的Action客户端）**
```bash
# 使用测试客户端（如果已编译）
ros2 run arm_controller arm_test_client
```

**步骤4：观察执行过程**
- 统一服务器：接收Action请求，验证权限
- 统一服务器：发布目标位置到 `/g1/left_arm/goal`, `/g1/right_arm/goal`
- Policy控制器：订阅目标，驱动机械臂移动
- Policy控制器：发布当前位置到 `/g1/left_arm/position`, `/g1/right_arm/position`
- 统一服务器：发送位置反馈到Action客户端

**步骤5：测试急停中断**
- 在Action执行过程中按 `Z` 键
- 统一服务器：`[Action执行] 🚨 急停触发，任务中止`
- Action返回：`result: 'failed', error: 'emergency stop triggered'`

**步骤6：恢复测试**
- 按 `Z` 键解除急停
- MCB重新请求权限
- 再次发送Action请求

---

## 监控与调试

### 实时监控话题

#### 监控位置反馈

```bash
# 监控左臂位置
ros2 topic echo /g1/left_arm/position

# 监控右臂位置
ros2 topic echo /g1/right_arm/position
```

#### 监控权限状态

```bash
# 监控MCB权限状态
ros2 topic echo /g1/arm/mcb_control_authority

# 监控急停状态
ros2 topic echo /g1/arm/emergency_stop
```

#### 监控目标位置

```bash
# 监控左臂目标（Action模式）
ros2 topic echo /g1/left_arm/goal

# 监控右臂目标（Action模式）
ros2 topic echo /g1/right_arm/goal
```

#### 监控急停话题

```bash
ros2 topic echo /msg_estop
```

### 查看服务列表

```bash
# 查看控制权限服务
ros2 service list | grep control_power

# 查看服务详情
ros2 service type /srv_control_power_transport
```

### 查看Action列表

```bash
# 查看所有Action
ros2 action list

# 查看Action详情
ros2 action info action_arm
```

### 日志文件

权限变更日志自动记录到：
```
/tmp/arm_control_authority.log
```

查看日志：
```bash
tail -f /tmp/arm_control_authority.log
```

---

## 故障排除

### 问题1：统一服务器启动失败

**错误信息**:
```
[错误] 无法导入接口: No module named 'robot_interfaces'
```

**解决方法**:
```bash
cd /home/york/RL_arm_control
colcon build --packages-select robot_interfaces --symlink-install
source install/setup.bash
```

---

### 问题2：Policy控制器无法连接ROS2

**错误信息**:
```
[robot] 缺少 ROS2 依赖: No module named 'rclpy'
```

**解决方法**:
```bash
# 确保已source ROS2环境
source /opt/ros/humble/setup.bash  # 或对应的ROS2版本
source install/setup.bash
```

---

### 问题3：MCB服务调用无响应

**现象**：服务调用一直等待，无响应

**排查步骤**:
```bash
# 1. 检查统一服务器是否运行
ros2 node list | grep unified_arm_control_server

# 2. 检查服务是否可用
ros2 service list | grep /srv_control_power_transport

# 3. 检查服务类型
ros2 service type /srv_control_power_transport
```

**解决方法**:
- 确认统一服务器正在运行
- 重启统一服务器

---

### 问题4：机器人输出无法开启

**现象**：MCB授权成功，但机器人输出仍为 `[-]`

**排查步骤**:
1. 检查是否处于急停状态：`急停:[🚨]`
2. 检查MCB权限状态：`MCB:[✓]`
3. 检查最终权限状态：`最终权限:[✓]`

**解决方法**:
- 如果急停：按 `Z` 键解除急停
- 如果MCB已授权但输出未开启：尝试R键重启
- 如果以上都正常：重启Policy控制器

---

### 问题5：急停无法解除

**现象**：按 `Z` 键或发送解除消息后，急停标志仍为 `🚨`

**排查步骤**:
```bash
# 检查急停话题状态
ros2 topic echo /msg_estop --once
```

**解决方法**:
```bash
# 强制发送解除急停消息
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'N'}" --once

# 如果仍无效，重启Policy控制器
```

---

### 问题6：Action请求被拒绝

**错误信息**:
```
[Action目标] 拒绝请求: no MCB control authority
```

**原因**：MCB未授权或急停状态

**解决方法**:
```bash
# 1. 检查急停状态，如果急停先解除
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'N'}" --once

# 2. MCB请求控制权限
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'action_fix', object: 'arm', type: 'get'}"

# 3. 重新发送Action请求
```

---

### 问题7：配置文件找不到

**错误信息**:
```
FileNotFoundError: [错误] 未找到动作点配置文件 actionpoints.yaml
```

**解决方法**:
```bash
# 1. 检查配置文件是否存在
ls /home/york/RL_arm_control/src/config/actionpoints.yaml

# 2. 如果不存在，从备份恢复或创建新的

# 3. 重新编译arm_controller
cd /home/york/RL_arm_control
colcon build --packages-select arm_controller --symlink-install
source install/setup.bash
```

---

### 问题8：控制权限转移时手臂突然运动（✅ 已修复 - v2方案）

**现象**：当MCB从Manipulation Module释放并重新授权给Policy Controller时，手臂突然高速运动

**根本原因**：

Policy Controller在MCB重新授权时，依赖仿真环境记忆的位置，但该位置可能与真实机器人实际位置不同步，导致突然运动。

**⚠️ v1方案问题（已废弃）**：

初始修复方案使用仿真环境记忆的位置（`env.get_left_palm_pos()`），但存在以下问题：
- 当仿真环境和真实机器人不同步时（如纯仿真模式移动），仍会导致突然运动
- 无法应对外力干扰、物理移动等情况
- 不符合工业机器人控制最佳实践

**✅ v2方案（当前版本 - 2026-02-23）**：

**核心思想**：从真实机器人读取关节状态，而非依赖仿真记忆

**实现步骤**：

1. **订阅真实机器人状态**：
```python
# 订阅 /lowstate 话题，获取真实机器人关节角度
self._lowstate_sub = self._node.create_subscription(
    LowState,
    '/lowstate',
    self._lowstate_callback,
    10
)
```

2. **存储真实关节角度**：
```python
def _lowstate_callback(self, msg):
    """从LowState消息提取关节角度"""
    with self._robot_state_lock:
        if hasattr(msg, 'motor_state') and len(msg.motor_state) > 28:
            for idx in range(15, 29):  # 15-28: 双臂关节
                self._real_robot_joint_angles[idx] = msg.motor_state[idx].q
```

3. **同步真实状态到仿真**：
```python
def sync_real_robot_state_to_sim(self, lock):
    """将真实机器人关节状态同步到仿真环境
    
    工作流程：
    1. 从 /lowstate 获取真实机器人关节角度
    2. 将角度更新到仿真环境的 qpos
    3. 执行 mj_forward 更新物理状态
    4. 使用仿真FK计算手掌位置
    
    Returns:
        (left_pos, right_pos): 从真实关节角度计算的手掌位置
    """
```

4. **MCB/R键授权时使用真实状态**：
```python
if not last_mcb_status and current_mcb_status:
    # 从真实机器人同步状态
    left_pos, right_pos = robot.sync_real_robot_state_to_sim(lock)
    
    if left_pos is not None:
        # 成功同步真实状态
        print("✓ 已从真实机器人同步关节角度到仿真")
    else:
        # 回退到仿真位置
        print("⚠ 无法同步真实状态，使用仿真位置")
        left_pos = env.get_left_palm_pos()
    
    # 更新目标位置
    left_goal[:] = left_pos
    env.set_left_goal(left_pos)
    
    # 开启机器人输出
    control_state['out_robot'] = robot.ok
```

**v2方案优势**：

| 特性 | v1方案（仿真记忆） | v2方案（真实状态同步） |
|------|------------------|---------------------|
| 数据源 | 仿真环境记忆 | 真实机器人关节角度（/lowstate） |
| 准确性 | ⚠️ 依赖仿真同步 | ✅ 直接读取真实状态 |
| 鲁棒性 | ⚠️ 纯仿真模式失效 | ✅ 任何情况可靠 |
| 外力干扰 | ❌ 无法应对 | ✅ 自动适应 |
| 工业标准 | 不符合 | ✅ 符合最佳实践 |

**验证修复**：

修复后，测试以下场景均安全：

**场景1：正常控制权转移**
1. Policy Controller控制手臂到Point A
2. MCB释放，Manipulation移动手臂到Point B
3. MCB重新授权给Policy Controller
4. **预期**：手臂保持在Point B不动 ✅

**场景2：纯仿真模式后恢复**
1. MCB释放，手臂在Point A
2. Policy Controller纯仿真模式（out_robot=False），用WASD移动仿真到Point B
3. R键或MCB授权恢复控制
4. **预期**：仿真从B跳回A，真实机器人保持在A不动 ✅

**场景3：外力干扰**
1. MCB释放，手臂在Point A
2. 人工物理移动手臂到Point C
3. MCB重新授权
4. **预期**：手臂保持在Point C不动 ✅

**技术参考**：

- [Unitree G1 开发者文档 - 基础服务接口](https://support.unitree.com/home/zh/G1_developer/basic_services_interface)
- LowState话题：`/lowstate` - 提供实时关节状态
- motor_state[15-28]：双臂14个关节的角度反馈

**✅ 当前版本已安全**：可以在生产环境使用控制权限转移功能。

---

## 键盘控制参考

### Policy控制器界面快捷键

| 按键 | 功能 | 说明 |
|------|------|------|
| **R** | 手动控制切换 | 独立控制，忽略MCB状态，自动开启/关闭机器人输出 |
| **Z** | 急停切换 | 手动触发/解除急停，拥有最高优先级 |
| **M** | Action模式切换 | 切换到Action Server控制模式 |
| **T** | 仿真输出切换 | 开启/关闭仿真显示 |
| **[** | 减慢速率 | 控制循环速率减半 |
| **]** | 加快速率 | 控制循环速率加倍 |
| **,** / **<** | 减小倍数 | 动作幅度减半 |
| **.** / **>** | 增大倍数 | 动作幅度加倍 |
| **Esc** / **X** | 退出程序 | 安全退出控制器 |

### 手动模式控制（非Action模式）

#### 左臂控制

| 按键 | 功能 | 方向 |
|------|------|------|
| **W** | Z轴+ | 上升 |
| **S** | Z轴- | 下降 |
| **A** | Y轴+ | 左移 |
| **D** | Y轴- | 右移 |
| **Q** | X轴+ | 前进 |
| **E** | X轴- | 后退 |

#### 右臂控制

| 按键 | 功能 | 方向 |
|------|------|------|
| **I** | Z轴+ | 上升 |
| **K** | Z轴- | 下降 |
| **J** | Y轴+ | 左移 |
| **L** | Y轴- | 右移 |
| **U** | X轴+ | 前进 |
| **O** | X轴- | 后退 |

---

## 安全注意事项

### ✅ 控制权限转移安全性（已修复）

**问题描述（已解决）**：

早期版本在控制权限在Policy Controller和Manipulation Module之间转移时，存在安全隐患：当MCB重新授权给Policy Controller时，Policy Controller会尝试将手臂从当前位置（Point B）拉回上次目标（Point A），导致突然的高速运动。

**✅ 已实施修复（日期：2026-02-23）**：

当前版本已在 `g1_dual_arm_policy_controller_ROS2_v3.py` 中实施以下修复：

1. **MCB授权事件**：在启用机器人输出前，自动同步当前手臂位置到目标位置
2. **R键启用事件**：在启用机器人输出前，自动同步当前手臂位置到目标位置

这些修复确保了在控制权限转移时，手臂不会突然移动到旧目标位置。

**验证步骤**：

1. Policy Controller控制手臂到Point A
2. MCB释放，Manipulation移动手臂到Point B
3. MCB重新授权给Policy Controller
4. **预期**：手臂保持在Point B不动（或缓慢微调）
5. **不应该**：手臂突然快速移动

**✅ 当前版本已安全**：可以在生产环境使用MCB控制权限功能。

---

### ⚠️ 操作安全

1. **急停优先原则**
   - 遇到任何异常情况，立即按 `Z` 键触发急停
   - 急停后检查确认安全后再解除

2. **权限确认**
   - 操作前确认UI显示的权限状态
   - 确保只有授权人员拥有控制权限

3. **工作空间检查**
   - 确保机械臂工作空间内无人员和障碍物
   - 注意左右臂可能发生碰撞

4. **MCB释放响应**
   - MCB释放时立即停止所有操作
   - 不要尝试绕过MCB安全机制

### ⚠️ 系统维护

1. **日志监控**
   - 定期检查 `/tmp/arm_control_authority.log`
   - 关注异常错误信息

2. **环境同步**
   - 每次编译后重新 `source install/setup.bash`
   - 确保所有终端使用相同的环境

3. **版本管理**
   - 记录系统版本和配置
   - 重大修改前备份配置文件

---

## 附录

### A. 完整测试验证序列

推荐按以下顺序完成所有测试场景，确保系统功能正常：

```
1. 启动系统（场景1）
   ↓
2. MCB授权测试（场景2）
   ↓
3. MCB释放测试（场景3）
   ↓
4. R键独立控制测试（场景4）
   ↓
5. MCB释放忽略R键测试（场景5）
   ↓
6. R键重启测试（场景6）
   ↓
7. Z键急停测试（场景7）
   ↓
8. ROS2话题急停测试（场景8）
   ↓
9. MCB双权限共存测试（场景9）
   ↓
10. Action模式完整流程测试（场景10）
```

### B. 快速参考卡片

#### 启动命令速查

```bash
# 终端1：统一服务器
cd /home/york/RL_arm_control && source install/setup.bash
ros2 run arm_controller arm_control_action_authority_server_compiled

# 终端2：Policy控制器
cd /home/york/RL_arm_control && source install/setup.bash
source src/pkg/unitree_ros2/setup.sh  # 实机控制必需
python3 src/arm_controller/arm_controller/g1_dual_arm_policy_controller_ROS2_v3.py
```

#### MCB控制速查

```bash
# 获取权限
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'quick_test', object: 'arm', type: 'get'}"

# 释放权限
ros2 service call /srv_control_power_transport robot_interfaces/srv/ControlPower \
  "{task_id: 'quick_test', object: 'arm', type: 'release'}"
```

#### 急停控制速查

```bash
# 触发急停
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'Y'}" --once

# 解除急停
ros2 topic pub /msg_estop robot_interfaces/msg/Estop "{emergency_stop: 'N'}" --once
```

### C. 优先级总结

```
系统安全优先级（从高到低）：

1. 急停（最高）
   - Z键本地触发
   - ROS2话题远程触发
   - 立即冻结所有控制
   - 自动释放MCB权限

2. MCB释放
   - 强制关闭机器人输出
   - 忽略R键状态
   - 安全优先原则

3. R键控制
   - 本地手动控制
   - 可独立工作
   - 应急控制能力

4. MCB授权
   - 远程控制授权
   - 与R键权限等级相同
   - 正常工作模式
```

---

## 文档修订历史

| 版本 | 日期 | 修订内容 | 作者 |
|------|------|---------|------|
| 2.3 | 2026-02-23 | **重大安全升级**：实施v2方案，使用真实机器人关节状态同步（/lowstate），替代仿真记忆方案，彻底解决所有突然运动问题 | York |
| 2.2 | 2026-02-23 | **安全修复**：实施了控制权限转移位置同步修复，解决了手臂突然运动问题（问题8） | York |
| 2.1 | 2026-02-23 | **重要安全更新**：添加控制权限转移风险警告和代码修复建议（问题8） | York |
| 2.0 | 2026-02-23 | 更新为统一服务器架构，合并authority_server和action_server | York |
| 1.1 | 2026-02-04 | 增加R键本地控制逻辑说明 | York |
| 1.0 | 2026-02-04 | 初始版本，分离的3终端架构 | York |

---

