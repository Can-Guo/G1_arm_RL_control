# arm_controller 源码驱动技术文档

> 本文档基于 `arm_controller_backup_20260805` 目录下全部源代码与工程文档编写。

>作者：York GUO，邮箱：1739950387@qq.com


## 1. 项目概述

本项目实现 Unitree G1 人形机器人的**双臂到达任务（实际面向于搬运快递箱作业）**：给定左右手的目标三维笛卡尔坐标，通过强化学习策略把双臂运动到目标位置，并保持"拍手姿势"（掌心朝身体侧方、手掌平面垂直于地面）的姿态约束。系统打通了"训练 → 策略部署 → ROS2 控制 → 机器人 DDS 下控 → MCB 上位机交互"的完整链路。

系统分四层，各层物理上运行于不同节点：

| 层 | 职责 | 关键文件 |
|---|---|---|
| 训练层 | MuJoCo 仿真环境 + PPO/SAC/TD3 训练 + 模型导出 | `g1_arm_rl_env_IhKP.py`、`train_g1_arm_policy_N8hM.py` |
| 策略层 | 加载训练模型，把目标坐标映射为关节增量动作，含真实状态同步与镜像处理 | `g1_dual_arm_policy_controller_ROS2_v3.py` |
| 控制层 | ROS2 Action Server（目标校验、超时、权限门控、急停） | `arm_control_action_authority_server_compiled.py` |
| 协议/交互层 | MCB（上位机）与控制器之间的 ROS2 接口与预设动作点 | `robot_interfaces`、`Arm_control_protocol_v1.0.md`、`action_points_config.py` |

### 1.1 技术栈

- **仿真**：MuJoCo（`mujoco>=3.0.0`），G1 双臂模型 `g1_29dof_with_hand_dual.xml`（`setup.py` 中依赖声明；XML 见 `unitree_mujoco/unitree_robots/g1/`）。
- **强化学习**：Stable-Baselines3 `2.7.0`（PPO/SAC/TD3），`DummyVecEnv` 向量化，Gymnasium `1.2.0`。
- **机器人通信**：ROS2 Humble + `rclpy`，自定义接口包 `robot_interfaces`（`package.xml`），Unitree DDS（CycloneDDS，通过 `unitree-sdk2py` 与 `unitree_ros2` 工作空间桥接）。
- **环境**：conda `g1_vibe`，Python 3.10，CUDA 12.8（见 `requirements.txt`）。

### 1.2 部署版本甄别（实际生效的代码）

备份目录同时存在多份架构演进产物。经交叉核对 **运行工具链、包元数据、交互文档与 `__pycache__` 编译产物**，判定实际部署版本如下：

- **统一服务器（生效）**：`RL_arm_controler_V4.1_Updated_Feb23_One_server_Fix_sudden_move_with_Estop&_Authority/arm_controller/arm_controller/arm_control_action_authority_server_compiled.py`。依据：
  1. 目录名 `One_server_Fix_sudden_move_with_Estop&_Authority` 明确指向"单服务器 + 突发移动修复 + 急停/权限"；`setup.py` 中 `console_scripts` 的 `arm_action_authority_server` 映射到该文件。
  2. 其 `__pycache__` 中存在 `arm_control_action_authority_server_compiled.cpython-310.pyc` 编译产物（实际被 import 运行过）。
  3. `control_power_&_emergency_updated_20260223.md`（v2.0 手册）描述 2 终端架构，与统一服务器的实现一致。
- **策略控制器（生效）**：`g1_dual_arm_policy_controller_ROS2_v3.py`。依据：
  1. `setup.py` 中 `dual_arm_controller` 入口指向该文件。
  2. `__pycache__` 中存在其编译产物。
  3. launch 文件与 `start_dual_arm_controller.sh` 均启动该脚本。
- **训练环境（生效）**：`g1_arm_rl_env_IhKP.py`（"拍手约束"版本）。依据：`train_g1_arm_policy_N8hM.py:55` 显式 `from g1_arm_rl_env_IhKP import make_env`；根目录 `__pycache__` 存在其编译产物。`g1_arm_rl_env.py` 为无姿态约束的早期版本（其 `DISTANCE_THRESHOLD` 常量仍被包级导入复用）。
- **废弃（3 终端架构遗留）**：`arm_control_authority_server.py`（仅权限服务）、`arm_control_action_server_compiled.py`（独立 Action 服务器）。统一服务器已合并二者，`SUNNY_README_latest.md`（v3.1）描述的三终端架构已过时。

> 注：根目录 `g1_arm_policy_controller.py` 与部署包内同名文件**内容不同**（`diff` 证实）：根目录版默认 `--iface enx00e04c680266`、默认左臂 5M 模型（`g1_arm_policy_controller.py:516`）；部署包版默认 `--iface ens37`、默认右臂 5M 模型。两者均为**单臂验证控制器**，与生效的双臂 ROS2 控制器不是同一份代码。

---

## 2. 系统架构

### 2.1 模块一览

| 模块 | 文件 | 职责要点 |
|---|---|---|
| G1ArmReachEnv | `g1_arm_rl_env.py` | 训练环境（早期版）：24 维观测、±0.05 rad 增量动作、距离奖励、碰撞（默认关闭）。提供 `DISTANCE_THRESHOLD=0.03` |
| G1ArmReachEnv (IhKP) | `g1_arm_rl_env_IhKP.py` | 训练环境（生效版）：叠加"拍手姿势"姿态约束、躯干邻近惩罚、丰富 TensorBoard info |
| make_env | `g1_arm_rl_env_IhKP.py:818` | 环境工厂（`right_arm` 参数控制左右臂训练） |
| train_g1_arm_policy_N8hM | `train_g1_arm_policy_N8hM.py` | 训练入口：PPO/SAC/TD3、分段保存 checkpoint、`-rArm`/`-lArm` 运行命名 |
| DualArmEnv | `g1_dual_arm_policy_controller_ROS2_v3.py:100+` | 部署侧仿真环境：双 MuJoCo XML、碰撞检测、rest 姿态、obs 组装 |
| RobotBridgeROS2 | `g1_dual_arm_policy_controller_ROS2_v3.py:474+` | 机器人桥接：`/arm_sdk` LowCmd 发布、`/lowstate` 订阅、R 键/MCB 权限、急停、真实状态同步 |
| unified_arm_control_server | `arm_control_action_authority_server_compiled.py` | 统一服务器：ActionServer `action_arm`、`/srv_control_power_transport` 服务、`/msg_estop` 急停、目标校验、10s 超时、10Hz 状态发布 |
| ActionPointsConfig | `action_points_config.py` | actionpoints.yaml 预设动作点加载（多路径搜索） |
| test_arm_client_simple | `test_arm_client_simple.py` | Action 客户端测试工具（测试记录/报告） |
| arm_controller 包 | `__init__.py` | 导出 `DISTANCE_THRESHOLD`、`get_config_path`/`get_model_path`/`get_mujoco_xml_path` 路径解析 |

### 2.2 架构总览（ASCII）

```
┌───────────────────────── 训练层（开发机） ─────────────────────────┐
│  g1_arm_rl_env_IhKP.py  ◄───  train_g1_arm_policy_N8hM.py          │
│  MuJoCo 仿真(24维obs / 7维±0.05rad)   PPO/SAC/TD3 → *.zip 模型       │
└────────────────────────────────┬───────────────────────────────────┘
                                 │ 模型文件 (models/*.zip)
┌────────────────────────────────▼───────────────────────────────────┐
│                   策略层（G1 主控，ROS2）                              │
│  g1_dual_arm_policy_controller_ROS2_v3.py                          │
│  ├─ DualArmEnv 部署仿真(实时同步) ──── 目标订阅 /g1/*/goal            │
│  ├─ policy.predict → Δq ──────────── 镜像+clip±0.05 → LowCmd         │
│  └─ RobotBridgeROS2: /arm_sdk发布 /lowstate订阅 R键/MCB权限 急停     │
└───────────────┬──────────────────────────────┬──────────────────────┘
                │ (a) /g1/left_arm/goal         │ (b) LowCmd (DDS, 50Hz)
                │ (c) /g1/right_arm/goal        │ (d) /lowstate (关节角)
┌───────────────▼──────────────────────────────▼──────────────────────┐
│                控制层（统一服务器，ROS2）                                │
│  arm_control_action_authority_server_compiled.py                   │
│  ├─ ActionServer 'action_arm'  ── 目标校验/10s超时/动态阈值             │
│  ├─ /srv_control_power_transport (ControlPower) 权限门控              │
│  └─ /msg_estop (Estop) 急停 ── 最高优先级                              │
└───────────────────────────────┬────────────────────────────────────┘
                                │ ROS2 (DDS)
┌───────────────────────────────▼────────────────────────────────────┐
│             交互层（MCB 上位机） / 协议 Arm_control_protocol_v1.0.md    │
│             目标坐标 / 预设动作点 / 急停 'Y'/'N' / 权限授权              │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.3 目录树（部署包结构）

```
arm_controller_backup_20260805/
├── README_202608.md                          # 项目总览（四层架构、部署指引、注意点）
├── g1_arm_rl_env.py                          # 训练环境（早期版，常量被包级复用）
├── g1_arm_rl_env_IhKP.py                     # 训练环境（拍手约束版，train 实际导入）
├── train_g1_arm_policy_N8hM.py               # 训练脚本（PPO/SAC/TD3，分段保存）
├── g1_arm_policy_controller.py               # 单臂验证控制器（根目录版，iface=enx...266）
├── g1_dual_arm_policy_controller.py          # 双臂验证控制器（SDK2 直连版）
├── arm_control_authority_server.py           # 旧版权限服务器（3 终端架构，废弃）
├── arm_control_action_server_compiled.py     # 旧版 Action 服务器（3 终端架构，废弃）
├── Arm_control_protocol_v1.0.md              # MCB 通信协议定义
├── control_power_&_emergency_updated_20260223.md  # v2.0 手册（2 终端架构，当前生效）
├── SUNNY_README_latest.md                    # v3.1 手册（3 终端架构，已过时）
├── 姿态约束详解_拍手姿势.md                    # 姿态约束演进说明
├── models/                                   # 训练产物（根目录，含历史模型）
├── backup_script/                            # 2025-09/10 历史备份脚本（废弃）
├── unitree_mujoco/                           # MuJoCo 模型资源（XML + 网格）
└── RL_arm_controler_V4.1_Updated_Feb23_One_server_Fix_sudden_move_with_Estop&_Authority/
    └── arm_controller/                       # ◄ 实际部署包（colcon build 对象）
        ├── package.xml                       # ament_python 元数据 / 依赖 robot_interfaces
        ├── setup.py                          # console_scripts 入口 / data_files 模型与XML
        ├── setup.cfg
        ├── requirements.txt                  # conda 环境：py3.10/mujoco3.3.5/sb3 2.7.0
        ├── launch/
        │   ├── start_dual_arm_controller.sh  # 启动脚本（Humble+conda g1_vibe+unitree DDS）
        │   └── dual_arm_control.launch.py    # launch 文件
        ├── models/                           # 部署模型（左5M/左10M/右5M，均存在）
        ├── unitree_mujoco/                   # 部署用 XML（g1_29dof_with_hand_dual.xml）
        ├── config/                           # actionpoints.yaml（可选，多路径搜索）
        └── arm_controller/                   # 包源码
            ├── __init__.py                   # DISTANCE_THRESHOLD / 路径解析
            ├── g1_arm_rl_env.py              # 部署包内环境（路径解析与根目录版略有差异）
            ├── g1_arm_policy_controller.py   # 单臂控制器（部署包版，iface=ens37）
            ├── g1_dual_arm_policy_controller_ROS2_v3.py  # ◄ 生效：双臂策略控制器
            ├── arm_control_action_authority_server_compiled.py  # ◄ 生效：统一服务器
            ├── arm_control_action_server_compiled.py  # 旧版 Action 服务器
            ├── arm_control_authority_server.py        # 旧版权限服务器
            ├── test_arm_client_simple.py     # Action 测试客户端
            ├── action_points_config.py       # 预设动作点加载器
            ├── dual_arm_controller_wrapper.py # 空文件（无内容，未使用）
            └── diagnose_authority.cpython-310.pyc  # 仅有编译产物，无对应 .py 源码
```

---

## 3. 数据流与通信拓扑

### 3.1 端到端数据流

```
MCB 上位机
  │  goal 请求 (target/action/x_l,y_l,z_l,x_r,y_r,z_r/high_resolution)   [ROS2 Action 'action_arm']
  ▼
统一服务器 arm_control_action_authority_server_compiled.py
  │  _validate_goal() 范围校验 (x∈[0,0.6], left y∈[0,0.6], right y∈[-0.6,0], z∈[0.55,1.45])  [L734-772]
  │  动态阈值 = DISTANCE_THRESHOLD × max(1, high_resolution)  [L575-580]
  │  10s 超时 / 10Hz feedback  [L592, L596]
  ▼  publish  [L173-178]
/g1/left_arm/goal  /g1/right_arm/goal   (Float32MultiArray [x,y,z])
  ▼
策略控制器 g1_dual_arm_policy_controller_ROS2_v3.py
  │  DualArmEnv：obs = concat[q(7), dq(7), p_hand(3), p_goal(3), Δp(3), step_frac(1)] → 24 维  [L303-310]
  │  policy.predict(obs, deterministic=True) → Δq(7)，clip ±0.05，speed.mult 缩放  [L1493]
  │  镜像关节：left_wrist_roll/yaw = -right_wrist_roll/yaw  [L1029-1042]
  │  碰撞检测（penetration ≥ 0.005）→ 冻结/停止  [L446-447]
  ▼  send_qpos() @ 50Hz
/arm_sdk  LowCmd（kp=50.0, kd=1.5, mode, qpos, dq）  [L551-552]
  ▼  Unitree DDS
G1 真实机器人
  ▲  关节状态回读
/lowstate  (motor_idx 15-28 → 双臂 14 关节)  [L740-741]
  │  sync_real_robot_state_to_sim()：授权瞬间把真实关节角写入 MuJoCo qpos，防突发移动  [L839-924]
  ▲
策略控制器 (RobotBridgeROS2)
```

### 3.2 权限与急停数据流（控制层）

```
权限判定（RobotBridgeROS2._has_control_authority）
  急停 (Emergency Stop)  ──────────────────── 最高优先级，一旦触发无条件冻结
       │
       ▼ 否则
  R键本地手动  OR  MCB 授权（/srv_control_power_transport）
       │
       ▼
  具有控制权限 → 执行动作 / 否则仅保持当前姿态
```

优先级链（`control_power_&_emergency_updated_20260223.md` 与服务器代码一致）：**急停 > MCB 释放 > R 键 > MCB 授权**。

- 急停触发（`/msg_estop` 收到 `Estop='Y'`）：服务器自动释放 MCB authority、调用 `stop_at_current_position()`、abort 当前 goal（统一服务器内部逻辑；策略控制器同步冻结）。
- 授权瞬间的**真实状态同步**：从 `/lowstate` 读取 `motor_idx 15-28` 的实时关节角度，写入 MuJoCo `qpos`（`g1_dual_arm_policy_controller_ROS2_v3.py:839-924`，映射见 L884-894），避免控制权从 MCB 转移到本地时策略观测状态与真实姿态脱节导致跳变。

### 3.3 动作时序图

```
MCB          统一服务器            策略控制器             G1
 │  Action_Goal │                    │                   │
 ├─────────────►│ _validate_goal()   │                   │
 │              │ 校验通过           │                   │
 │  Feedback ◄──┤ 目标发布 ─────────►│ 接收目标            │
 │    (10Hz)    │  /g1/*/goal        │ obs→predict→Δq     │
 │              │                    │ LowCmd ──────────►│
 │              │                    │  <低状态/位置回传>   │
 │              │◄───────────────────│ /lowstate          │
 │              │ check_reached()    │                    │
 │  结果 ◄──────┤ dist < 动态阈值    │                    │
 │   Result     │ (成功/失败/超时)    │                    │
 └──────────────┴────────────────────┴────────────────────┘
```

### 3.4 数据流断点（需注意的工程差异）

| 断点 | 证据 | 影响 |
|---|---|---|
| 测试客户端默认 action 名为 `/action_arm`，服务器注册名为 `action_arm` | `test_arm_client_simple.py`（默认 `ARM_CONTROL_ACTION_NAME=/action_arm`）vs 服务器 `action_arm`（服务器 L307） | 若客户端未显式指定 action 名，ROS2 会因名字不匹配找不到 Action Server |
| 文档超时值不一致 | 旧文档/测试客户端描述 30s；统一服务器代码 `timeout = 10.0`（L596）；`README_202608.md` L536 亦明确"以最新代码行为准" | 依赖旧文档的集成方会误判超时行为 |
| 训练默认关闭碰撞 | `g1_arm_rl_env_IhKP.py:55` `_DISABLE_COLLISION: bool = True`；`g1_arm_rl_env.py:56` 同 | 训练出的策略在真实机器人上不保证无碰撞表现；部署控制器仍保留碰撞检测冻结逻辑 |
| launch 文件启动旧版服务器 | `dual_arm_control.launch.py` 内 `ros2 run arm_controller arm_action_server_compiled` 指向旧版独立 Action 服务器，而非统一服务器 | 若按 launch 一键启动，跑的是旧 3 终端架构；统一服务器需手动 `ros2 run arm_controller arm_action_authority_server` |
| `--sim-only` 默认值为字符串 | `g1_arm_policy_controller.py` 中 `default="true"`（字符串而非布尔） | 影响单臂验证控制器的仿真/实机切换判断，按真值语义处理时需注意 |
| `dual_arm_controller_wrapper.py` 为空 | 文件存在但无内容 | 不会被 import，属占位文件 |
| `diagnose_authority` 有 .pyc 无源码 | 部署包 `__pycache__` 中存在编译产物，源码缺失 | 曾用于诊断权限问题，已不可直接运行/修改 |

---

## 4. 核心算法原理

### 4.1 问题定义与观测空间（24 维）

每臂独立的到达任务被建模为 MDP。单臂观测为 24 维向量（`g1_arm_rl_env_IhKP.py:358-364`、部署控制器 `g1_dual_arm_policy_controller_ROS2_v3.py:303-310`）：

```
obs = concat[
  q(7),        # 7 个关节角, [-π, π]
  dq(7),       # 7 个关节角速度, [-10, 10] rad/s
  p_hand(3),   # 手掌位置 (x,y,z), [-1.5, 1.5] m
  p_goal(3),   # 目标位置, [-1.5, 1.5] m
  Δp(3),       # 目标-手掌 差值
  step_frac(1) # 当前步数/最大步数 ∈ [0,1]
]
```

动作空间：`Box(-0.05, 0.05, shape=(7,))`（`g1_arm_rl_env_IhKP.py:355`），即每步给 7 个关节一个**增量角度**，经 `qpos[adr] = clip(qpos[adr] + dq, -3.14, 3.14)` 积分（L544-548）。增量式设计让策略输出天然平滑、幅度有界，利于真实机器人安全执行。

成功判据：`dist = ‖p_goal - p_hand‖ < 0.03`（`g1_arm_rl_env_IhKP.py:711`），即 3cm 内即视为到达。

### 4.2 奖励设计

基础稀疏-稠密混合奖励（`g1_arm_rl_env_IhKP.py:554`）：

```python
reward = -5.0 * dist - 0.1 * np.linalg.norm(action)
```

- `-5.0 * dist`：主项，距离越大惩罚越大，引导策略靠近目标；
- `-0.1 * ‖action‖`：动作正则，抑制抖动，鼓励最小能耗；
- 成功奖励：`success = dist < 0.03 and not collision_hit` 时 `reward += 1.0`（L711-714）。

**拍手姿势约束**（该版本的核心姿态改进，取代早期"垂直抓握"约束。详见 `姿态约束详解_拍手姿势.md`）：

```python
# 掌心法向量取旋转矩阵第 1 列（诊断确认：列1才是掌心法向，主指横向Y轴）
palm_normal = ori_mat[:, 1]            # L605

# 左臂掌心应指向右侧(-Y)，右臂掌心应指向左侧(+Y)
lateral_alignment = palm_normal[1] if not right_arm else -palm_normal[1]   # L615-618
lateral_error = 1.0 - lateral_alignment
reward -= 2.0 * (lateral_error ** 2)   # L627 强权重，平方损失加强完美对齐

# 手掌平面垂直地面：|palm_normal[2]| → 0
vertical_deviation = abs(float(palm_normal[2]))
reward -= 1.5 * vertical_deviation     # L633

# 手指方向自然性：抓握轴大致水平
grasp_axis_world = ori_mat[:, 0]
grasp_vertical_component = abs(float(grasp_axis_world[2]))
reward -= 0.5 * grasp_vertical_component  # L641
```

**碰撞惩罚**（默认 `_DISABLE_COLLISION=True` 时关闭，代码路径保留）：

```python
if not self._DISABLE_COLLISION:
    for i in range(self.data.ncon):
        c = self.data.contact[i]
        g1, g2 = c.geom1, c.geom2
        if (g1 in self._arm_gids and g2 in self._protect_gids) or (
            g2 in self._arm_gids and g1 in self._protect_gids):
            penetration = max(0.0, -c.dist)
            if penetration < 0.002:
                continue
            reward -= 200.0 + 2000.0 * penetration   # 固定成本 + 穿透深度缩放
            collision_hit = True
            break
```

**躯干邻近惩罚**（防止手臂贴近躯干中心）：

```python
d_torso = np.linalg.norm(p_hand - torso_pos)        # torso_pos = [0,0,0.90] (L459)
if d_torso < 0.10:
    reward -= (0.10 - d_torso) * 100.0
```

### 4.3 目标采样（训练时）

目标采样策略（`g1_arm_rl_env_IhKP.py:441-521`）：
- 60% 正面目标：`y ∈ [-0.45,-0.05]`（右臂）/ `[0.05,0.45]`（左臂），正对躯干前方；
- `reach_max = 0.60` m 可达半径，肩部到目标距离不超过该值；
- 与躯干中心保持 ≥ 0.10 m 距离。

### 4.4 训练流程与分段保存

`train_g1_arm_policy_N8hM.py`：
- 算法可选 PPO/SAC/TD3（`ALGOS`，默认 `ppo`，L106-116）；
- `--num-envs 16` DummyVecEnv 并行、`--total-steps` 默认 50M、`--checkpoint 250_000` 分段保存（L120）；
- 命名：`run_name = YYYYMMDD-HHMMSS + "-rArm"/"-lArm"`，TensorBoard 日志在 `runs/g1_arm_reach/`（L154-157）；
- 支持 `--resume-from` 从 checkpoint 续训（L170）；
- `DetailedMetricsCallback` 将 info 字典中 `task/`、`pose/`、`reward/`、`hand/`、`goal/`、`error/`、`joints/`、`action/` 分组指标写入 TensorBoard。

### 4.5 部署策略的"防突发移动"机制

控制权转移瞬间的同步是 v2 修复的核心（`control_power_&_emergency_updated_20260223.md` 专题说明）。实现位于 `g1_dual_arm_policy_controller_ROS2_v3.py:839-924`：

```python
def sync_real_robot_state_to_sim(self, lock):
    # 1. 从 /lowstate 读真实关节角（motor_idx 15-28）
    # 2. 若超过 1.0s 未收到 → 打印同步警告，跳过本次同步（L866-867）
    # 3. 依据 motor_idx→joint_name 映射写回 MuJoCo qpos (L884-903)
    # 4. 由 qpos 前向计算手掌位置，作为后续策略的观测起点（L910-924）
```

**关节索引与镜像**（关键约定）：

| 含义 | 索引 | 备注 |
|---|---|---|
| 左臂 7 关节 | 15-21 | `left_shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw` |
| 右臂 7 关节 | 22-28 | 镜像布局 |
| 手部抓手 | 29 | 部署时 `motor_cmd[29].q = 1.0`（合拢） |
| 镜像关节 | 19↔26, 21↔28 | `left_wrist_roll = -right_wrist_roll`，`left_wrist_yaw = -right_wrist_yaw`（L1029-1042） |

镜像的意义：左右臂在 XML 中按对称布局建模，但 G1 实机关节方向相反；直接下发右臂策略输出的左臂增量会导致方向性错误。发送前对第 5、7 号关节取反（L1036-1042），并把被镜像的关节同步回仿真（L1058-1063）。

### 4.6 控制器运行主循环

`run_curses` 主循环（约 L1162-1740）：
- **手动模式**：WASDQE 控制左臂、IJKLUO 控制右臂，每键 `STEP=0.015` m 步进（L1162），目标位置 clip 到合法工作空间；
- **Action 模式**：接收 `/g1/*/goal` 目标；
- 每步 `policy.predict(obs, deterministic=True)`（L1493），动作 `× speed.mult`（默认 0.5）后 clip ±0.05；
- **hold 模式**：当 `dist < dynamic_threshold` 时停止输出，等待重新远距离再 arm（`elif left_hold and left_dist >= DISTANCE_THRESHOLD`，L1482；右臂 L1515）；
- 位置发布 10Hz、机器人下发 50Hz（`speed.dt = max(0.01, args.rate)`，L1746）。

---

## 5. 通信协议

### 5.1 ROS2 接口总览

| 接口 | 类型 | 服务/话题/Action | 说明 |
|---|---|---|---|
| `action_arm` | `robot_interfaces.action.ArmControl` | Action | 双臂目标控制（`arm_control_action_authority_server_compiled.py:307`） |
| `/srv_control_power_transport` | `robot_interfaces.srv.ControlPower` | Service | MCB 控制权限请求/释放 |
| `/msg_estop` | `robot_interfaces.msg.Estop` | Topic | 急停信号（`'Y'`/`'N'`） |
| `/g1/left_arm/goal` | `std_msgs/Float32MultiArray` | Topic | 左臂目标坐标 [x,y,z] |
| `/g1/right_arm/goal` | `std_msgs/Float32MultiArray` | Topic | 右臂目标坐标 [x,y,z] |
| `/lowstate` | `unitree_api.msg.LowState` | Topic | 真实关节状态（idx 15-28） |
| `/arm_sdk` | `unitree_api.msg.LowCmd` | Topic | 关节下控指令（kp=50, kd=1.5） |
| 状态发布 | `std_msgs/Bool` | Topic | 10Hz 控制状态广播 |

### 5.2 ArmControl Action 协议（`Arm_control_protocol_v1.0.md`）

**Goal**：

| 字段 | 类型 | 含义 |
|---|---|---|
| `target` | string | `empty_box` / `product_box` / 空 |
| `action` | string | `pick` / `place` / `move` / 空 |
| `x_l, y_l, z_l` | float | 左臂目标坐标 |
| `x_r, y_r, z_r` | float | 右臂目标坐标 |
| `high_resolution` | int32 | 精度模式（见下） |

**Feedback**：6 个 float（左右手当前位置 x,y,z）。

**Result**：`result`（success/failed）+ `error` 消息 + 6 个 float 终点坐标。

### 5.3 目标校验与动态精度（`arm_control_action_authority_server_compiled.py:734-772, 575-580`）

```python
# _validate_goal 校验范围（基于 G1 工作空间）
left:  x ∈ [0.0, 0.6],  y ∈ [0.0, 0.6],  z ∈ [0.55, 1.45]
right: x ∈ [0.0, 0.6],  y ∈ [-0.6, 0.0], z ∈ [0.55, 1.45]

# 动态距离阈值
base_threshold = DISTANCE_THRESHOLD          # 0.03 m
multiplier = max(1, goal.high_resolution)    # 1/2/3 → 0.03/0.06/0.09 m
```

`high_resolution` 越高（≥1 的整数）代表**越宽松**的到达判定（`multiplier` 直接放大阈值），用于 MCB 上位机按场景调节灵敏度。非法 target/action/high_resolution 或越界坐标在 `_validate_goal` 中被拒绝并返回 error。

### 5.4 权限协议

`ControlPower` 服务的权限转移 + 本地 R 键构成控制权来源，逻辑为"或"（`RobotBridgeROS2` 打印：`控制权限 = R键使能 OR MCB授权`，`[ROS2] 新架构` 注释）。急停优先级最高，触发即冻结、释放 MCB、abort 当前 goal。权限状态可通过 10Hz Bool 话题广播。

### 5.5 预设动作点（`action_points_config.py`）

`ActionPointsConfig` 从 `actionpoints.yaml` 加载预设点（如 `arm_home`、`pick_empty_box`、`pick_product_box`），`get_position()` 返回 `((x_l,y_l,z_l),(x_r,y_r,z_r))`。配置文件搜索路径（L52-76）包括 MCB 标准路径 `~/sunny/hr_ros2_ws/src/config/`、`/home/york/RL_arm_control/src/config/` 与多级相对路径；找不到则抛 `FileNotFoundError`。

---

## 6. 工程实现细节

### 6.1 硬编码常量汇总

| 常量 | 值 | 位置 |
|---|---|---|
| `DISTANCE_THRESHOLD` | 0.03 m | `g1_arm_rl_env.py:39`；部署控制器 `g1_dual_arm_policy_controller_ROS2_v3.py:46`；服务器兜底 `arm_control_action_authority_server_compiled.py:60-64` |
| 动作幅度 | ±0.05 rad | `g1_arm_rl_env_IhKP.py:355`；`g1_arm_rl_env.py:366` |
| 观测维度 | 24 | 各环境 `observation_space` |
| 训练最大步数 | 200 | `g1_arm_rl_env_IhKP.py:380` |
| 部署最大步数 | 500 | `g1_dual_arm_policy_controller_ROS2_v3.py:123` |
| 成功距离 | < 0.03 m | `g1_arm_rl_env_IhKP.py:711` |
| 碰撞穿透阈值 | 0.005 m | `g1_dual_arm_policy_controller_ROS2_v3.py:446,460` |
| 碰撞惩罚系数 | -200 / -2000 | `g1_arm_rl_env_IhKP.py:663-665` |
| 拍手约束权重 | 2.0 / 1.5 / 0.5 | `g1_arm_rl_env_IhKP.py:627,633,641` |
| 躯干邻近惩罚系数 | 100.0 | `g1_arm_rl_env_IhKP.py:789-791` |
| 可达半径 `reach_max` | 0.60 m | `g1_arm_rl_env_IhKP.py:488` |
| 躯干中心 | (0, 0, 0.90) | `g1_arm_rl_env_IhKP.py:459` |
| 手掌 FK 偏移 | (0.0415, ±0.003, 0) | `g1_arm_rl_env_IhKP.py:326` |
| Action 超时 | 10.0 s | 服务器 `arm_control_action_authority_server_compiled.py:596` |
| Feedback 频率 | 10.0 Hz | 同上 L592 |
| LowCmd kp / kd | 50.0 / 1.5 | `g1_dual_arm_policy_controller_ROS2_v3.py:551-552` |
| 手部关节 | idx 29 = 1.0 | 同上 L557 |
| 关节角同步过期阈值 | 1.0 s | 同上 L866 |
| 手动步进 `STEP` | 0.015 m | 同上 L1162 |
| 动作速度缩放 | 0.5 | 同上 L1746 |
| 默认目标 | left=(0.1,0.25,0.70), right=(0.1,-0.25,0.70) | 服务器 L97-98 |
| 默认左模型 | `models/ppo_left_5M_Feb09/20260209-12310-lArm/ppo_g1_left_5000k.zip` | `g1_dual_arm_policy_controller_ROS2_v3.py:1665` |
| 默认右模型 | `models/ppo_right_5M_Aug28/ppo_g1_right_5000k.zip` | 同上 L1686 |
| 训练 checkpoint 间隔 | 250,000 | `train_g1_arm_policy_N8hM.py:120` |

### 6.2 部署模型核对

`models/` 下已确认存在的文件与代码默认引用一致：

```
ppo_left_5M_Feb09/20260209-12310-lArm/ppo_g1_left_5000k.zip   ✓ (默认左模型)
ppo_left_5M_Feb09/20260209-12310-lArm/ppo_g1_left_final.zip   ✓
ppo_left_10M_Oct09/20251010-100658-lArm/ppo_g1_left_final.zip ✓
ppo_left_5M_Feb10_Cons01/ppo_g1_left_5000k.zip                ✓
ppo_left_5M_Feb10_Cons01/ppo_g1_left_final.zip                ✓
ppo_right_5M_Aug28/ppo_g1_right_5000k.zip                     ✓ (默认右模型)
```

`PPO.load(..., device="cpu")`（L58）—— 实机推理在 CPU 上运行，对控制周期友好。

### 6.3 构建与运行工具链

- **依赖**：`requirements.txt`（conda 环境 `g1_vibe`）：python 3.10.18 / mujoco 3.3.5 / sb3 2.7.0 / torch 2.8.0 / gymnasium 1.2.0 / unitree-sdk2py 1.0.1 / CUDA 12.8。
- **构建**：colcon，`ament_python` 包。`setup.py` 的 `data_files` 打包 models 与 XML 到 share 目录；`console_scripts` 暴露 `arm_action_authority_server`（统一服务器）、`dual_arm_controller`（策略控制器）、`arm_test_client` 等入口。
- **运行环境**：`start_dual_arm_controller.sh` 依次 `source /opt/ros/humble/setup.bash` → workspace `install/setup.bash` → `unitree_ros2/setup_ros2_env.sh`（配置 RMW/Domain）→ 校验依赖 → 启动控制器。
- **路径解析**：`__init__.py` 的 `get_model_path` / `get_mujoco_xml_path` / `get_config_path` 优先从 `ament_index_python` 的包 share 目录查找，兜底到开发目录相对路径 —— 兼容 colcon 安装态与源码态两种运行方式。

### 6.4 潜在工程风险提示

1. **碰撞训练关闭**：当前训练默认 `_DISABLE_COLLISION=True`，模型未在碰撞惩罚下训练；部署控制器虽有碰撞冻结逻辑，但策略本身"未曾学过避开碰撞"。若实机密集任务，需重新开启碰撞训练。
2. **左右臂策略独立训练**：`-lArm`/`-rArm` 分别训练左右臂模型（`train_g1_arm_policy_N8hM.py` 运行命名），部署时分别加载两个 PPO。左右臂共享同一环境代码但参数不同（`make_env(right_arm=...)`）。README 也提示 `--right-arm` 更适合视为实验区分与路径命名信号。
3. **`__init__.py` 依赖旧环境文件**：包级 `DISTANCE_THRESHOLD` 从 `g1_arm_rl_env`（早期版）导入，而非 `g1_arm_rl_env_IhKP`；两者常量值相同（0.03），但若未来只改 IhKP 版会导致服务器侧阈值不同步。

---

## 7. 快速开始

### 7.1 环境准备

```bash
conda activate g1_vibe                          # Python 3.10 + mujoco + sb3
cd ~/RL_arm_control
source /opt/ros/humble/setup.bash
source install/setup.bash                       # colcon build 后的 workspace
source src/pkg/unitree_ros2/setup_ros2_env.sh   # Unitree DDS (CycloneDDS, Domain)
```

### 7.2 训练（开发机，无机器人）

```bash
# 右臂策略
python train_g1_arm_policy_N8hM.py --algo ppo --right-arm --total-steps 50000000 --num-envs 16 --checkpoint 250000
# 左臂策略（默认）
python train_g1_arm_policy_N8hM.py --algo ppo --total-steps 50000000 --num-envs 16 --checkpoint 250000
# 断点续训
python train_g1_arm_policy_N8hM.py --algo ppo --resume-from models/xxx/ppo_g1_left_5000k.zip
```

TensorBoard：`tensorboard --logdir runs/g1_arm_reach`。

### 7.3 仿真验证（无机器人，单臂/双臂）

```bash
# 单臂验证（部署包版默认右臂 + ens37）
python g1_arm_policy_controller.py --sim-only true
# 双臂验证（根目录版，SDK2 直连）
python g1_dual_arm_policy_controller.py
```

### 7.4 实机部署（2 终端架构）

**终端 1 —— 策略控制器：**

```bash
cd ~/RL_arm_control
source /opt/ros/humble/setup.bash && source install/setup.bash
source src/pkg/unitree_ros2/setup_ros2_env.sh
ros2 run arm_controller dual_arm_controller        # g1_dual_arm_policy_controller_ROS2_v3.py
# 或一键脚本（注意：launch 文件仍指向旧版 action server，见 3.4 断点）
bash src/arm_controller/launch/start_dual_arm_controller.sh
```

**终端 2 —— 统一服务器：**

```bash
ros2 run arm_controller arm_action_authority_server   # arm_control_action_authority_server_compiled.py
```

**终端 3 —— MCB 上位机 / 测试客户端（可选）：**

```bash
ros2 run arm_controller arm_test_client
# 注意：客户端默认 action 名为 /action_arm，需与服务器注册名 action_arm 对齐（见 3.4）
```

### 7.5 使用流程

1. 在 MCB 端确认权限：通过 `/srv_control_power_transport` 请求控制权限；或本地按 `R` 键取得手动权限。
2. 下发目标：通过 `action_arm` 发送 `target/action` + 六维坐标 + `high_resolution`；或使用预设动作点。
3. 观察 Feedback 直到 dist < 动态阈值，收到 `Result(success)`。
4. 结束/异常：MCB 释放权限、按 `Z` 触发急停（`/msg_estop` `'Y'`）冻结机器人。
