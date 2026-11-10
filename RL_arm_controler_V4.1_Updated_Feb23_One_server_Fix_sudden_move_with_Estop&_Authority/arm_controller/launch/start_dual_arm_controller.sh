#!/bin/bash
################################################################################
# G1 双臂控制器启动脚本 (MCB Computer)
# 
# 功能: 启动双臂 RL 策略控制器，订阅 Action Server 的目标指令
# 文件: g1_dual_arm_policy_controller_ROS2_v3.py
#
# 使用方法:
#   ./start_dual_arm_controller.sh
#
# 作者: York
# 日期: 2025-11-21
################################################################################

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 动态获取工作空间路径（脚本在 src/arm_controller/launch/ 目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

echo -e "${GREEN}工作空间: ${WORKSPACE}${NC}"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}G1 双臂控制器启动脚本 (MCB)${NC}"
echo -e "${BLUE}========================================${NC}"

# 1. 验证 Python 环境
echo -e "${YELLOW}[1/4] 验证 Python 环境...${NC}"
PYTHON_PATH=$(which python3)
echo -e "${GREEN}Python 路径: ${PYTHON_PATH}${NC}"

# 2. 设置 ROS2 环境
echo -e "${YELLOW}[2/4] 配置 ROS2 环境...${NC}"
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
echo -e "${GREEN}ROS2 Humble 环境已加载${NC}"

# 3. 配置 Unitree DDS (CycloneDDS)
echo -e "${YELLOW}[3/4] 配置 Unitree DDS 通信...${NC}"
UNITREE_ENV="${WORKSPACE}/src/pkg/unitree_ros2/setup_ros2_env.sh"
if [ -f "${UNITREE_ENV}" ]; then
    source "${UNITREE_ENV}"
    echo -e "${GREEN}Unitree DDS 配置已加载${NC}"
    echo -e "${GREEN}DDS 配置: RMW=${RMW_IMPLEMENTATION}, Domain=${ROS_DOMAIN_ID}${NC}"
else
    echo -e "${YELLOW}警告: Unitree 环境脚本未找到${NC}"
    echo -e "${YELLOW}路径: ${UNITREE_ENV}${NC}"
fi

# 4. 验证依赖
echo -e "${YELLOW}[4/4] 验证依赖包...${NC}"
python3 -c "import stable_baselines3; import mujoco; import gymnasium" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}依赖验证通过 (stable-baselines3, mujoco, gymnasium)${NC}"
else
    echo -e "${RED}错误: 缺少必需的 Python 包${NC}"
    exit 1
fi

# 5. 启动控制器
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}正在启动双臂 RL 控制器...${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}提示: 按 'Ctrl+C' 停止控制器${NC}"
echo -e "${YELLOW}提示: 按 'm' 切换手动/Action模式${NC}"
echo -e "${YELLOW}提示: 按 'r' 切换机器人输出${NC}"
echo ""

cd "${WORKSPACE}"
python3 src/arm_controller/arm_controller/g1_dual_arm_policy_controller_ROS2_v3.py

# 退出清理
echo ""
echo -e "${GREEN}控制器已停止${NC}"
