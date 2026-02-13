cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 定义接口文件（Action、Service和Message）
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmControl.action"
  "srv/general/ControlPower.srv"
  "msg/Estop.msg"
  DEPENDENCIES action_msgs std_msgs
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)

# Linting (可选)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
