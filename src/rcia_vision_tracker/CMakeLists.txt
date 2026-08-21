cmake_minimum_required(VERSION 3.10)
project(rcia_vision_tracker)

#----------------------------- 编译器标准配置 -----------------------------#
# C++标准配置（ROS2要求C++17及以上）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)    # 使用C++17特性
endif()

#----------------------------- 编译优化配置 ------------------------------#
# 针对Intel/AMD处理器优化（根据实际硬件调整）
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -flto -funroll-loops")

# 确保符号默认可见
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=default")

#------------------------------ 依赖包配置 -------------------------------#
# 基础依赖
find_package(ament_cmake REQUIRED)       # ROS2构建系统
find_package(rclcpp REQUIRED)            # ROS2 C++客户端库
find_package(rclcpp_components REQUIRED) # 组件化支持
find_package(visualization_msgs REQUIRED)

# 消息接口依赖
find_package(rcia_vision_interfaces REQUIRED) # 自定义视觉消息
find_package(sensor_msgs REQUIRED)       # 标准传感器消息
find_package(geometry_msgs REQUIRED)     # 几何消息类型
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

# 数学求解依赖
find_package(OpenCV REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED HINTS $ENV{EIGEN3_DIR})
find_package(Ceres REQUIRED HINTS $ENV{CERES_DIR})


#--------------------------- 组件化节点配置 ----------------------------#
# 定义组件库（共享库形式）
add_library(${PROJECT_NAME}_lib SHARED
  src/vision_tracker_node.cpp
  src/spinTop_tracker.cpp
  src/spinTop_predictor.cpp
  src/gimbal_control.cpp
  src/guard_dog_publish.cpp
  src/trajectory_compensator.cpp
)

# 注册节点
rclcpp_components_register_nodes(${PROJECT_NAME}_lib
  "rcia::vision_tracker::VisionTrackerNode"
)

install(TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

#------------------------- 包含目录配置（层级说明） ----------------------#
target_include_directories(${PROJECT_NAME}_lib PRIVATE
  # 第三方库路径
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}

  # 源码路径
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # 本项目公共头文件
  ${CMAKE_CURRENT_SOURCE_DIR}/src      # 源码目录（可选）

  # ROS接口路径
  ${rcia_vision_interfaces_INCLUDE_DIRS}    # 自定义接口头文件
  ${sensor_msgs_INCLUDE_DIRS}          # 标准传感器消息头文件
  ${geometry_msgs_INCLUDE_DIRS}        # 几何消息类型头文件
  ${tf2_geometry_msgs_INCLUDE_DIRS}    # tf2几何消息类型头文件
)

include_directories(
  ${angles_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
)

#--------------------------- 依赖链接配置 ----------------------------#
# 添加组件依赖项
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  rcia_vision_interfaces
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs

  OpenCV
  Eigen3
  Ceres
)

#--------------------------- 安装配置 ------------------------------#
# 安装组件库
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib   # 静态库安装路径
  LIBRARY DESTINATION lib   # 动态库安装路径
  RUNTIME DESTINATION bin   # 可执行文件路径
)

# 安装包含目录（如果需要被其他包引用）
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 链接库
target_link_libraries(${PROJECT_NAME}_lib
  ${CERES_LIBRARIES}
)

#------------------------ 编译器警告配置 -------------------------#
# 启用严格编译检查
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall       # 所有警告
    -Wextra     # 额外警告
    -Wpedantic  # 严格标准检查
    -Werror     # 视警告为错误（调试阶段可注释）
  )
endif()

#------------------------ 测试配置 -----------------------------#
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()  # 自动查找测试依赖
endif()

#----------------------- 项目元数据配置 ------------------------#
ament_package()