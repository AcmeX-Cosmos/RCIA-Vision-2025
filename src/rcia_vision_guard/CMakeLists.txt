cmake_minimum_required(VERSION 3.10)
project(rcia_vision_guard)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcia_vision_interfaces REQUIRED)

include_directories(
  include
  ${rcia_vision_interfaces_INCLUDE_DIRS}
)


# 定义可执行文件目标
add_executable(guard_dog
  src/guard_dog.cpp
)

# 添加依赖项
ament_target_dependencies(guard_dog
  rclcpp
  rcia_vision_interfaces
)

# 安装可执行文件
install(TARGETS guard_dog
  DESTINATION lib/${PROJECT_NAME})


# 编译选项
if(CMAKE_COMPILERIS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 添加 ament_package 调用
ament_package()
