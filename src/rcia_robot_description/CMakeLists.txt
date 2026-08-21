cmake_minimum_required(VERSION 3.10)
project(rcia_robot_description)

# 编译器警告设置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖查找
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装资源配置
install(
  DIRECTORY urdf 
  DESTINATION share/${PROJECT_NAME}
)

# ROS2包声明
ament_package()