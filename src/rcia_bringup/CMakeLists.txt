cmake_minimum_required(VERSION 3.10)
project(rcia_bringup)

find_package(ament_cmake REQUIRED)

# 手动安装 launch 和 config 目录
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
