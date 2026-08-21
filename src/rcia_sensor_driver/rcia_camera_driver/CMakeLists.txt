# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(rcia_camera_driver)

# 查找ROS2核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(camera_info_manager REQUIRED)

# 查找相机SDK库
set(MVVIEWER_INCLUDE_DIR /opt/HuarayTech/MVviewer/include)
set(MVVIEWER_LIB_DIR /opt/HuarayTech/MVviewer/lib)

# 添加头文件目录
include_directories(
  include
  ${MVVIEWER_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# 定义源文件
set(SOURCES
  src/camera_node.cpp
)

# 生成节点库
add_library(camera_driver SHARED ${SOURCES})
target_compile_features(camera_driver PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(camera_driver
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  rclcpp_components 
  camera_info_manager
)

# 链接相机SDK库和OpenCV
target_link_libraries(camera_driver
  ${OpenCV_LIBS}
  ${MVVIEWER_LIB_DIR}/libMVSDK.so
)

# 注册组件节点
rclcpp_components_register_nodes(camera_driver
  rcia::camera_driver::CameraNode
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装配置
install(TARGETS camera_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

ament_package()