cmake_minimum_required(VERSION 3.10)
project(rcia_vision_detector)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 优化编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -flto -funroll-loops")

# 查找ROS2包
find_package (ament_cmake_auto REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcia_vision_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(class_loader REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED CONFIG)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rosidl_default_generators REQUIRED)
find_package(angles REQUIRED)

# 查找OpenCV包
find_package(OpenCV 4.9.0 EXACT REQUIRED)

# 查找其他依赖库
find_package(Eigen3 REQUIRED HINTS $ENV{EIGEN3_DIR})
find_package(Ceres REQUIRED HINTS $ENV{CERES_DIR})
ament_auto_find_build_dependencies()

include_directories(include)
include_directories(/opt/HuarayTech/MVviewer/include)
include_directories(${angles_INCLUDE_DIRS})

# 设置 ONNX Runtime 路径。可通过 ONNXRUNTIME_ROOT 指向本地安装目录。
if(DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT_PATH "$ENV{ONNXRUNTIME_ROOT}")
else()
  set(ONNXRUNTIME_ROOT_PATH /opt/onnxruntime)
endif()
set(ONNXRUNTIME_INCLUDE_DIRS 
    ${ONNXRUNTIME_ROOT_PATH}/include/onnxruntime
    ${ONNXRUNTIME_ROOT_PATH}/include/onnxruntime/core/session/)
set(ONNXRUNTIME_LIB ${ONNXRUNTIME_ROOT_PATH}/build/Linux/Release/libonnxruntime.so)

set(MVVIEWER_LIB /opt/HuarayTech/MVviewer/lib/libMVSDK.so)

# 添加库目标
ament_auto_add_library(rcia_vision_detector_lib SHARED
  DIRECTORY src
)

# 设置目标包含目录
target_include_directories(rcia_vision_detector_lib PRIVATE
  /opt/HuarayTech/MVviewer/include
  /usr/local/include/glog

  # 项目源码包含目录
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rcia_vision_interfaces_INCLUDE_DIRS}

  ${Eigen3_INCLUDE_DIRS}
  ${Ceres_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
)

# 添加依赖项
ament_target_dependencies(rcia_vision_detector_lib
  rclcpp
  rclcpp_components
  image_transport
  cv_bridge
  ament_index_cpp
  
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs

  rcia_vision_interfaces
)

# 链接库
target_link_libraries(rcia_vision_detector_lib
  ${CMAKE_DL_LIBS}
  
  class_loader::class_loader
  rclcpp_components::component

  # ROS2库
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  
  glog
  pthread
  tf2_geometry_msgs::tf2_geometry_msgs
  ${CERES_LIBRARIES}
  ${ONNXRUNTIME_LIB}
  ${tf2_geometry_msgs_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MVVIEWER_LIB}
)

# 注册 ImageSubscriber 为 ROS 2 组件
rclcpp_components_register_node(rcia_vision_detector_lib
  PLUGIN "rcia::vision_identify::ArmorIdentifier"
  EXECUTABLE rcia_vision_detector_node
)

# 安装目标
install(TARGETS rcia_vision_detector_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY classify_model/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_components
  class_loader
  ament_index_cpp

  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
)

ament_export_include_directories(include)
ament_export_libraries(rcia_vision_detector_lib)

ament_package()
