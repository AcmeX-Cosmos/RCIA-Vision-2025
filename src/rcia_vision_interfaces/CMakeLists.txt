cmake_minimum_required(VERSION 3.10)
project(rcia_vision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${geometry_msgs_INCLUDE_DIRS})


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmorIdentifyInfo.msg"
  "msg/ArmorBaposeInfo.msg"
  "msg/SerialReceiveData.msg"
  "msg/SerialTransmitData.msg"
  "msg/GimbalCurrentInfo.msg"
  "msg/GimbalPreviousInfo.msg"
  "msg/HeartBeat.msg"
  "msg/Trajectory.msg"
  "msg/TargetSpinTop.msg"
  "msg/OdomMeasurement.msg"
  "msg/TrackerState.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
