
cmake_minimum_required(VERSION 3.5)
project(vxs_sensor)

#Default to C++ 17?
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
list(APPEND AMENT_LIBS ament_cmake)
find_package(rclcpp REQUIRED)
list(APPEND AMENT_LIBS rclcpp)
find_package(cv_bridge REQUIRED)
list(APPEND AMENT_LIBS cv_bridge)
find_package(image_transport REQUIRED)
list(APPEND AMENT_LIBS image_transport)
find_package(std_msgs REQUIRED)
list(APPEND AMENT_LIBS std_msgs)

#let's see...
message( "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${SMAKE_MODULE_PATH};/usr/share/cmake/Modules")
find_package(VXSDK REQUIRED)
list(APPEND AMENT_LIBS VXSDK)

add_executable(vxs_node src/vxs_node.cpp)
ament_target_dependencies(vxs_node ${AMENT_LIBS})

install(TARGETS
  vxs_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()