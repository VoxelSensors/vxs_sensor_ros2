
cmake_minimum_required(VERSION 3.5)
project(vxs_sensor)

#Default to C++ 17?
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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

list(APPEND VXS_NODE_INCLUDE_DIRS src)

#Find VXS SDK
message( "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${SMAKE_MODULE_PATH};/usr/share/cmake/Modules")
find_package(VXSDK REQUIRED)
if (VXSDK_FOUND)
  list(APPEND NODE_INCLUDE_DIRS ${VXSDK_INCLUDE_DIR})
  list(APPEND OTHER_LIBS ${VXSDK_LIBRARY})
else()
  message("VXSDK not found!")
endif()

include_directories(${VXS_NODE_INCLUDE_DIRS})
set(VXS_NODE_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vxs_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vxs_node.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vxs_node_main.cpp)

add_executable(vxs_node ${VXS_NODE_SOURCES})
ament_target_dependencies(vxs_node ${AMENT_LIBS})
target_link_libraries(vxs_node ${OTHER_LIBS})

install(TARGETS
  vxs_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()