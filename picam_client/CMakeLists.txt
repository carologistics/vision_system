cmake_minimum_required(VERSION 3.8)
project(picam_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetConfidence.srv"
  "srv/SetIOU.srv"
  "srv/StreamControl.srv"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_library(base64 STATIC
  src/base64.cpp
  include/picam_client/base64.h
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(picam_client_node
  src/picam_client_node.cpp
)

target_link_libraries(picam_client_node
  "${cpp_typesupport_target}"
  base64
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(picam_client_node
  rclcpp
  std_msgs
  vision_msgs
  sensor_msgs
  std_srvs
  cv_bridge
  OpenCV
)

install(TARGETS
  picam_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
