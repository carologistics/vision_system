cmake_minimum_required(VERSION 3.8)
project(picam_client_ros)

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
  "msg/BoundingBox.msg"
  "srv/SetConfidence.srv" 
  "srv/SetIOU.srv"
  "srv/StreamControl.srv"
)

include_directories(include)

add_library(base64 src/base64.cpp)

add_executable(picam_client_node 
  src/picam_client_node.cpp
)

ament_target_dependencies(picam_client_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(picam_client_node
  base64
)

install(TARGETS
  picam_client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()