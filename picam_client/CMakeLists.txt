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
find_package(rclcpp_components REQUIRED)
find_package(picam_client_msgs REQUIRED)

include_directories(include)



add_library(picam_client SHARED
  src/picam_client_node.cpp
  src/base64.cpp
)

target_link_libraries(picam_client
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(picam_client
  rclcpp
  std_msgs
  vision_msgs
  sensor_msgs
  std_srvs
  cv_bridge
  OpenCV
  rclcpp_components
  picam_client_msgs
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_picam_client)
install(TARGETS picam_client
        EXPORT export_picam_client
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

rclcpp_components_register_node(
    picam_client
    PLUGIN "picam_client::PicamClientNode"
    EXECUTABLE picam_client_node
    EXECUTOR MultiThreadedExecutor
)


ament_package()
