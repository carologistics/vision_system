cmake_minimum_required(VERSION 3.8)
project(object_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(robotino_vision_msgs REQUIRED)
find_package(tf2)
find_package(tf2_ros)
find_package(geometry_msgs)


add_executable(object_tracking src/object_tracking.cpp)
target_include_directories(object_tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(object_tracking PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(object_tracking
   rclcpp
   robotino_vision_msgs
   geometry_msgs
   tf2
   tf2_ros
)

install(TARGETS object_tracking
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
