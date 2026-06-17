cmake_minimum_required(VERSION 3.8)
project(object_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotino_vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV QUIET COMPONENTS core imgproc dnn)

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS ENV ONNXRUNTIME_ROOT
  PATH_SUFFIXES include include/onnxruntime include/onnxruntime/core/session
)
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  PATHS ENV ONNXRUNTIME_ROOT
  PATH_SUFFIXES lib lib64
)
if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY AND OpenCV_FOUND)
  add_executable(object_tracking src/object_tracking.cpp)
  target_include_directories(object_tracking PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${ONNXRUNTIME_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS})
  target_compile_features(object_tracking PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_link_libraries(object_tracking
    ${ONNXRUNTIME_LIBRARY}
    ${OpenCV_LIBS}
  )

  ament_target_dependencies(object_tracking
     rclcpp
     robotino_vision_msgs
     geometry_msgs
     tf2
     tf2_ros
     tf2_geometry_msgs
     sensor_msgs
  )

  install(TARGETS object_tracking
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "Skipping C++ object_tracking executable because ONNX Runtime or OpenCV was not found")
endif()

install(PROGRAMS
  src/object_tracking.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_tracking_py
)

install(DIRECTORY
  models
  DESTINATION lib/${PROJECT_NAME}/..
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

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
