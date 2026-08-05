cmake_minimum_required(VERSION 3.8)
project(color_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

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

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 编译 img_publisher 节点
add_executable(img_publisher src/img_publisher.cpp)
ament_target_dependencies(img_publisher rclcpp sensor_msgs cv_bridge OpenCV)

# 编译 img_processor 节点
add_executable(img_processor src/img_processor.cpp)
ament_target_dependencies(img_processor rclcpp sensor_msgs cv_bridge OpenCV)

# 安装可执行文件
install(TARGETS
  img_publisher
  img_processor
  DESTINATION lib/${PROJECT_NAME}
)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(sync_node src/sync_node.cpp)
ament_target_dependencies(sync_node
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
)

install(TARGETS
  sync_node
  DESTINATION lib/${PROJECT_NAME}
)
