cmake_minimum_required(VERSION 3.8)
project(yathra)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(yathra_serial src/yathra_serial.cpp)
add_executable(dummy_depth src/dummy_depth.cpp)

ament_target_dependencies(yathra_serial 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs
)

ament_target_dependencies(dummy_depth rclcpp geometry_msgs nav_msgs)

install(TARGETS
  dummy_depth
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
  yathra_serial
  DESTINATION lib/${PROJECT_NAME}
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
