cmake_minimum_required(VERSION 3.5)
project(linetracer_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) #camera opencv publisher
find_package(OpenCV REQUIRED) #camera opencv publisher
find_package(geometry_msgs REQUIRED) #dynamixel_subscriber

include_directories(include)

add_executable(linetracer_linux src/dxl.cpp src/camera.cpp src/linetracer_linux.cpp)
ament_target_dependencies(linetracer_linux rclcpp sensor_msgs std_msgs OpenCV geometry_msgs)
target_link_libraries(linetracer_linux dxl_x64_cpp)
target_include_directories(linetracer_linux PRIVATE /usr/local/include/dynamixel_sdk_cpp)

install(TARGETS
  linetracer_linux
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
