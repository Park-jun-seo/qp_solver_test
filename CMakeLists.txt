################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.12)
project(qp_tutorial)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
rclcpp
std_msgs
sensor_msgs
geometry_msgs
)

add_executable(qp_funtion_test src/qp_funtion_test.cpp)
target_link_libraries(qp_funtion_test libqpOASES.a) #시스템 라이브러리
ament_target_dependencies(qp_funtion_test ${dependencies})

add_executable(linear_prog_1 src/linear_prog_1.cpp)
target_link_libraries(linear_prog_1 libqpOASES.a) #시스템 라이브러리
ament_target_dependencies(linear_prog_1 ${dependencies})

add_executable(linear_prog_2 src/linear_prog_2.cpp)
target_link_libraries(linear_prog_2 libqpOASES.a) #시스템 라이브러리
ament_target_dependencies(linear_prog_2 ${dependencies})

add_executable(balancing_node src/balancing_node.cpp)
target_link_libraries(balancing_node libqpOASES.a) #시스템 라이브러리
ament_target_dependencies(balancing_node ${dependencies})

add_executable(example4 src/example4.cpp)
target_link_libraries(example4 libqpOASES.a) #시스템 라이브러리
ament_target_dependencies(example4 ${dependencies})

add_executable(quadratic_prog src/quadratic_prog.cpp)
target_link_libraries(quadratic_prog libqpOASES.a) #시스템 라이브러리
ament_target_dependencies(quadratic_prog ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS
  qp_funtion_test
  linear_prog_1
  linear_prog_2
  example4
  balancing_node
  quadratic_prog
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/balancing_robot.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  mjcf
  DESTINATION share/${PROJECT_NAME}/
)
################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()