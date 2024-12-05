cmake_minimum_required(VERSION 3.8)
project(diff_drive_description)

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
find_package(Eigen3 REQUIRED)
find_package(ekf REQUIRED)

set(CMAKE_CXX_STANDARD 17)


add_executable(ekf_node src/ekf_node.cpp)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})  # Add Eigen3 include directory

target_include_directories(ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ament_target_dependencies(ekf_node rclcpp std_msgs ) 

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  Eigen3
  ekf
)

ament_target_dependencies(
  ekf_node
  ${dependencies}
)

install(TARGETS
ekf_node
#direct_order_dispatcher
DESTINATION lib/${PROJECT_NAME}

)
# install the launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# install the urdf directory
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# install the urdf directory
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

# install the urdf directory
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
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
