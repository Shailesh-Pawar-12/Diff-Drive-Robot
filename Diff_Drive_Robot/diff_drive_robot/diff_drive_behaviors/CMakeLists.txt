cmake_minimum_required(VERSION 3.8)
project(diff_drive_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(nav2_behaviors REQUIRED)
find_package(bt_interfaces REQUIRED)



set(dependencies
  nav2_behaviors
  bt_interfaces
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)


# plugins
add_library(diff_drive_rotate_behavior SHARED
  src/rotate.cpp
)

# plugins
add_library(diff_drive_linear_behavior SHARED
  src/linear.cpp
)

ament_target_dependencies(diff_drive_rotate_behavior
  ${dependencies}
)

ament_target_dependencies(diff_drive_linear_behavior
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS
  diff_drive_rotate_behavior
  diff_drive_linear_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src/
)


# ament_export_include_directories(include)
ament_export_libraries(
  diff_drive_rotate_behavior
  diff_drive_linear_behavior
)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
