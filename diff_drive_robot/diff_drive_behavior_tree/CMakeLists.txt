cmake_minimum_required(VERSION 3.16)
project(diff_drive_behavior_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(bt_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_DEPS
    behaviortree_ros2
    std_msgs
    std_srvs
    bt_interfaces
     )

add_executable(bt_task_planner
  src/rotate_action.cpp
  src/linear_action.cpp
  src/bt_task_planner.cpp)
ament_target_dependencies(bt_task_planner ${THIS_PACKAGE_DEPS})


add_library(rotate_plugin SHARED src/rotate_action.cpp)
target_compile_definitions(rotate_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(rotate_plugin ${THIS_PACKAGE_DEPS})

add_library(linear_plugin SHARED src/linear_action.cpp)
target_compile_definitions(linear_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(linear_plugin ${THIS_PACKAGE_DEPS})

add_executable(bt_task_planner_dyn src/bt_task_planner.cpp)
target_compile_definitions(bt_task_planner_dyn PRIVATE USE_ROTATE_PLUGIN )
target_compile_definitions(bt_task_planner_dyn PRIVATE USE_LINEAR_PLUGIN )
target_link_libraries(bt_task_planner_dyn rotate_plugin )
target_link_libraries(bt_task_planner_dyn linear_plugin )
ament_target_dependencies(bt_task_planner_dyn ${THIS_PACKAGE_DEPS})


include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

######################################################
# INSTALL

install(TARGETS
  bt_task_planner
  bt_task_planner_dyn
  rotate_plugin
  linear_plugin
  DESTINATION lib/${PROJECT_NAME}
  )

######################################################
# INSTALL plugins for other packages to load

install(TARGETS
  rotate_plugin
  linear_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )

######################################################
# INSTALL Behavior.xml's and launch files

install(DIRECTORY
    behavior_trees
    launch
    DESTINATION share/${PROJECT_NAME}/
    )


ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)

ament_package()
