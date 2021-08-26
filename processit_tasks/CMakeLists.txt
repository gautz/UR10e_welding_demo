cmake_minimum_required(VERSION 3.5)
project(processit_tasks)

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
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosparam_shortcuts REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(processit_msgs REQUIRED)
find_package(processit_cax REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  rosparam_shortcuts
  geometry_msgs
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  tf2_geometry_msgs
  processit_msgs
  processit_cax
)

include_directories(include)

add_library(processit_tasks SHARED src/cartesian_task.cpp)
ament_target_dependencies(processit_tasks ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(mtc_task_frame SHARED src/move_to_task_frame.cpp)
set_target_properties(mtc_task_frame PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(mtc_task_frame ${THIS_PACKAGE_INCLUDE_DEPENDS})
# rclcpp_components_register_nodes(mtc_task_frame "MoveIt::TaskConstructor::Stages::MoveToTaskFrame")

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    mtc_task_frame
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  mtc_task_frame
  )
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
