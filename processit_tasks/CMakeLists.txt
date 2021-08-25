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

# catkin_package(
# 	INCLUDE_DIRS include
# 	LIBRARIES mtc_task_frame
# 	DEPENDS YAML_CPP
# )

include_directories(
  include
)

add_library(mtc_task_frame SHARED src/move_to_task_frame.cpp)
# add_dependencies(mtc_task_frame ${catkin_EXPORTED_TARGETS})
# target_link_libraries(mtc_task_frame ${catkin_LIBRARIES})
# target_compile_definitions(mtc_task_frame PRIVATE -DMOVEIT_CARTESIAN_INTERPOLATOR=${MOVEIT_CARTESIAN_INTERPOLATOR})

add_library(processit_tasks SHARED
  src/cartesian_task.cpp
)
ament_target_dependencies(processit_tasks ${THIS_PACKAGE_INCLUDE_DEPENDS})


ament_package()
