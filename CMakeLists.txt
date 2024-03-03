cmake_minimum_required(VERSION 3.8)
project(mtc_pour)

add_compile_options(-std=c++17)

find_package(rclcpp REQUIRED)
find_package(rviz_marker_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rviz_marker_tools
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  tf2_eigen
)

include_directories(
  include
)

add_library(mtc_pour_into SHARED src/pour_into.cpp)
ament_target_dependencies(mtc_pour_into ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(mtc_pour_into PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>/${PROJECT_NAME})

add_executable(${PROJECT_NAME}_tams_ur5_demo src/mtc_pour_tams_ur5_demo.cpp)
ament_target_dependencies(${PROJECT_NAME}_tams_ur5_demo ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}_tams_ur5_demo
	mtc_pour_into
)

add_executable(${PROJECT_NAME}_baxter_demo src/mtc_pour_baxter_demo.cpp)
ament_target_dependencies(${PROJECT_NAME}_baxter_demo ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}_baxter_demo
	mtc_pour_into
)

install(TARGETS mtc_pour_into
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS ${PROJECT_NAME}_tams_ur5_demo ${PROJECT_NAME}_baxter_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
