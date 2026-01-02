cmake_minimum_required(VERSION 3.8)
project(quadruped_locomotion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadruped_locomotion REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ql_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  pluginlib
  geometry_msgs
  nav_msgs
)

target_link_libraries(${PROJECT_NAME}
  quadruped_locomotion::client
)

pluginlib_export_plugin_description_file(
  controller_interface quadruped_locomotion_controller.xml
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES quadruped_locomotion_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

endif()

ament_package()
