cmake_minimum_required(VERSION 3.8)
project(clearpath_robots_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  include
)


add_library(clearpath_robots_sim SHARED
  src/hardware_interfaces/diff_drive/hardware.cpp
  src/hardware_interfaces/diff_drive/hardware_interface.cpp
)

add_library(clearpath_robots_sim__path_followers SHARED
  src/path_followers/MPC/mpc_controller.cpp
)

target_include_directories(clearpath_robots_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(clearpath_robots_sim__path_followers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(clearpath_robots_sim
  rclcpp
  hardware_interface
  pluginlib
)

ament_target_dependencies(clearpath_robots_sim__path_followers
  rclcpp
  tf2
  Eigen3
  geometry_msgs
  nav_msgs
  sensor_msgs
)

pluginlib_export_plugin_description_file(
  hardware_interface
  src/hardware_interfaces/diff_drive/hardware.xml
)

install(
  TARGETS clearpath_robots_sim
  EXPORT export_clearpath_robots_sim
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS clearpath_robots_sim__path_followers
  EXPORT export_clearpath_robots_sim__path_followers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES src/hardware_interfaces/diff_drive/hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

# MPC controller node
add_executable(mpc_controller_node src/path_followers/MPC/mpc_controller_node.cpp)
target_link_libraries(mpc_controller_node
  clearpath_robots_sim__path_followers
)

install(
  TARGETS mpc_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
