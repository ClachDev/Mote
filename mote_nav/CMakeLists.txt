cmake_minimum_required(VERSION 3.16)
project(mote_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(mote_nav SHARED
  src/wheel_speed_limit_critic.cpp
)

target_compile_features(mote_nav PUBLIC cxx_std_17)

target_include_directories(mote_nav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mote_nav
  rclcpp
  dwb_core
  dwb_msgs
  nav_2d_msgs
  nav2_costmap_2d
  nav2_core
  nav2_util
  pluginlib
  geometry_msgs
)

pluginlib_export_plugin_description_file(dwb_core wheel_speed_critic_plugins.xml)

# Kept out of the critic library so loading a DWB plugin does not drag tf2_ros
# and the odometry message definitions into the controller server.
add_library(odom_tf_relay_component SHARED
  src/odom_tf_relay.cpp
)

target_compile_features(odom_tf_relay_component PUBLIC cxx_std_17)

target_include_directories(odom_tf_relay_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# The relay's output is compared against the Python node it replaces with an
# exact equality, so the arithmetic must not be re-associated. aarch64 has FMA
# in its baseline, and GCC contracts multiply-add pairs into it by default,
# which rounds once where Python rounds twice.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(odom_tf_relay_component PRIVATE -ffp-contract=off)
endif()

ament_target_dependencies(odom_tf_relay_component
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2_ros
)

# Also builds a standalone `odom_tf_relay` executable, which is how the node is
# run outside a container (and how the two implementations were compared).
rclcpp_components_register_node(odom_tf_relay_component
  PLUGIN "mote_nav::OdomTfRelay"
  EXECUTABLE odom_tf_relay
)

install(TARGETS mote_nav odom_tf_relay_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES wheel_speed_critic_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_wheel_speed_limit test/test_wheel_speed_limit.cpp)
  target_link_libraries(test_wheel_speed_limit mote_nav)

  ament_add_gtest(test_plugin_load test/test_plugin_load.cpp)
  ament_target_dependencies(test_plugin_load dwb_core pluginlib)

  ament_add_gtest(test_odom_tf_relay test/test_odom_tf_relay.cpp)
  target_link_libraries(test_odom_tf_relay odom_tf_relay_component)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(test_odom_tf_relay PRIVATE -ffp-contract=off)
  endif()
endif()

ament_package()
