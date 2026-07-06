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

install(TARGETS mote_nav
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
endif()

ament_package()
