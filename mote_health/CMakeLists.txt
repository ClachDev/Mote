cmake_minimum_required(VERSION 3.16)
project(mote_health)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# The roll-up rules, the config reader, and the two pieces of mote_bringup this
# node cannot import because it is not Python. A library rather than sources on
# the executable so the gtests link the same objects the node runs.
add_library(mote_health SHARED
  src/config.cpp
  src/health_monitor.cpp
  src/health_rollup.cpp
  src/mote_home.cpp
  src/sd_notify.cpp
)

target_compile_features(mote_health PUBLIC cxx_std_17)

target_include_directories(mote_health PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mote_health yaml-cpp::yaml-cpp)

ament_target_dependencies(mote_health
  ament_index_cpp
  diagnostic_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

add_executable(health_monitor src/health_monitor_main.cpp)
target_link_libraries(health_monitor mote_health)
ament_target_dependencies(health_monitor rclcpp)

install(TARGETS mote_health
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Into lib/${PROJECT_NAME}, which is where `ros2 run` looks.
install(TARGETS health_monitor DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(sensor_msgs REQUIRED)

  # The roll-up decisions: the port of test_health_monitor.py's _TopicWatch /
  # _TfWatch cases, which are the behaviour rather than the subscriptions.
  ament_add_gtest(test_health_rollup test/test_health_rollup.cpp)
  target_link_libraries(test_health_rollup mote_health)

  ament_add_gtest(test_config test/test_config.cpp)
  target_link_libraries(test_config mote_health)

  ament_add_gtest(test_mote_home test/test_mote_home.cpp)
  target_link_libraries(test_mote_home mote_health)

  ament_add_gtest(test_sd_notify test/test_sd_notify.cpp)
  target_link_libraries(test_sd_notify mote_health)

  # The live-node half: a generic subscription that delivers nothing leaves a
  # node which still runs, still publishes on time, and reports every subsystem
  # as missing — so a real publisher has to reach a real summary.
  ament_add_gtest(test_health_monitor_node test/test_health_monitor_node.cpp TIMEOUT 120)
  target_link_libraries(test_health_monitor_node mote_health)
  ament_target_dependencies(test_health_monitor_node rclcpp sensor_msgs std_msgs diagnostic_msgs)
endif()

ament_package()
