cmake_minimum_required(VERSION 3.16)
project(mote_hardware)

cmake_policy(SET CMP0167 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(SCServo CONFIG REQUIRED)

add_library(mote_hardware SHARED
  src/mote_hardware.cpp
)

target_compile_features(mote_hardware PUBLIC cxx_std_17)

target_include_directories(mote_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mote_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Plain signature required here: ament_target_dependencies uses plain target_link_libraries.
target_link_libraries(mote_hardware SCServo::SCServo)

pluginlib_export_plugin_description_file(hardware_interface mote_hardware_plugin.xml)

add_executable(servo_debug
  tools/servo_debug.cpp
)
target_link_libraries(servo_debug mote_hardware SCServo::SCServo)
target_compile_features(servo_debug PUBLIC cxx_std_17)

add_executable(velocity_cal
  tools/velocity_cal.cpp
)
target_link_libraries(velocity_cal mote_hardware SCServo::SCServo)
target_compile_features(velocity_cal PUBLIC cxx_std_17)

add_executable(swap_ids
  tools/swap_ids.cpp
)
target_link_libraries(swap_ids mote_hardware SCServo::SCServo)
target_compile_features(swap_ids PUBLIC cxx_std_17)

add_executable(setup_ids
  tools/setup_ids.cpp
)
target_link_libraries(setup_ids mote_hardware SCServo::SCServo)
target_compile_features(setup_ids PUBLIC cxx_std_17)

add_executable(servo_ping
  tools/servo_ping.cpp
)
target_link_libraries(servo_ping mote_hardware SCServo::SCServo)
target_compile_features(servo_ping PUBLIC cxx_std_17)

install(TARGETS mote_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS servo_debug velocity_cal swap_ids setup_ids servo_ping
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(FILES mote_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_encoder test/test_encoder.cpp)
  target_include_directories(test_encoder PRIVATE include)
endif()

ament_package()
