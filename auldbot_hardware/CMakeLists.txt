cmake_minimum_required(VERSION 3.16)
project(auldbot_hardware)

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

add_library(auldbot_hardware SHARED
  src/auldbot_hardware.cpp
)

target_compile_features(auldbot_hardware PUBLIC cxx_std_17)

target_include_directories(auldbot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(auldbot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Plain signature required here: ament_target_dependencies uses plain target_link_libraries.
target_link_libraries(auldbot_hardware SCServo::SCServo)

pluginlib_export_plugin_description_file(hardware_interface auldbot_hardware_plugin.xml)

add_executable(servo_debug
  tools/servo_debug.cpp
)
target_link_libraries(servo_debug auldbot_hardware SCServo::SCServo)
target_compile_features(servo_debug PUBLIC cxx_std_17)

add_executable(velocity_cal
  tools/velocity_cal.cpp
)
target_link_libraries(velocity_cal auldbot_hardware SCServo::SCServo)
target_compile_features(velocity_cal PUBLIC cxx_std_17)

add_executable(swap_ids
  tools/swap_ids.cpp
)
target_link_libraries(swap_ids auldbot_hardware SCServo::SCServo)
target_compile_features(swap_ids PUBLIC cxx_std_17)

install(TARGETS auldbot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS servo_debug velocity_cal swap_ids
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(FILES auldbot_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
