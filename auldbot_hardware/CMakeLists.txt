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

set(SCSERVO_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/SCServo_Linux)

file(GLOB SCSERVO_SOURCES ${SCSERVO_SRC_DIR}/*.cpp)

# Suppress VLA warning from third-party SCServo SDK
set_source_files_properties(
  ${SCSERVO_SRC_DIR}/SCSCL.cpp
  ${SCSERVO_SRC_DIR}/SMS_STS.cpp
  ${SCSERVO_SRC_DIR}/SMSBL.cpp
  ${SCSERVO_SRC_DIR}/SMSCL.cpp
  PROPERTIES COMPILE_OPTIONS "-Wno-vla"
)

add_library(auldbot_hardware SHARED
  src/auldbot_hardware.cpp
  ${SCSERVO_SOURCES}
)

target_include_directories(auldbot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SCSERVO_SRC_DIR}
)

ament_target_dependencies(auldbot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface auldbot_hardware_plugin.xml)

install(TARGETS auldbot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES auldbot_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
