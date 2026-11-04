# Lyrical compatibility for the unpatched submodules.
#
# colcon.meta injects this file into sllidar_ros2 and kinematic_icp only, as
# CMAKE_PROJECT_<name>_INCLUDE, so it runs straight after their project() call.
# Mote's own packages link exported targets directly and must not rely on it.

# ament_ros_core's exported targets carry c_std compile features, which CMake
# refuses to evaluate in a project that did not enable C. kinematic_icp
# declares LANGUAGES CXX.
enable_language(C)

# Lyrical renamed tf2/time.h to tf2/time.hpp; include/ forwards the old name.
include_directories(BEFORE ${CMAKE_CURRENT_LIST_DIR}/include)

# ament_cmake removed ament_target_dependencies() in Lyrical; both submodules
# still call it. Plain signature only, matching both callers: each package's
# exported ${pkg}_TARGETS are linked, falling back to ${pkg}_INCLUDE_DIRS and
# ${pkg}_LIBRARIES for a package that exports no targets.
function(ament_target_dependencies target)
  foreach(pkg ${ARGN})
    if(NOT ${pkg}_FOUND)
      message(FATAL_ERROR "ament_target_dependencies(): '${pkg}' was not found before")
    endif()
    if(${pkg}_TARGETS)
      target_link_libraries(${target} ${${pkg}_TARGETS})
    else()
      target_include_directories(${target} PUBLIC ${${pkg}_INCLUDE_DIRS})
      target_compile_definitions(${target} PUBLIC ${${pkg}_DEFINITIONS})
      target_link_libraries(${target} ${${pkg}_LIBRARIES})
    endif()
  endforeach()
endfunction()
