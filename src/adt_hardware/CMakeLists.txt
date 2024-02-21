cmake_minimum_required(VERSION 3.16)
project(ADT_HARDWARE LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  ADT_HARDWARE
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(ADT_HARDWARE PUBLIC cxx_std_17)
target_include_directories(ADT_HARDWARE PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ADT_HARDWARE>
)
ament_target_dependencies(
  ADT_HARDWARE PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ADT_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ADT_HARDWARE.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ADT_HARDWARE
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ADT_HARDWARE
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ADT_HARDWARE
)
install(TARGETS ADT_HARDWARE
  EXPORT export_ADT_HARDWARE
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_ADT_HARDWARE HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
