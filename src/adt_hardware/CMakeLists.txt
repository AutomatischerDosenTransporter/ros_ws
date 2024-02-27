cmake_minimum_required(VERSION 3.16)
project(adt_hardware LANGUAGES CXX)

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
  adt_hardware
  SHARED
  hardware/basis_motor_bridge.cpp
)
target_compile_features(adt_hardware PUBLIC cxx_std_17)
target_include_directories(adt_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/adt_hardware>
)
ament_target_dependencies(
  adt_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "adt_hardware_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface adt_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/adt_hardware
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/adt_hardware
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/adt_hardware
)
install(TARGETS adt_hardware
  EXPORT export_adt_hardware
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
ament_export_targets(export_adt_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
