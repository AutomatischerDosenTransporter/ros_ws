cmake_minimum_required(VERSION 3.8)
project(adt_code)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(adt_code_msg REQUIRED)

add_executable(service_roboter_motor_driver src/service_roboter_motor_driver.cpp)
add_executable(basis_roboter_relay_driver src/basis_roboter_relay_driver.cpp)
add_executable(basis_roboter_motor_driver src/basis_roboter_motor_driver.cpp)
add_executable(basis_roboter_motor_manager src/basis_roboter_motor_manager.cpp)
add_executable(arduino_bridge src/arduino_bridge.cpp)

ament_target_dependencies(service_roboter_motor_driver rclcpp std_msgs geometry_msgs adt_code_msg)
ament_target_dependencies(basis_roboter_relay_driver rclcpp std_msgs geometry_msgs adt_code_msg)
ament_target_dependencies(basis_roboter_motor_driver rclcpp std_msgs geometry_msgs adt_code_msg)
ament_target_dependencies(basis_roboter_motor_manager rclcpp std_msgs geometry_msgs adt_code_msg)
ament_target_dependencies(arduino_bridge rclcpp std_msgs adt_code_msg)

target_link_libraries(service_roboter_motor_driver serial)
target_link_libraries(basis_roboter_relay_driver serial)
target_link_libraries(basis_roboter_motor_driver serial)
target_link_libraries(basis_roboter_motor_manager serial)
target_link_libraries(arduino_bridge serial)

target_include_directories(service_roboter_motor_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include>)
target_include_directories(basis_roboter_relay_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include>)
target_include_directories(basis_roboter_motor_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include>)
target_include_directories(basis_roboter_motor_manager PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include>)
target_include_directories(arduino_bridge PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include>)
target_compile_features(service_roboter_motor_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(basis_roboter_relay_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(basis_roboter_motor_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(basis_roboter_motor_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(arduino_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS service_roboter_motor_driver DESTINATION lib/${PROJECT_NAME})
install(TARGETS basis_roboter_relay_driver DESTINATION lib/${PROJECT_NAME})
install(TARGETS basis_roboter_motor_driver DESTINATION lib/${PROJECT_NAME})
install(TARGETS basis_roboter_motor_manager DESTINATION lib/${PROJECT_NAME})
install(TARGETS arduino_bridge DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
