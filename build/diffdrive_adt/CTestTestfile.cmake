# CMake generated Testfile for 
# Source directory: /workspaces/ros_ws/src/diffdrive_adt
# Build directory: /workspaces/ros_ws/build/diffdrive_adt
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(example_2_urdf_xacro "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/workspaces/ros_ws/build/diffdrive_adt/test_results/diffdrive_adt/example_2_urdf_xacro.xunit.xml" "--package-name" "diffdrive_adt" "--output-file" "/workspaces/ros_ws/build/diffdrive_adt/ament_cmake_pytest/example_2_urdf_xacro.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/workspaces/ros_ws/src/diffdrive_adt/test/test_urdf_xacro.py" "-o" "cache_dir=/workspaces/ros_ws/build/diffdrive_adt/ament_cmake_pytest/example_2_urdf_xacro/.cache" "--junit-xml=/workspaces/ros_ws/build/diffdrive_adt/test_results/diffdrive_adt/example_2_urdf_xacro.xunit.xml" "--junit-prefix=diffdrive_adt")
set_tests_properties(example_2_urdf_xacro PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/workspaces/ros_ws/build/diffdrive_adt" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/workspaces/ros_ws/src/diffdrive_adt/CMakeLists.txt;69;ament_add_pytest_test;/workspaces/ros_ws/src/diffdrive_adt/CMakeLists.txt;0;")
add_test(view_example_2_launch "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/workspaces/ros_ws/build/diffdrive_adt/test_results/diffdrive_adt/view_example_2_launch.xunit.xml" "--package-name" "diffdrive_adt" "--output-file" "/workspaces/ros_ws/build/diffdrive_adt/ament_cmake_pytest/view_example_2_launch.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/workspaces/ros_ws/src/diffdrive_adt/test/test_view_robot_launch.py" "-o" "cache_dir=/workspaces/ros_ws/build/diffdrive_adt/ament_cmake_pytest/view_example_2_launch/.cache" "--junit-xml=/workspaces/ros_ws/build/diffdrive_adt/test_results/diffdrive_adt/view_example_2_launch.xunit.xml" "--junit-prefix=diffdrive_adt")
set_tests_properties(view_example_2_launch PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/workspaces/ros_ws/build/diffdrive_adt" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/workspaces/ros_ws/src/diffdrive_adt/CMakeLists.txt;70;ament_add_pytest_test;/workspaces/ros_ws/src/diffdrive_adt/CMakeLists.txt;0;")