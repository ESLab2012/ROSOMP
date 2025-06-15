cmake_minimum_required(VERSION 3.5)

# Replace with the correct path
set(OpenMP_C_FLAGS "-fopenmp")
set(OpenMP_C_LIB_NAMES "omp")
set(OpenMP_omp_LIBRARY "/home/neu/Disk/llvm-install/lib/libomp.so")

project(intra_process_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)

# Added lookup for OpenMP
find_package(OpenMP REQUIRED)

include_directories(include)
# TODO(sloretz) stop exporting old-style CMake variables in the future
ament_export_include_directories("include/${PROJECT_NAME}")

##
## Demos
##

# Simple example of using unique_ptr to minimize copies.
add_executable(two_node_pipeline
  src/two_node_pipeline/two_node_pipeline.cpp)
target_link_libraries(two_node_pipeline
  rclcpp::rclcpp
  ${std_msgs_TARGETS})

add_executable(two_node_pipeline_casestudy
  src/two_node_pipeline/two_node_pipeline_casestudy.cpp)
target_link_libraries(two_node_pipeline_casestudy
  rclcpp::rclcpp
  ${std_msgs_TARGETS})

if(OPENMP_FOUND)
  target_compile_options(two_node_pipeline 
    PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(two_node_pipeline 
    ${OpenMP_CXX_LIBRARIES})
endif() 

if(OPENMP_FOUND)
  target_compile_options(two_node_pipeline_casestudy
    PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(two_node_pipeline_casestudy
    ${OpenMP_CXX_LIBRARIES})
endif() 

install(TARGETS
  two_node_pipeline
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  two_node_pipeline_casestudy
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

  # Add each test case.  Multi-executable tests can be specified in
  # semicolon-separated strings, like  exe1:exe2.
  set(demo_tests
    two_node_pipeline)
  
  set(demo_tests
    two_node_pipeline_casestudy)

  macro(tests)
    foreach(demo_test ${demo_tests})
      string(REPLACE ":" ";" demo_executables "${demo_test}")
      set(RCLCPP_DEMOS_EXPECTED_OUTPUT "")
      foreach(executable ${demo_executables})
        list(APPEND RCLCPP_DEMOS_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/${executable}")
      endforeach()

      set(RCLCPP_DEMOS_EXECUTABLE "")
      foreach(executable ${demo_executables})
        list(APPEND RCLCPP_DEMOS_EXECUTABLE "$<TARGET_FILE:${executable}>")
      endforeach()

      string(REPLACE ";" "_" exe_list_underscore "${demo_executables}")
      configure_file(
        test/test_executables_demo.py.in
        test_${exe_list_underscore}${target_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}.py.configured"
      )

      add_launch_test(
        "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        TARGET test_demo_${exe_list_underscore}${target_suffix}
        TIMEOUT 30
        ENV
        RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
        RMW_IMPLEMENTATION=${rmw_implementation}
      )
      if(TEST test_demo_${exe_list_underscore}${target_suffix})
        set_tests_properties(test_demo_${exe_list_underscore}${target_suffix}
          PROPERTIES DEPENDS "test_demo_${exe_list_underscore}${target_suffix}  test_demo_${exe_list_underscore}${target_suffix}")
      endif()
    endforeach()
  endmacro()

  call_for_each_rmw_implementation(tests)
endif()

ament_package()