cmake_minimum_required(VERSION 3.5)
project(LAB1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/dummy_script.py
  scripts/static_tf_publisher.py
  scripts/explore_bag.py
  scripts/plot_wheel_odom.py
  scripts/odom_to_path.py
  LAB1/wheel_odom_node.py
  LAB1/ekf_fusion_node.py
  scripts/plot_wheel_ekf_comparison.py
  scripts/record_phase2_data.py
  scripts/plot_phase2_offline.py

  LAB1/icp_localization_node.py
  scripts/record_phase3_data.py
  scripts/plot_phase3_offline.py
  scripts/icp_comparison_plotter.py

  scripts/record_phase4_data.py
  scripts/plot_phase4_offline.py
  scripts/odom_to_path_multi.py
  DESTINATION lib/${PROJECT_NAME}
)


# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node_test src/cpp_node.cpp)
ament_target_dependencies(cpp_node_test rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node_test
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  launch
  config
  results

  DESTINATION share/${PROJECT_NAME})

ament_package()
