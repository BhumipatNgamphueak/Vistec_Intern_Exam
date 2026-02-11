cmake_minimum_required(VERSION 3.8)
project(deploy_policy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts (rename to remove .py extension)
install(PROGRAMS
  scripts/go2_deploy_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_deploy_node
)

install(PROGRAMS
  scripts/data_logger_gazebo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME data_logger_gazebo
)

install(PROGRAMS
  scripts/data_logger_gazebo_v2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME data_logger_gazebo_v2
)

install(PROGRAMS
  scripts/data_logger_gazebo_single.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME data_logger_gazebo_single
)

install(PROGRAMS
  scripts/pose_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_to_odom
)

install(PROGRAMS
  scripts/imu_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_to_odom
)

install(PROGRAMS
  scripts/isaac_test_experiments.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME isaac_test_experiments
)

install(PROGRAMS
  scripts/isaac_data_logger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME isaac_data_logger
)

install(PROGRAMS
  scripts/isaac_integrated_experiment.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME isaac_integrated_experiment
)

install(PROGRAMS
  scripts/gazebo_state_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_state_to_odom
)

install(PROGRAMS
  scripts/gazebo_model_state_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_model_state_to_odom
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
