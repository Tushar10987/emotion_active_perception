cmake_minimum_required(VERSION 3.8)
project(emotion_active_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(action_files
  src/emotion_active_perception/actions/MoveAndCapture.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs unique_identifier_msgs action_msgs
)

if(NOT TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/emotion_active_perception)
endif()

# Ensure Python package is present in site-packages for runtime
install(DIRECTORY src/emotion_active_perception/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

install(PROGRAMS
  scripts/ap_action_server
  scripts/emotion_controller
  scripts/uncertainty_node
  scripts/auto_labeler_node
  scripts/data_manager_node
  scripts/example_drl_agent
  DESTINATION lib/${PROJECT_NAME}
)

# console scripts are provided via entry_points; no need to install scripts

install(FILES resource/emotion_active_perception
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY src/emotion_active_perception/config
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

