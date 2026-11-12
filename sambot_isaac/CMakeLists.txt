cmake_minimum_required(VERSION 3.8)
project(sambot_isaac)

find_package(ament_cmake REQUIRED)

# ROS-side helper node (cmd_vel relay)
install(PROGRAMS scripts/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_relay)

# Isaac-side scripts run inside Isaac's own Python, so they are installed as data.
install(
  DIRECTORY usd urdf scripts launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
