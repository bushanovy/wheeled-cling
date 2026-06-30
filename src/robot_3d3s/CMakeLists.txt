cmake_minimum_required(VERSION 3.8)
project(robot_3d3s)

find_package(ament_cmake REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(robot_3d3s_kmw100_adhesion_system SHARED
  src/kmw100_adhesion_system.cpp
)
target_link_libraries(robot_3d3s_kmw100_adhesion_system
  gz-msgs10::gz-msgs10
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)

install(TARGETS robot_3d3s_kmw100_adhesion_system
  LIBRARY DESTINATION lib
)

# Install all resource directories into the package share folder
install(DIRECTORY
  config
  launch
  meshes
  models
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts so they can be run as nodes
install(PROGRAMS
  scripts/joint_control_gui.py
  scripts/motion_demo.py
  scripts/motion_demo_swerve.py
  scripts/teleop_keyboard.py
  scripts/teleop_keyboard_swerve.py
  scripts/teleop_keyboard_swerve_gazebo.py
  scripts/odom_node.py
  scripts/cmd_vel_to_wheels.py
  scripts/cmd_vel_stamper.py
  scripts/climb_surface_controller.py
  scripts/climb_rviz_goal_bridge.py
  scripts/climb_rviz_robot_visualizer.py
  scripts/mockup_contact_monitor.py
  scripts/magnetic_adhesion.py
  scripts/climb_scene_rviz.py
  scripts/adhesion_report.py
  scripts/climb_goal_controller.py
  scripts/surface_keepout_publisher.py
  scripts/wheel_tf_debug.py
  scripts/surface_geometry.py
  scripts/mockup_climb_planner.py
  scripts/bayesian_motion_model.py
  scripts/surface_goal_planner.py
  scripts/horizontal_pipe_planner.py
  scripts/adhesion_scene_rviz.py
  scripts/adhesion_dashboard.py
  scripts/gazebo_model_tf_relay.py
  scripts/surface_attachment_guard.py
  scripts/experimental_polygon_rviz.py
  scripts/experimental_polygon_grid.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_surface_geometry test/test_surface_geometry.py)
endif()

ament_package()
