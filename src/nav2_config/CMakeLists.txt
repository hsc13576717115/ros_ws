cmake_minimum_required(VERSION 3.5)
project(nav2_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config rviz maps launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/send_goal.py
    scripts/cmd_vel_to_move_cmd.py
    scripts/cmd_vel_visualizer.py
    scripts/wait_for_odom_activate_nav2.py
    scripts/preset_waypoint_mission.py
    scripts/print_current_pose.py
    scripts/run_dog_runtime_logger.py
    scripts/lock_cpu_performance.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
