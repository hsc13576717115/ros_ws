cmake_minimum_required(VERSION 3.8)
project(r2_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(moveit_ros_planning_interface QUIET)
if(NOT moveit_ros_planning_interface_FOUND AND DEFINED ENV{ROS_DISTRO})
  set(_r2_moveit_config_dir
    "/opt/ros/$ENV{ROS_DISTRO}/share/moveit_ros_planning_interface/cmake")
  if(EXISTS "${_r2_moveit_config_dir}/moveit_ros_planning_interfaceConfig.cmake")
    set(moveit_ros_planning_interface_DIR "${_r2_moveit_config_dir}")
    find_package(moveit_ros_planning_interface QUIET)
  endif()
endif()
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

pkg_check_modules(LIBSERIAL REQUIRED IMPORTED_TARGET libserial)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmCartesianTarget.msg"
  "msg/ArmMotionState.msg"
  "srv/MoveToXZ.srv"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_ik_solver
  src/ik_solver.cpp
)
target_include_directories(${PROJECT_NAME}_ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_damiao_driver SHARED
  src/damiao.cpp
)
target_include_directories(${PROJECT_NAME}_damiao_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_damiao_driver
  PkgConfig::LIBSERIAL
)

add_library(${PROJECT_NAME}_dm_hw SHARED
  src/dm_hw.cpp
)
target_include_directories(${PROJECT_NAME}_dm_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_dm_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)
target_link_libraries(${PROJECT_NAME}_dm_hw
  ${PROJECT_NAME}_damiao_driver
)

add_executable(arm_command_server_node
  src/arm_command_server_node.cpp
)
target_include_directories(arm_command_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_command_server_node
  rclcpp
  geometry_msgs
)
target_link_libraries(arm_command_server_node
  ${PROJECT_NAME}_ik_solver
  ${cpp_typesupport_target}
)
add_dependencies(arm_command_server_node ${PROJECT_NAME})
if(moveit_ros_planning_interface_FOUND)
  target_compile_definitions(arm_command_server_node PRIVATE R2_ARM_CONTROL_HAS_MOVEIT=1)
  ament_target_dependencies(arm_command_server_node
    moveit_ros_planning_interface
  )
endif()
get_target_property(
  arm_command_server_include_directories
  ${cpp_typesupport_target}
  INTERFACE_INCLUDE_DIRECTORIES)
if(arm_command_server_include_directories)
  target_include_directories(arm_command_server_node PUBLIC
    ${arm_command_server_include_directories}
  )
endif()

add_executable(ik_commander_node
  src/ik_commander_node.cpp
)
target_include_directories(ik_commander_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ik_commander_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(ik_commander_node
  ${PROJECT_NAME}_ik_solver
)

add_executable(arm_pose_test_node
  src/arm_pose_test_node.cpp
)
target_include_directories(arm_pose_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_pose_test_node
  rclcpp
  sensor_msgs
)
target_link_libraries(arm_pose_test_node
  ${PROJECT_NAME}_ik_solver
)

add_executable(ee_state_publisher_node
  src/ee_state_publisher_node.cpp
)
target_include_directories(ee_state_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ee_state_publisher_node
  geometry_msgs
  rclcpp
  sensor_msgs
)
target_link_libraries(ee_state_publisher_node
  ${PROJECT_NAME}_ik_solver
)

add_executable(joint_state_timestamp_bridge_node
  src/joint_state_timestamp_bridge_node.cpp
)
target_include_directories(joint_state_timestamp_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joint_state_timestamp_bridge_node
  rclcpp
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface dm_hw_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES dm_hw_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dpad_gpio_toggle_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}_ik_solver
    ${PROJECT_NAME}_damiao_driver
    ${PROJECT_NAME}_dm_hw
    arm_command_server_node
    ik_commander_node
    arm_pose_test_node
    ee_state_publisher_node
    joint_state_timestamp_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ik_solver
    test/test_ik_solver.cpp
  )
  if(TARGET test_ik_solver)
    target_link_libraries(test_ik_solver
      ${PROJECT_NAME}_ik_solver
    )
    target_include_directories(test_ik_solver PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
  endif()

  ament_add_gtest(test_damiao_limits
    test/test_damiao_limits.cpp
  )
  if(TARGET test_damiao_limits)
    target_link_libraries(test_damiao_limits
      ${PROJECT_NAME}_damiao_driver
    )
    target_include_directories(test_damiao_limits PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
