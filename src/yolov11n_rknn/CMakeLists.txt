cmake_minimum_required(VERSION 3.8)
project(yolov11n_rknn)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# 查找依赖
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)  # 用于获取包路径
find_package(OpenCV 4 REQUIRED)

# RKNN API 路径（使用本地 RKNN SDK）
set(RKNN_API_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include" CACHE PATH "RKNN API include directory")
set(RKNN_RUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64" CACHE PATH "RKNN runtime library directory")

# 包含目录
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${RKNN_API_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# 链接目录
link_directories(${RKNN_RUNTIME_DIR})

# =============================================================================
# 目标：yolo_detection_node
# =============================================================================
add_executable(yolo_detection_node
  src/yolo_detection_node.cpp
  src/yolo_detector.cpp
)

ament_target_dependencies(yolo_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
  cv_bridge
  ament_index_cpp
)

target_link_libraries(yolo_detection_node
  ${OpenCV_LIBS}
  rknn_api
)

# =============================================================================
# 安装
# =============================================================================
install(TARGETS
  yolo_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件、配置文件和模型文件
install(DIRECTORY
  launch
  config
  models
  data
  DESTINATION share/${PROJECT_NAME}/
)

# 安装头文件
install(DIRECTORY
  include/
  DESTINATION include/
)

# 安装 RKNN 运行时库
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so
  DESTINATION lib/
)

# 设置 RPATH，让可执行文件能找到本地 RKNN 库
set_target_properties(yolo_detection_node PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# =============================================================================
# 测试
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
