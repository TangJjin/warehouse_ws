cmake_minimum_required(VERSION 3.8)
project(drone_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_msgs REQUIRED)

# ── k230_tcp_bridge ──────────────────────────────────────
add_executable(k230_tcp_bridge
  src/k230_tcp_bridge.cpp
)

target_compile_features(k230_tcp_bridge PUBLIC cxx_std_17)

target_include_directories(k230_tcp_bridge
  PRIVATE
  include
)

ament_target_dependencies(k230_tcp_bridge
  rclcpp
  std_msgs
  drone_msgs
)
target_link_libraries(k230_tcp_bridge Threads::Threads)

install(TARGETS
  k230_tcp_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/k230_snapshot_decode.sh
  DESTINATION lib/${PROJECT_NAME}
)

# ── qr_vision_node ───────────────────────────────────────
set(RKNN_INCLUDE $ENV{RKNN_INCLUDE})
set(RKNN_LIB $ENV{RKNN_LIB})
set(RKNN_RUNTIME_SO "${RKNN_LIB}/librknnrt.so")

if(RKNN_INCLUDE AND RKNN_LIB AND EXISTS "${RKNN_RUNTIME_SO}")
  find_package(OpenCV REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(image_geometry REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(sensor_msgs REQUIRED)

  add_executable(qr_vision_node
    src/main.cpp
    src/qr_vision_node.cpp
    src/rknn_yolo_detector.cpp
    src/yolo_postprocessor.cpp
    src/depth_processor.cpp
  )

  target_compile_features(qr_vision_node PUBLIC cxx_std_17)

  target_include_directories(qr_vision_node
    PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${RKNN_INCLUDE}
  )

  target_link_libraries(qr_vision_node
    ${OpenCV_LIBS}
    ${RKNN_RUNTIME_SO}
  )

  ament_target_dependencies(qr_vision_node
    cv_bridge
    image_geometry
    message_filters
    rclcpp
    sensor_msgs
  )

  install(TARGETS
    qr_vision_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "Skipping qr_vision_node because RKNN runtime is unavailable. "
    "Set RKNN_INCLUDE and RKNN_LIB so that ${RKNN_RUNTIME_SO} exists to enable it.")
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
