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

ament_package()
