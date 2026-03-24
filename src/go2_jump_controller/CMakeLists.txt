cmake_minimum_required(VERSION 3.8)
project(go2_jump_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(go2_jump_planner REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)

add_library(${PROJECT_NAME}_crc STATIC
  src/unitree_crc.cpp
)
target_compile_features(${PROJECT_NAME}_crc PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_crc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_crc unitree_go)

add_executable(jump_controller_node src/jump_controller_node.cpp)
target_compile_features(jump_controller_node PRIVATE cxx_std_17)
target_include_directories(jump_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(jump_controller_node
  ${PROJECT_NAME}_crc
)
ament_target_dependencies(jump_controller_node
  go2_jump_planner
  rclcpp
  std_msgs
  unitree_go
)

install(
  TARGETS ${PROJECT_NAME}_crc jump_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
