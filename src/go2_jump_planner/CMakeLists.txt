cmake_minimum_required(VERSION 3.8)
project(go2_jump_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/jump_plan.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(jump_plan_cli src/jump_plan_cli.cpp)
target_compile_features(jump_plan_cli PRIVATE cxx_std_17)
target_link_libraries(jump_plan_cli ${PROJECT_NAME})

add_executable(jump_target_node src/jump_target_node.cpp)
target_compile_features(jump_target_node PRIVATE cxx_std_17)
target_link_libraries(jump_target_node ${PROJECT_NAME})
ament_target_dependencies(jump_target_node rclcpp std_msgs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS jump_plan_cli jump_target_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
