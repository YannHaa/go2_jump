cmake_minimum_required(VERSION 3.8)
project(go2_jump_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(go2_jump_core REQUIRED)
find_package(go2_jump_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)

set(MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/mujoco-3.3.6")
find_library(MUJOCO_LIB NAMES mujoco libmujoco.so
  PATHS "${MUJOCO_ROOT}/lib"
  NO_DEFAULT_PATH
)
if(NOT MUJOCO_LIB)
  message(FATAL_ERROR "Could not find MuJoCo library under ${MUJOCO_ROOT}/lib")
endif()

add_library(${PROJECT_NAME} SHARED
  src/unitree_crc.cpp
  src/whole_body_mpc.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE "${MUJOCO_ROOT}/include")
target_link_libraries(${PROJECT_NAME} "${MUJOCO_LIB}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${MUJOCO_ROOT}/lib"
  INSTALL_RPATH "${MUJOCO_ROOT}/lib"
)
ament_target_dependencies(${PROJECT_NAME}
  go2_jump_core
  go2_jump_msgs
  rclcpp
  unitree_go
)

add_executable(whole_body_mpc_node src/whole_body_mpc_node.cpp)
target_compile_features(whole_body_mpc_node PRIVATE cxx_std_17)
target_link_libraries(whole_body_mpc_node ${PROJECT_NAME})
ament_target_dependencies(whole_body_mpc_node
  go2_jump_core
  go2_jump_msgs
  rclcpp
  unitree_go
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS whole_body_mpc_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(go2_jump_core go2_jump_msgs rclcpp unitree_go)

ament_package()
