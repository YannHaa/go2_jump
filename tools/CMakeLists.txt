cmake_minimum_required(VERSION 3.16)
project(go2_jump_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

find_package(Threads REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(verify_rt_lowcmd verify_rt_lowcmd.cpp)
target_link_libraries(verify_rt_lowcmd PRIVATE unitree_sdk2 Threads::Threads)
