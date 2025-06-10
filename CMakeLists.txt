cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project("Lidar Obstacle")

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_subdirectory(src)

