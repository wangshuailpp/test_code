cmake_minimum_required(VERSION 2.8)
project(untitled1)

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

set(CMAKE_CXX_STANDARD 14)

add_executable(untitled1 main.cpp)