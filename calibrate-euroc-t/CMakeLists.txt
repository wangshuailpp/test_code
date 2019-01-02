cmake_minimum_required(VERSION 2.8)
project(untitled)

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 14)

add_executable(untitled
        main.cpp rotation_factor.h)

# 为使用 sophus，您需要使用find_package命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# 与Ceres链接
target_link_libraries( untitled ${CERES_LIBRARIES} ${Sophus_LIBRARIES})
