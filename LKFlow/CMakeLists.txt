cmake_minimum_required(VERSION 3.5.1)
project(test1)

find_package(OpenCV 3.1.0 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_DIRS} ${EIGEN3_INCLUDE_DIR})
message(eigen_path:${EIGEN3_INCLUDE_DIR})
message(opencv_path:${OpenCV_DIRS})
message(opencv_libs:${OpenCV_LIBS})

set(CMAKE_CXX_STANDARD 14)

add_executable(test1 main.cpp)

target_link_libraries(test1 ${OpenCV_LIBS})