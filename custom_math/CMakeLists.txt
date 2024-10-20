cmake_minimum_required(VERSION 3.29)
project(AIS4104-assignments)
find_package(Eigen3 CONFIG REQUIRED)


set(CMAKE_CXX_STANDARD 20)

add_library(custom_math_lib custom_math.cpp)
target_link_libraries(custom_math_lib PRIVATE Eigen3::Eigen)

