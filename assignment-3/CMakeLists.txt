find_package(Eigen3 CONFIG REQUIRED)

add_executable(test_3 main.cpp)
target_link_libraries(test_3 PRIVATE custom_math_lib)
target_link_libraries(test_3 PRIVATE Eigen3::Eigen)



