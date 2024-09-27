find_package(Eigen3 CONFIG REQUIRED)

add_executable(test_2 main.cpp)
target_link_libraries(test_2 PRIVATE custom_math_lib)
target_link_libraries(test_2 PRIVATE Eigen3::Eigen)



