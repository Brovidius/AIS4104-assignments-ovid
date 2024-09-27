find_package(Eigen3 CONFIG REQUIRED)

add_executable(test main.cpp
        ../custom_math/custom_math.cpp
        ../custom_math/custom_math.h
)


target_link_libraries(test PRIVATE Eigen3::Eigen)
