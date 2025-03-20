

add_library(pure_pursuit_lib src/client.cpp)
target_include_directories(pure_pursuit_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(Eigen3 REQUIRED)
target_link_libraries(pure_pursuit_lib Eigen3::Eigen)