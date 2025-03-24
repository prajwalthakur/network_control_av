add_library(server_lib src/server.cpp)
target_include_directories(server_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(Eigen3 REQUIRED)
target_link_libraries(server_lib Eigen3::Eigen pure_pursuit_lib)

