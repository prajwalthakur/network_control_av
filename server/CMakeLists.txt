add_library(server_lib src/server.cpp)
target_include_directories(server_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(server_lib pure_pursuit_lib)