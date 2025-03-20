
add_library(client_lib src/client.cpp)
target_include_directories(client_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(client_lib robot_lib)