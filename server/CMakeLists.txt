add_library(server_lib src/client.cpp)
target_include_directories(server_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(client_lib pure_pursuit)