cmake_minimum_required(VERSION 3.20)
project(network_control_av)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# C++ standard
set(CMAKE_CXX_STANDARD 20)  
set(CMAKE_CXX_STANDARD_REQUIRED ON) 


# add the modules
add_subdirectory(client)
add_subdirectory(server)
add_subdirectory(robot)
add_subdirectory(pure_pursuit)
add_subdirectory(simulator)

add_executable(network_control_av main.cpp)
target_link_libraries(

    network_control_av
    client_lib
    server_lib
    robot_lib
    simulator_lib
    pure_pursuit_lib

)
