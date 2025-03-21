add_library(robot_lib
    src/robot_base.cpp
    src/robot_abc.cpp
)

# Include headers 

target_include_directories(robot_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
message(STATUS "Include directories: ${CMAKE_CURRENT_SOURCE_DIR}/include")



# link dependencies
find_package(Eigen3 REQUIRED)
target_link_libraries(robot_lib Eigen3::Eigen)