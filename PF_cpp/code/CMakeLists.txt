cmake_minimum_required(VERSION 3.8)
project(jaeseoko)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_subdirectory(../public/src ${CMAKE_BINARY_DIR}/public) 
add_subdirectory(visualize)
add_subdirectory(load)
add_subdirectory(particleFilter)

add_executable(main MACOSX_BUNDLE main.cpp)
target_link_libraries(main load visualize particleFilter fssimplewindow)
