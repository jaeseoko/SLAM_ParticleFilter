set(TARGET_NAME particleFilter)
add_library(${TARGET_NAME}
    particleFilter.cpp
    particleFilter.h
)
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(particleFilter load)