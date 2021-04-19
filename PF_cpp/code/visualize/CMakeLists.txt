set(TARGET_NAME visualize)
add_library(${TARGET_NAME}
	visualize.cpp
    visualize.h
)
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(visualize fssimplewindow load)