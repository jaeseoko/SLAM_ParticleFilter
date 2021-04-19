set(TARGET_NAME load)
add_library(${TARGET_NAME}
	load_map.cpp
    load_map.h
    load_log.cpp
    load_log.h
)
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})