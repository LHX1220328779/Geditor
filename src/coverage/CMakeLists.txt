include_directories(${PROJECT_SOURCE_DIR}/src/coverage)
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.c*)
add_library(${PROJECT_NAME}_coverage ${SRCS})