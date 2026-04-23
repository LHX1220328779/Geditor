file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
add_library(${PROJECT_NAME}_map ${SRCS})
target_link_libraries(${PROJECT_NAME}_map ${PROJECT_NAME}_core)