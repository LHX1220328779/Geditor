file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_library(${PROJECT_NAME}_core
        ${SRCS}
        loader.h earcut.h)

target_link_libraries(
        ${PROJECT_NAME}_core
        ${PROJECT_NAME}_algo
)