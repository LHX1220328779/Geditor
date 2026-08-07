set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED Widgets Concurrent Network)
find_package(PCL REQUIRED )

include_directories(${PROJECT_SOURCE_DIR}/build/src/ui/ui_autogen/include ./ ${PCL_INCLUDE_DIRS})

file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_executable(${PROJECT_NAME}
        ${SRCS}
        res.qrc
)

target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets Qt5::Concurrent Qt5::Network
        ${PROJECT_NAME}_algo
        ${PROJECT_NAME}_core
        ${PROJECT_NAME}_pcd
        ${PROJECT_NAME}_map
        ${PROJECT_NAME}_sys
        ${PROJECT_NAME}_utils
        ${PROJECT_NAME}_coverage
        ${PROJECT_NAME}_render
        ${PROJECT_NAME}_sprinkle
        ${THIRD_PARTY_LIBS}
        ${PCL_IO_LIBRARIES}
        ${PCL_FILTERS_LIBRARY}
)

add_custom_command(TARGET  ${PROJECT_NAME} POST_BUILD
                   COMMAND ${CMAKE_STRIP} "$<TARGET_FILE:${PROJECT_NAME}>"
                   COMMENT "Strip debug symbols done on final binary.")
