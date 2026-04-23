add_executable(db2pdb db2pdb.cc)

target_link_libraries(db2pdb
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
)
add_custom_command(TARGET  db2pdb POST_BUILD
                   COMMAND ${CMAKE_STRIP} "$<TARGET_FILE:db2pdb>"
                   COMMENT "Strip debug symbols done on final binary.")
                   
find_package(PCL REQUIRED )
add_executable(pcd2pdb pcd2pdb.cc)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(pcd2pdb
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
)
add_custom_command(TARGET  pcd2pdb POST_BUILD
                   COMMAND ${CMAKE_STRIP} "$<TARGET_FILE:pcd2pdb>"
                   COMMENT "Strip debug symbols done on final binary.")

include_directories(/usr/include/eigen3)
# include_directories(/opt/ros/$ENV{ROS_DISTRO}/include /usr/include/eigen3)
# link_directories(/opt/ros/$ENV{ROS_DISTRO}/lib)
# add_executable(vdb2osm vdb2osm.cc)
# target_link_libraries(vdb2osm
#         ${PROJECT_NAME}_algo
#         ${PROJECT_NAME}_core
#         ${PROJECT_NAME}_pcd
#         ${PROJECT_NAME}_map
#         ${PROJECT_NAME}_sys
#         ${PROJECT_NAME}_utils
#         ${PROJECT_NAME}_coverage
#         ${PROJECT_NAME}_render
#         ${PROJECT_NAME}_sprinkle
#         ${THIRD_PARTY_LIBS}
#         lanelet2_core
#         lanelet2_io
#         lanelet2_projection
# )
# add_custom_command(TARGET  vdb2osm POST_BUILD
#                    COMMAND ${CMAKE_STRIP} "$<TARGET_FILE:vdb2osm>"
#                    COMMENT "Strip debug symbols done on final binary.")

add_executable(vdb2conch vdb2conch.cc)
target_link_libraries(vdb2conch
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
)
add_custom_command(TARGET  vdb2conch POST_BUILD
                   COMMAND ${CMAKE_STRIP} "$<TARGET_FILE:vdb2conch>"
                   COMMENT "Strip debug symbols done on final binary.")

find_package(Protobuf REQUIRED)
file(GLOB protobuf_files
  common_msgs/basic_msgs/*.proto
	common_msgs/map_msgs/*.proto
	common_msgs/localization_msgs/*.proto
	common_msgs/routing_msgs/*.proto
)
SET(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})
LIST(APPEND PROTO_FLAGS -I${CMAKE_CURRENT_SOURCE_DIR})
set(PROTO_SRCS  )
set(PROTO_HDRS  )
FOREACH(FIL ${protobuf_files})
	GET_FILENAME_COMPONENT(FIL_WE ${FIL} NAME_WE)
	string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})
	string(REGEX REPLACE "(.+)\\${FILE_NAME}.*" "\\1" FILE_PATH ${FIL})
	# string(REGEX MATCH "(modules/map/proto/|modules/map/relative_map/proto/|modules/perception/proto/|modules/common/proto/|modules/localization/proto/)" OUT_PATH ${FILE_PATH})
	string(REGEX MATCH "(common_msgs/basic_msgs/|common_msgs/localization_msgs/|common_msgs/map_msgs/|common_msgs/routing_msgs/)" OUT_PATH ${FILE_PATH})
	EXECUTE_PROCESS(
		COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS} --cpp_out=${PROTO_META_BASE_DIR} ${FIL}
                COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS} --python_out=${PROTO_META_BASE_DIR} ${FIL}
	)
	if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}${FIL_WE}.pb.cc)
		LIST(APPEND PROTO_SRCS ${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}${FIL_WE}.pb.cc)
		LIST(APPEND PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}${FIL_WE}.pb.h)
	endif ()
ENDFOREACH()

include_directories(
	# ${Boost_INCLUDE_DIRS}
	# ${PROJECT_SOURCE_DIR}
	# ${PROJECT_SOURCE_DIR}/install/include
	${Protobuf_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
	# ${EIGEN3_INCLUDE_DIR}
	# ${OpenCV_INCLUDE_DIRS}
)
set(SOURCES
    vdb2pb.cc
    ${PROTO_SRCS}
)
# 收集所有头文件（可选，用于IDE显示）
set(HEADERS
    ${PROTO_HDRS}
)
add_executable (vdb2pb ${SOURCES} ${HEADERS})
target_link_libraries(vdb2pb
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
        ${Protobuf_LIBRARIES}
)
set(SOURCES
    vdb2dynamic.cc
    ${PROTO_SRCS}
)
set(HEADERS
    ${PROTO_HDRS}
)
add_executable (vdb2dynamic ${SOURCES} ${HEADERS})
target_link_libraries(vdb2dynamic
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
        ${Protobuf_LIBRARIES}
)