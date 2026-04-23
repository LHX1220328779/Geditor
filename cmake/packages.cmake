list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# openGL
# cmake_policy(SET CMP0072 NEW)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})

# libpng
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})

# glog
find_package(Glog REQUIRED COMPONENTS system)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# thirdparty
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/minilzo/)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sqlite_wrap/src)


set(THIRD_PARTY_LIBS
        ${OPENGL_LIBRARIES}
        GLEW
        ${PNG_LIBRARY}
        ${PROJECT_SOURCE_DIR}/thirdparty/minilzo/build/libminilzo.a
        ${PROJECT_SOURCE_DIR}/thirdparty/sqlite_wrap/build/libsqlite_wrap.a
        ${PROJECT_SOURCE_DIR}/thirdparty/sqlite_wrap/build/libsqlite3.a
        ${CMAKE_DL_LIBS}
        glog gflags pthread
        ${OpenCV_LIBS}
        )