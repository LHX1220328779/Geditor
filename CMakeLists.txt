CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(geditor)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-g -std=c++17 -Wall -Wno-reorder -fPIC -Werror=return-type")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -std=c++17 -Wall -Wno-reorder -s")
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++17 -Wall -Wno-reorder")

set(CMAKE_STRIP "strip")
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
include(cmake/packages.cmake)

include_directories(src)
include_directories(thirdparty)
add_subdirectory(src)
