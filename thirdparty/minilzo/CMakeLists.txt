cmake_minimum_required(VERSION 2.8)
project(minilzo)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-g -std=c++11 -Wall -Wno-reorder -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -std=c++11 -Wall -Wno-reorder")
add_library(minilzo minilzo.c)

