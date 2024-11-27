cmake_minimum_required(VERSION 3.29)
project(vsgMrMap)

set(CMAKE_CXX_STANDARD 20)

include_directories(
        ../../../src/chrono_thirdparty/stb
        ../../../src/chrono_thirdparty/cxxopts
)

add_executable(vsgMrMap main.cpp
        stb_implementations.cpp
        readGrayMap.cpp
        readGrayMap.h
)
