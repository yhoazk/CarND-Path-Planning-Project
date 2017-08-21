project(hybrid_astar)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
        hybrid_breadth_first.cpp
        main.cpp)

add_executable(hybrid_astar ${sources})