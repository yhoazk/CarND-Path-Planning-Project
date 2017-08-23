project(poly_solver)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)
INCLUDE_DIRECTORIES(your/header/dir)
set(sources
        main.cpp)

add_executable(hybrid_astar ${sources})
