project(behaviour_planner)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)
INCLUDE_DIRECTORIES(your/header/dir)
set(sources
        road.cpp
        vehicle.cpp
        main.cpp)

add_executable(behaviour_planner ${sources})
