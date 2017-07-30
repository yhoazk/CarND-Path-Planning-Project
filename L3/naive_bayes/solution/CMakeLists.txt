project(naive_bayes)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
        classifier.cpp
        main.cpp)

add_executable(NBC_PATH ${sources})