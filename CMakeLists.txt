project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
#set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DUSE_LIBUV ${CMAKE_CXX_FLAGS}")
set(sources src/main.cpp src/spline.h src/path_finder.cpp src/path_finder.h)

link_directories(/usr/local/lib)
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources} src/spline.h)

target_link_libraries(path_planning z ssl uv uWS)
