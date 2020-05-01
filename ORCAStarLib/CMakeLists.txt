cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O0" )

add_subdirectory(tinyxml)

add_subdirectory(Src)


