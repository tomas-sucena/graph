cmake_minimum_required(VERSION 3.23)
project(graph)

set(CMAKE_CXX_STANDARD 14)

add_executable(graph main.cpp src/Graph.cpp src/Graph.h src/Edge.h src/Vertex.h)
