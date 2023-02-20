cmake_minimum_required(VERSION 3.23)
project(graph)

set(CMAKE_CXX_STANDARD 14)

add_executable(graph
        src/Edge.h
        src/DGraph.cpp
        src/DGraph.h
        src/Graph.cpp
        src/Graph.h
        src/UGraph.cpp
        src/UGraph.h
        src/Vertex.h
        tests/directed/bfsTest.cpp
        tests/directed/dfsTest.cpp
        tests/directed/dijkstraTest.cpp
        tests/directed/initializationTest.cpp
        tests/directed/manipulationTest.cpp
        tests/ExampleGraphs.cpp
        tests/ExampleGraphs.h
        tests/undirected/bfsTest.cpp
        tests/undirected/dfsTest.cpp
        tests/undirected/dijkstraTest.cpp
        tests/undirected/initializationTest.cpp
        tests/undirected/manipulationTest.cpp)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

target_link_libraries(graph
        gtest
        gtest_main)

enable_testing()
