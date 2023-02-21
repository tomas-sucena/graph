cmake_minimum_required(VERSION 3.23)
project(graph)

set(CMAKE_CXX_STANDARD 14)

add_executable(graph
        src/DGraph.cpp
        src/DGraph.h
        src/implementation/Edge.h
        src/implementation/Graph.cpp
        src/implementation/Graph.h
        src/implementation/Vertex.h
        src/UGraph.cpp
        src/UGraph.h
        tests/bfsTest.cpp
        tests/dfsTest.cpp
        tests/dijkstraTest.cpp
        tests/TestGraphs.cpp
        tests/TestGraphs.h
        tests/initializationTest.cpp
        tests/manipulationTest.cpp)

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
