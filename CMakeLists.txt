cmake_minimum_required(VERSION 3.16.3)
project(graph)

if (${CMAKE_VERSION} VERSION_GREATER "3.23.2")
    cmake_policy(SET CMP0135 NEW)
endif()

set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(PROJECT_HEADERS
        include/DGraph.h
        include/Edge.hpp
        include/Vertex.hpp
        include/UGraph.h
        src/DynamicPQ.hpp
        src/Graph.h
        src/Path.hpp)

set(PROJECT_SOURCES
        src/DGraph.cpp
        src/Graph.cpp
        src/UGraph.cpp)

set(PROJECT_TESTS
        tests/bfsTest.cpp
        tests/dfsTest.cpp
        tests/dijkstraTest.cpp
        tests/dynamicPQTest.cpp
        tests/initializationTest.cpp
        tests/manipulationTest.cpp
        tests/maximumFlowTest.cpp
        tests/mstTest.cpp
        tests/TestGraphs.h
        tests/TestGraphs.cpp)

add_executable(graph
        ${PROJECT_HEADERS}
        ${PROJECT_SOURCES}
        ${PROJECT_TESTS})

target_link_libraries(graph
        gtest
        gtest_main)

enable_testing()
