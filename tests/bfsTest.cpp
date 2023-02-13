//
// Created by tosul on 06/02/2023.
//

#include <gtest/gtest.h>

#include "../src/Graph.h"
#include "ExampleGraphs.h"

using std::list;
using testing::Eq;

TEST(bfs, shortest_path){
    // undirected graph
    Graph g1 = ExampleGraphs::graph1();

    list<list<int>> paths = g1.getShortestPaths(1, 1);

    ASSERT_EQ(1, paths.size());
    for (auto it = paths.begin(); it != paths.end(); ++it){
        EXPECT_EQ(1, it->front());
        EXPECT_EQ(1, it->back());
        EXPECT_EQ(1, it->size());
    }

    paths = g1.getShortestPaths(1, 4);

    ASSERT_EQ(2, paths.size());
    for (auto it = paths.begin(); it != paths.end(); ++it){
        EXPECT_EQ(1, it->front());
        EXPECT_EQ(4, it->back());
        EXPECT_EQ(3, it->size());
    }

    paths = g1.getShortestPaths(4, 1);

    ASSERT_EQ(2, paths.size());
    for (auto it = paths.begin(); it != paths.end(); ++it){
        EXPECT_EQ(4, it->front());
        EXPECT_EQ(1, it->back());
        EXPECT_EQ(3, it->size());
    }
}