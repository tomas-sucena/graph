//
// Created by tosul on 12/02/2023.
//

#include <gtest/gtest.h>
#include <set>

#include "../src/Graph.h"
#include "ExampleGraphs.h"

using testing::Eq;

TEST(dfs, isDAG){
    // DAG
    Graph g4 = ExampleGraphs::graph4();
    Graph g6 = ExampleGraphs::graph6();
    Graph g10 = ExampleGraphs::graph10();

    EXPECT_TRUE(g4.isDAG());
    EXPECT_TRUE(g6.isDAG());
    EXPECT_TRUE(g10.isDAG());

    // directed graphs with cycles
    Graph g5 = ExampleGraphs::graph5();
    EXPECT_FALSE(g5.isDAG());

    g5.removeEdge(1, 2);
    EXPECT_TRUE(g5.isDAG());

    Graph g12 = ExampleGraphs::graph12();
    EXPECT_FALSE(g12.isDAG());

    // undirected graphs
    Graph g1 = ExampleGraphs::graph1();
    Graph g2 = ExampleGraphs::graph2();
    Graph g3 = ExampleGraphs::graph3();

    EXPECT_FALSE(g1.isDAG());
    EXPECT_FALSE(g2.isDAG());
    EXPECT_FALSE(g3.isDAG());
}
