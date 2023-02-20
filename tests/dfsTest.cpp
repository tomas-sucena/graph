//
// Created by Tomás Sucena Lopes on 12/02/2023.
//

#include <gtest/gtest.h>
#include <set>

#include "../src/DGraph.h"
#include "../src/UGraph.h"
#include "ExampleGraphs.h"

using testing::Eq;

TEST(dfs, isDAG){
    // DAGs
    DGraph g4 = ExampleGraphs::graph4();
    DGraph g6 = ExampleGraphs::graph6();
    DGraph g10 = ExampleGraphs::graph10();

    EXPECT_TRUE(g4.isDAG());
    EXPECT_TRUE(g6.isDAG());
    EXPECT_TRUE(g10.isDAG());

    // directed graphs with cycles
    DGraph g5 = ExampleGraphs::graph5();
    EXPECT_FALSE(g5.isDAG());

    g5.removeEdge(1, 2);
    EXPECT_TRUE(g5.isDAG());

    DGraph g12 = ExampleGraphs::graph12();
    EXPECT_FALSE(g12.isDAG());
}