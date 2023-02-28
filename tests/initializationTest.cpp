//
// Created by Tomás Sucena Lopes on 05/02/2023.
//

#include <gtest/gtest.h>

#include "../src/DGraph.h"
#include "../src/UGraph.h"
#include "TestGraphs.h"

using testing::Eq;

TEST(initialization, default_constructor){
    // undirected graph
    UGraph g1;

    EXPECT_FALSE(g1.isDirected());
    EXPECT_FALSE(g1.isWeighted());
    ASSERT_EQ(g1.countVertices(), g1.getVertices().size());
    EXPECT_EQ(0, g1.countVertices());
    
    // directed graph
    DGraph g2;

    EXPECT_TRUE(g2.isDirected());
    EXPECT_FALSE(g2.isWeighted());
    EXPECT_TRUE(g2.isDAG());
    ASSERT_EQ(g2.countVertices(), g2.getVertices().size());
    EXPECT_EQ(0, g2.countVertices());
}

TEST(initialization, parametrized_constructor){
    // undirected graph
    UGraph g1(5);

    EXPECT_FALSE(g1.isDirected());
    EXPECT_FALSE(g1.isWeighted());
    ASSERT_EQ(g1.countVertices(), g1.getVertices().size());
    EXPECT_EQ(5, g1.countVertices());

    // directed graph
    DGraph g2(5);

    EXPECT_TRUE(g2.isDirected());
    EXPECT_FALSE(g2.isWeighted());
    EXPECT_TRUE(g2.isDAG());
    ASSERT_EQ(g2.countVertices(), g2.getVertices().size());
    EXPECT_EQ(5, g2.countVertices());
}

TEST(initialization, subgraph){
    // directed graph
    DGraph g4 = TestGraphs::graph4();
    EXPECT_EQ(9, g4.countVertices());

    /*DGraph sub2 = g4.getSubgraph({1, 2, 3, 4});
    EXPECT_EQ(4, sub2.countVertices());*/
}
