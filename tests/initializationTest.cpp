//
// Created by tosul on 05/02/2023.
//

#include <gtest/gtest.h>

#include "../src/Graph.h"

using testing::Eq;

TEST(initialization, default_constructor){
    Graph g1;

    EXPECT_TRUE(g1.isDirected());
    ASSERT_EQ(g1.numberOfVertices(), g1.getVertices().size());
    EXPECT_EQ(0, g1.numberOfVertices());
}

TEST(initialization, parametrized_constructors){
    Graph g1(false, 5);

    EXPECT_FALSE(g1.isDirected());
    ASSERT_EQ(g1.numberOfVertices(), g1.getVertices().size());
    EXPECT_EQ(5, g1.numberOfVertices());

    Graph g2(5, false);

    EXPECT_FALSE(g2.isDirected());
    ASSERT_EQ(g2.numberOfVertices(), g2.getVertices().size());
    EXPECT_EQ(5, g2.numberOfVertices());
}

TEST(initialization, vertex_addition){
    int n = 4;
    Graph g1(n);

    ASSERT_EQ(n, g1.numberOfVertices());

    for (int i = 0; i < 4;){
        g1.addVertex();
        ASSERT_EQ(n + ++i, g1.numberOfVertices());
    }
}

TEST(initialization, edge_addition){
    Graph g1(4, true);

    g1.addEdge(0, 1);
    g1.addEdge(0, 2);
    g1.addEdge(1, 3);
    g1.addEdge(2, 3);

    EXPECT_EQ(4, g1.numberOfEdges());

    EXPECT_EQ(2, g1.outDegree(0));
    EXPECT_EQ(1, g1.outDegree(1));
    EXPECT_EQ(1, g1.outDegree(2));
    EXPECT_EQ(0, g1.outDegree(3));
}




