//
// Created by Tomás Sucena Lopes on 05/02/2023.
//

#include <gtest/gtest.h>
#include <list>
#include <unordered_set>

#include "../src/DGraph.h"
#include "../src/UGraph.h"
#include "TestGraphs.h"

#define uSet std::unordered_set

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
    // undirected graph
    UGraph g2 = TestGraphs::graph2();
    EXPECT_EQ(12, g2.countVertices());
    EXPECT_EQ(2, g2.countConnectedComponents());

    UGraph sub2 = g2.getSubgraph({1, 2, 3, 4, 7, 8, 9});
    EXPECT_EQ(7, sub2.countVertices());
    EXPECT_EQ(1, sub2.countConnectedComponents());

    sub2 = g2.getSubgraph({5, 6, 10, 11, 12});
    EXPECT_EQ(5, sub2.countVertices());
    EXPECT_EQ(1, sub2.countConnectedComponents());

    sub2 = g2.getSubgraph({1, 2, 3, 8, 5, 6, 11, 12});
                                   // 1, 2, 3, 4, 5, 6, 7, 8
    EXPECT_EQ(8, sub2.countVertices());
    EXPECT_EQ(2, sub2.countConnectedComponents());

    std::list<int> indices = {1, 2, 3, 5, 6, 8, 11, 12};
    uSet<int> cc1 = {1, 2, 3, 8}, cc2 = {5, 6, 11, 12};

    for (int i : indices){
        for (int j : indices){
            if (i == j) continue;

            int IinCC1 = (cc1.find(i) != cc1.end());
            int IinCC2 = (cc2.find(i) != cc2.end());
            int JinCC1 = (cc1.find(j) != cc1.end());
            int JinCC2 = (cc2.find(j) != cc2.end());

            if ((IinCC1 && JinCC1) || (IinCC2 && JinCC2)){
                EXPECT_TRUE(sub2.areConnected(i, j));
                continue;
            }

            EXPECT_FALSE(sub2.areConnected(i, j));
        }
    }

    // directed graph
    DGraph g4 = TestGraphs::graph4();
    EXPECT_EQ(9, g4.countVertices());

    DGraph sub4 = g4.getSubgraph({1, 2, 3, 4});
    EXPECT_EQ(4, sub4.countVertices());

    EXPECT_TRUE(sub4.areConnected(1, 2));
    EXPECT_TRUE(sub4.areConnected(1, 3));
    EXPECT_FALSE(sub4.areConnected(1, 4));
    EXPECT_FALSE(sub4.areConnected(2, 3));
}
