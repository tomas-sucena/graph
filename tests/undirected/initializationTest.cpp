//
// Created by tosul on 05/02/2023.
//

#include <gtest/gtest.h>

#include "../../src/Graph.h"

using testing::Eq;

TEST(initialization, default_constructor){
    Graph g1;

    EXPECT_TRUE(g1.isDirected());
    ASSERT_EQ(g1.countVertices(), g1.getVertices().size());
    EXPECT_EQ(0, g1.countVertices());
}

TEST(initialization, parametrized_constructors){
    Graph g1(false, 5);

    EXPECT_FALSE(g1.isDirected());
    ASSERT_EQ(g1.countVertices(), g1.getVertices().size());
    EXPECT_EQ(5, g1.countVertices());

    Graph g2(5, false);

    EXPECT_FALSE(g2.isDirected());
    ASSERT_EQ(g2.countVertices(), g2.getVertices().size());
    EXPECT_EQ(5, g2.countVertices());
}
