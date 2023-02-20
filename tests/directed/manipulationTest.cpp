//
// Created by tosul on 06/02/2023.
//

#include <gtest/gtest.h>
#include <set>

#include "../src/Graph.h"

#define VertexPair std::pair<int, int>

using testing::Eq;

TEST(manipulation, vertex_addition){
    int n = 4;
    Graph g1(n);

    ASSERT_EQ(n, g1.countVertices());

    for (int i = 0; i < 4;){
        g1.addVertex();
        ASSERT_EQ(n + ++i, g1.countVertices());
    }
}

TEST(manipulation, edge_addition){
    // directed graph
    Graph g1(4, true);

    g1.addEdge(1, 2);
    g1.addEdge(1, 3);
    g1.addEdge(2, 4);
    g1.addEdge(3, 4);

    EXPECT_EQ(4, g1.countEdges());

    EXPECT_EQ(2, g1.outDegree(1));
    EXPECT_EQ(1, g1.outDegree(2));
    EXPECT_EQ(1, g1.outDegree(3));
    EXPECT_EQ(0, g1.outDegree(4));

    EXPECT_EQ(0, g1.inDegree(1));
    EXPECT_EQ(1, g1.inDegree(2));
    EXPECT_EQ(1, g1.inDegree(3));
    EXPECT_EQ(2, g1.inDegree(4));

    std::set<VertexPair> connected = {{1, 2}, {1, 3}, {2, 4}, {3, 4}};

    for (int i = 1; i <= 4; ++i){
        for (int j = 1; j <= 4; ++j){
            if (connected.find({i, j}) != connected.end()){
                EXPECT_TRUE(g1.areConnected(i, j));
                continue;
            }

            EXPECT_FALSE(g1.areConnected(i, j));
        }
    }

    // undirected graph
    Graph g2(4, false);

    g2.addEdge(1, 2);
    g2.addEdge(1, 3);
    g2.addEdge(2, 4);
    g2.addEdge(3, 4);

    EXPECT_EQ(8, g2.countEdges());

    EXPECT_EQ(2, g2.outDegree(1));
    EXPECT_EQ(2, g2.outDegree(2));
    EXPECT_EQ(2, g2.outDegree(3));
    EXPECT_EQ(2, g2.outDegree(4));

    EXPECT_EQ(2, g2.inDegree(1));
    EXPECT_EQ(2, g2.inDegree(2));
    EXPECT_EQ(2, g2.inDegree(3));
    EXPECT_EQ(2, g2.inDegree(4));

    for (int i = 1; i <= 4; ++i){
        for (int j = 1; j <= 4; ++j){
            bool srcToDest = connected.find({i, j}) != connected.end();
            bool destToSrc = connected.find({j, i}) != connected.end();

            if (srcToDest || destToSrc){
                EXPECT_TRUE(g2.areConnected(i, j));
                continue;
            }

            EXPECT_FALSE(g2.areConnected(i, j));
        }
    }
}

TEST(manipulation, vertex_removal){
    Graph g1(3, false);

    g1.addEdge(1, 2);
    g1.addEdge(2, 3);

    EXPECT_EQ(3, g1.countVertices());
    EXPECT_EQ(4, g1.countEdges());
    EXPECT_EQ(1, g1.countConnectedComponents());
    EXPECT_EQ(1, g1.outDegree(1));
    EXPECT_EQ(1, g1.inDegree(1));
    EXPECT_EQ(2, g1.outDegree(2));
    EXPECT_EQ(2, g1.inDegree(2));
    EXPECT_EQ(1, g1.outDegree(3));
    EXPECT_EQ(1, g1.inDegree(3));

    EXPECT_EQ(4, g1.removeVertex(2));

    EXPECT_EQ(2, g1.countVertices());
    EXPECT_EQ(0, g1.countEdges());
    EXPECT_EQ(2, g1.countConnectedComponents());
    EXPECT_EQ(0, g1.outDegree(1));
    EXPECT_EQ(0, g1.inDegree(1));
    EXPECT_EQ(0, g1.outDegree(2)); // the previous vertex 3 is now vertex 2
    EXPECT_EQ(0, g1.inDegree(2));
    EXPECT_EQ(-1, g1.outDegree(3));
    EXPECT_EQ(-1, g1.inDegree(3));
}
