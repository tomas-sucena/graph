//
// Created by Tomás Sucena Lopes on 29/05/2023.
//

#include <gtest/gtest.h>
#include <list>
#include <unordered_set>
#include <vector>

#include "../include/graph/UGraph.h"
#include "TestGraphs.h"

#define uSet std::unordered_set

using testing::Eq;

TEST(MST, Prim) {
    // undirected and unweighted graphs
    UGraph g1 = TestGraphs::graph1();

    std::list<Edge *> MST = g1.getMST();
    std::vector<uSet<int>> res = {{2, 3},
                                  {4},
                                  {1},
                                  {5},
                                  {6, 7},
                                  {9},
                                  {8},
                                  {7},
                                  {6}};

    for (const Edge *e: MST) {
        uSet<int> &edges = res[e->getSrc() - 1];
        EXPECT_NE(edges.find(e->getDest()), edges.end());
    }

    // undirected and weighted graphs
    UGraph g8 = TestGraphs::graph8();

    MST = g8.getMST();
    res = {{2, 3},
           {1},
           {1, 4},
           {3, 5},
           {6, 7},
           {5, 9},
           {5, 8},
           {7},
           {6}};

    for (const Edge *e: MST) {
        uSet<int> &edges = res[e->getSrc() - 1];
        EXPECT_NE(edges.find(e->getDest()), edges.end());
    }
}