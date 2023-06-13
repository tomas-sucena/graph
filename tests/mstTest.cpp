//
// Created by Tomás Sucena Lopes on 29/05/2023.
//

#include <gtest/gtest.h>
#include <list>
#include <unordered_set>
#include <vector>

#include "../src/UGraph.h"
#include "TestGraphs.h"

#define uSet std::unordered_set

using testing::Eq;

TEST(MST, Prim) {
    /* UNDIRECTED AND UNWEIGHTED GRAPHS */
    UGraph g1 = TestGraphs::graph1();

    std::list<Edge *> MST = g1.getMST();
    std::vector<uSet<int>> res = {{2, 3},
                                  {1, 4},
                                  {1},
                                  {2, 5},
                                  {4, 6, 7},
                                  {5, 9},
                                  {5, 8},
                                  {7},
                                  {6}};

    for (const Edge *e: MST) {
        uSet<int> &edges = res[e->getSrc() - 1];
        EXPECT_NE(edges.find(e->getDest()), edges.end());
    }

    /* UNDIRECTED AND WEIGHTED GRAPHS */
    UGraph g8 = TestGraphs::graph8();

    MST = g8.getMST();
    res = {{2, 3},
           {1},
           {1, 4},
           {3, 5},
           {4, 6, 7},
           {5, 9},
           {5, 8},
           {7},
           {6}};

    for (const Edge *e: MST) {
        uSet<int> &edges = res[e->getSrc() - 1];
        EXPECT_NE(edges.find(e->getDest()), edges.end());
    }

    UGraph g_1(5);

    g_1.addEdge(1, 2, 2);
    g_1.addEdge(1, 4, 5);
    g_1.addEdge(2, 3, 14);
    g_1.addEdge(2, 4, 5);
    g_1.addEdge(2, 5, 4);
    g_1.addEdge(3, 5, 34);
    g_1.addEdge(4, 5, 58);

    MST = g_1.getMST();
    res = {{2, 4},
           {1, 3, 5},
           {2},
           {1},
           {2}};

    for (const Edge *e : MST) {
        uSet<int> &edges = res[e->getSrc() - 1];
        EXPECT_NE(edges.find(e->getDest()), edges.end());
    }

    UGraph g_2(5);

    g_2.addEdge(1, 2, 35);
    g_2.addEdge(1, 3, 40);
    g_2.addEdge(2, 3, 25);
    g_2.addEdge(2, 4, 10);
    g_2.addEdge(3, 4, 20);
    g_2.addEdge(3, 5, 15);
    g_2.addEdge(4, 5, 30);

    MST = g_2.getMST();
    res = {{2},
           {1, 4},
           {4, 5},
           {2, 3},
           {3}};

    for (const Edge *e : MST) {
        uSet<int> &edges = res[e->getSrc() - 1];
        EXPECT_NE(edges.find(e->getDest()), edges.end());
    }

    // example from Wikipedia
    UGraph g_3(4);

    g_3.addEdge(1, 2, 2);
    g_3.addEdge(1, 4, 1);
    g_3.addEdge(2, 4, 2);
    g_3.addEdge(3, 4, 3);

    MST = g_3.getMST();
    res = {{2, 4},
           {1},
           {4},
           {1, 3}};

    for (const Edge *e : MST) {
        uSet<int> &edges = res[e->getSrc() - 1];
        EXPECT_NE(edges.find(e->getDest()), edges.end());
    }

    MST = g_3.getMST(4);
    res = {{4},
           {4},
           {4},
           {1, 2, 3}};

    for (const Edge *e : MST) {
        uSet<int> &edges = res[e->getSrc() - 1];
        EXPECT_NE(edges.find(e->getDest()), edges.end());
    }
}