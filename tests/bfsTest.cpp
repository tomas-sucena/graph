//
// Created by Tomás Sucena Lopes on 06/02/2023.
//

#include <algorithm>
#include <gtest/gtest.h>

#include "../src/DGraph.h"
#include "../src/UGraph.h"
#include "TestGraphs.h"

using std::list;
using testing::Eq;

TEST(bfs, shortest_path){
    // undirected graph
    UGraph g1 = TestGraphs::graph1();

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

TEST(bfs, topological_sort){
    // directed graphs with cycles
    DGraph g5 = TestGraphs::graph5();
    DGraph g7 = TestGraphs::graph7();
    DGraph g11 = TestGraphs::graph11();

    EXPECT_TRUE(g5.topologicalSort().empty());
    EXPECT_TRUE(g7.topologicalSort().empty());
    EXPECT_TRUE(g11.topologicalSort().empty());

    // DAGs
    /* TESTES DO PROF PEDRO RIBEIRO */
    DGraph g4 = TestGraphs::graph4();
    list<int> order = g4.topologicalSort();

    EXPECT_EQ(9, order.size()); // size of order
    for (int v=1; v<=9; v++) EXPECT_NE(order.end(), find(order.begin(), order.end(), v)); // v exists

    vector<pair<int, int>> edges4 = {{1,2},{1,3}, {2,4}, {3,4}, {4,5}, {5,6}, {9,6},{7,5}, {8,7}};
    for (auto e : edges4) {
        auto u = find(order.begin(), order.end(), e.first);
        auto v = find(order.begin(), order.end(), e.second);
        EXPECT_LT(distance(order.begin(), u), distance(order.begin(), v)); // pos(u) < pos(v)
    }


    DGraph g6 = TestGraphs::graph6();
    order = g6.topologicalSort();

    EXPECT_EQ(8, order.size()); // size of order
    for (int v=1; v<=8; v++) EXPECT_NE(order.end(), find(order.begin(), order.end(), v)); // v exists

    vector<pair<int, int>> edges6 = {{1,2},{3,1}, {2,4}, {5,4}, {5,6}, {6,7}, {6,8}, {8,7}};
    for (auto e : edges6) {
        auto u = find(order.begin(), order.end(), e.first);
        auto v = find(order.begin(), order.end(), e.second);
        EXPECT_LT(distance(order.begin(), u), distance(order.begin(), v)); // pos(u) < pos(v)
    }


    DGraph g10 = TestGraphs::graph10();
    order = g10.topologicalSort();

    EXPECT_EQ(8, order.size()); // size of order
    for (int v=1; v<=8; v++) EXPECT_NE(order.end(), find(order.begin(), order.end(), v)); // v exists

    vector<pair<int, int>> edges10 = {{2,1},{1,3}, {4,2}, {5,4}, {5,6}, {6,7}, {6,8}, {8,7}};
    for (auto e : edges10) {
        auto u = find(order.begin(), order.end(), e.first);
        auto v = find(order.begin(), order.end(), e.second);
        EXPECT_LT(distance(order.begin(), u), distance(order.begin(), v)); // pos(u) < pos(v)
    }

    // special case - graph constituted by more than one DAG
    DGraph g(3);

    g.addEdge(1, 2);
    g.addEdge(2, 3);

    std::list<int> res = {1, 2, 3};
    EXPECT_EQ(res, g.topologicalSort());

    g.reserve(3);
    g.addEdge(4, 5);
    g.addEdge(5, 6);

    res = {1, 4, 2, 5, 3, 6};
    EXPECT_EQ(res, g.topologicalSort());

    g.addEdge(2, 4);
    g.addEdge(3, 5);

    res = {1, 2, 3, 4, 5, 6};
    EXPECT_EQ(res, g.topologicalSort());
}

TEST(bfs, reachable){
    list<int> res;

    // undirected and unweighted graph
    UGraph g1 = TestGraphs::graph1();

    for (int i = 1; i <= g1.countVertices(); ++i){
        res = {i};
        EXPECT_EQ(res, g1.getReachable(i, 0));
    }

    res = {1, 2, 3};
    EXPECT_EQ(res, g1.getReachable(1, 1));

    res = {2, 1, 4};
    EXPECT_EQ(res, g1.getReachable(2, 1));

    res = {4, 2, 3, 5};
    EXPECT_EQ(res, g1.getReachable(4, 1));
}
