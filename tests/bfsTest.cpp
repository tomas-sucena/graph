//
// Created by tosul on 06/02/2023.
//

#include <algorithm>
#include <gtest/gtest.h>

#include "../src/Graph.h"
#include "ExampleGraphs.h"

using std::list;
using testing::Eq;

TEST(bfs, shortest_path){
    // undirected graph
    Graph g1 = ExampleGraphs::graph1();

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
    // undirected graphs
    Graph g1 = ExampleGraphs::graph1();
    Graph g2 = ExampleGraphs::graph2();
    Graph g3 = ExampleGraphs::graph3();
    Graph g8 = ExampleGraphs::graph8();
    Graph g9 = ExampleGraphs::graph9();

    EXPECT_TRUE(g1.topologicalSort().empty());
    EXPECT_TRUE(g2.topologicalSort().empty());
    EXPECT_TRUE(g3.topologicalSort().empty());
    EXPECT_TRUE(g8.topologicalSort().empty());
    EXPECT_TRUE(g9.topologicalSort().empty());
    
    // directed graphs with cycles
    Graph g5 = ExampleGraphs::graph5();
    Graph g7 = ExampleGraphs::graph7();
    Graph g11 = ExampleGraphs::graph11();

    EXPECT_TRUE(g5.topologicalSort().empty());
    EXPECT_TRUE(g7.topologicalSort().empty());
    EXPECT_TRUE(g11.topologicalSort().empty());

    // DAGs
    /* TESTES DO PROF PEDRO RIBEIRO */
    Graph g4 = ExampleGraphs::graph4();
    list<int> order = g4.topologicalSort();

    EXPECT_EQ(9, order.size()); // size of order
    for (int v=1; v<=9; v++) EXPECT_NE(order.end(), find(order.begin(), order.end(), v)); // v exists

    vector<pair<int, int>> edges4 = {{1,2},{1,3}, {2,4}, {3,4}, {4,5}, {5,6}, {9,6},{7,5}, {8,7}};
    for (auto e : edges4) {
        auto u = find(order.begin(), order.end(), e.first);
        auto v = find(order.begin(), order.end(), e.second);
        EXPECT_LT(distance(order.begin(), u), distance(order.begin(), v)); // pos(u) < pos(v)
    }


    Graph g6 = ExampleGraphs::graph6();
    order = g6.topologicalSort();

    EXPECT_EQ(8, order.size()); // size of order
    for (int v=1; v<=8; v++) EXPECT_NE(order.end(), find(order.begin(), order.end(), v)); // v exists

    vector<pair<int, int>> edges6 = {{1,2},{3,1}, {2,4}, {5,4}, {5,6}, {6,7}, {6,8}, {8,7}};
    for (auto e : edges6) {
        auto u = find(order.begin(), order.end(), e.first);
        auto v = find(order.begin(), order.end(), e.second);
        EXPECT_LT(distance(order.begin(), u), distance(order.begin(), v)); // pos(u) < pos(v)
    }


    Graph g10 = ExampleGraphs::graph10();
    order = g10.topologicalSort();

    EXPECT_EQ(8, order.size()); // size of order
    for (int v=1; v<=8; v++) EXPECT_NE(order.end(), find(order.begin(), order.end(), v)); // v exists

    vector<pair<int, int>> edges10 = {{2,1},{1,3}, {4,2}, {5,4}, {5,6}, {6,7}, {6,8}, {8,7}};
    for (auto e : edges10) {
        auto u = find(order.begin(), order.end(), e.first);
        auto v = find(order.begin(), order.end(), e.second);
        EXPECT_LT(distance(order.begin(), u), distance(order.begin(), v)); // pos(u) < pos(v)
    }

}

TEST(bfs, reachable){
    list<int> res;

    // undirected and unweighted graph
    Graph g1 = ExampleGraphs::graph1();

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