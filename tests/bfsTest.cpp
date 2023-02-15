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
    // non DAGs
    Graph g1 = ExampleGraphs::graph1();
    Graph g2 = ExampleGraphs::graph2();
    Graph g3 = ExampleGraphs::graph3();
    Graph g5 = ExampleGraphs::graph5();
    Graph g7 = ExampleGraphs::graph7();
    Graph g8 = ExampleGraphs::graph8();
    Graph g9 = ExampleGraphs::graph9();

    EXPECT_TRUE(g1.topologicalSort().empty());
    EXPECT_TRUE(g2.topologicalSort().empty());
    EXPECT_TRUE(g3.topologicalSort().empty());
    EXPECT_TRUE(g5.topologicalSort().empty());
    EXPECT_TRUE(g7.topologicalSort().empty());
    EXPECT_TRUE(g8.topologicalSort().empty());
    EXPECT_TRUE(g9.topologicalSort().empty());

    // DAGs
    /* TESTES DO PROF PEDRO RIBEIRO */
    cout << "Testando 'topologicalSort'" << endl;

    cout << "  . graph4" << endl;
    Graph g4 = ExampleGraphs::graph4();
    list<int> order = g4.topologicalSort();
    // Uncomment to print order
    // cout << "    Order:"; for (auto v : order ) cout << " " << v; cout << endl;
    // A naive exhaustive verification
    EXPECT_EQ(9, order.size()); // size of order
    for (int v=1; v<=9; v++) EXPECT_NE(order.end(), find(order.begin(), order.end(), v)); // v exists
    // "Ugly", but to keep the graph class as simple as possible and as it was given so as not to confuse students
    vector<pair<int, int>> edges4 = {{1,2},{1,3}, {2,4}, {3,4}, {4,5}, {5,6}, {9,6},{7,5}, {8,7}};
    for (auto e : edges4) {
        auto u = find(order.begin(), order.end(), e.first);
        auto v = find(order.begin(), order.end(), e.second);
        EXPECT_LT(distance(order.begin(), u), distance(order.begin(), v)); // pos(u) < pos(v)
    }

    cout << "  . graph6" << endl;
    Graph g6 = ExampleGraphs::graph6();
    order = g6.topologicalSort();
    // Uncomment to print order
    // cout << "    Order:"; for (auto v : order ) cout << " " << v; cout << endl;
    // A naive exhaustive verification
    EXPECT_EQ(8, order.size()); // size of order
    for (int v=1; v<=8; v++) EXPECT_NE(order.end(), find(order.begin(), order.end(), v)); // v exists
    // "Ugly", but to keep the graph class as simple as possible and as it was given so as not to confuse students
    vector<pair<int, int>> edges6 = {{1,2},{3,1}, {2,4}, {5,4}, {5,6}, {6,7}, {6,8}, {8,7}};
    for (auto e : edges6) {
        auto u = find(order.begin(), order.end(), e.first);
        auto v = find(order.begin(), order.end(), e.second);
        EXPECT_LT(distance(order.begin(), u), distance(order.begin(), v)); // pos(u) < pos(v)
    }

    cout << "  . graph10" << endl;
    Graph g10 = ExampleGraphs::graph10();
    order = g10.topologicalSort();
    // Uncomment to print order
    // cout << "    Order:"; for (auto v : order ) cout << " " << v; cout << endl;
    // A naive exhaustive verification
    EXPECT_EQ(8, order.size()); // size of order
    for (int v=1; v<=8; v++) EXPECT_NE(order.end(), find(order.begin(), order.end(), v)); // v exists
    // "Ugly", but to keep the graph class as simple as possible and as it was given so as not to confuse students
    vector<pair<int, int>> edges10 = {{2,1},{1,3}, {4,2}, {5,4}, {5,6}, {6,7}, {6,8}, {8,7}};
    for (auto e : edges10) {
        auto u = find(order.begin(), order.end(), e.first);
        auto v = find(order.begin(), order.end(), e.second);
        EXPECT_LT(distance(order.begin(), u), distance(order.begin(), v)); // pos(u) < pos(v)
    }

}