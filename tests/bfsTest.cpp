//
// Created by tosul on 06/02/2023.
//

#include <gtest/gtest.h>

#include "../src/Graph.h"
#include "ExampleGraphs.h"

using testing::Eq;

TEST(bfs, distance){
    // undirected and undirected graphs
    Graph g1 = ExampleGraphs::graph1();

    EXPECT_EQ(0, g1.distance(1,1));
    EXPECT_EQ(1, g1.distance(1,2));
    EXPECT_EQ(1, g1.distance(1,3));
    EXPECT_EQ(2, g1.distance(1,4));
    EXPECT_EQ(3, g1.distance(1,5));
    EXPECT_EQ(4, g1.distance(1,6));
    EXPECT_EQ(4, g1.distance(1,7));
    EXPECT_EQ(5, g1.distance(1,8));
    EXPECT_EQ(5, g1.distance(1,9));
    EXPECT_EQ(2, g1.distance(4,1));
    EXPECT_EQ(1, g1.distance(4,2));
    EXPECT_EQ(1, g1.distance(4,3));
    EXPECT_EQ(0, g1.distance(4,4));
    EXPECT_EQ(1, g1.distance(4,5));
    EXPECT_EQ(2, g1.distance(4,6));
    EXPECT_EQ(2, g1.distance(4,7));
    EXPECT_EQ(3, g1.distance(4,8));
    EXPECT_EQ(3, g1.distance(4,9));
    EXPECT_EQ(2, g1.distance(2,3));
    EXPECT_EQ(4, g1.distance(9,8));
    EXPECT_EQ(2, g1.distance(8,5));
    EXPECT_EQ(4, g1.distance(2,8));

    std::cout << "  . graph2" << std::endl;
    Graph g2 = ExampleGraphs::graph2();

    EXPECT_EQ(1, g2.distance(1,2));
    EXPECT_EQ(2, g2.distance(1,3));
    EXPECT_EQ(3, g2.distance(1,4));
    EXPECT_EQ(-1, g2.distance(1,5));
    EXPECT_EQ(-1, g2.distance(12,1));
    EXPECT_EQ(2, g2.distance(1,7));
    EXPECT_EQ(1, g2.distance(1,8));
    EXPECT_EQ(2, g2.distance(1,9));
    EXPECT_EQ(3, g2.distance(4,7));
    EXPECT_EQ(2, g2.distance(9,3));
    EXPECT_EQ(2, g2.distance(10,6));
    EXPECT_EQ(1, g2.distance(5,12));

    std::cout << "  . graph3" << std::endl;
    Graph g3 = ExampleGraphs::graph3();

    EXPECT_EQ(4, g3.distance(5,1));
    EXPECT_EQ(3, g3.distance(5,2));
    EXPECT_EQ(2, g3.distance(5,3));
    EXPECT_EQ(1, g3.distance(5,4));
    EXPECT_EQ(4, g3.distance(5,6));
    EXPECT_EQ(4, g3.distance(5,7));
    EXPECT_EQ(3, g3.distance(5,8));
    EXPECT_EQ(2, g3.distance(5,9));
    EXPECT_EQ(1, g3.distance(5,10));
    EXPECT_EQ(3, g3.distance(2,10));
    EXPECT_EQ(2, g3.distance(1,7));
    EXPECT_EQ(4, g3.distance(3,8));

    // directed and unweighted graphs
    Graph g4 = ExampleGraphs::graph4();

    EXPECT_EQ(0, g4.distance(1,1));
    EXPECT_EQ(1, g4.distance(1,2));
    EXPECT_EQ(1, g4.distance(1,3));
    EXPECT_EQ(2, g4.distance(1,4));
    EXPECT_EQ(3, g4.distance(1,5));
    EXPECT_EQ(4, g4.distance(1,6));
    EXPECT_EQ(-1, g4.distance(1,7));
    EXPECT_EQ(-1, g4.distance(1,8));
    EXPECT_EQ(-1, g4.distance(1,9));
    EXPECT_EQ(-1, g4.distance(4,1));
    EXPECT_EQ(-1, g4.distance(4,2));
    EXPECT_EQ(-1, g4.distance(4,3));
    EXPECT_EQ(0, g4.distance(4,4));
    EXPECT_EQ(1, g4.distance(4,5));
    EXPECT_EQ(2, g4.distance(4,6));
    EXPECT_EQ(-1, g4.distance(4,7));
    EXPECT_EQ(-1, g4.distance(4,8));
    EXPECT_EQ(-1, g4.distance(4,9));
    EXPECT_EQ(-1, g4.distance(2,3));
    EXPECT_EQ(-1, g4.distance(9,8));
    EXPECT_EQ(2, g4.distance(8,5));
    EXPECT_EQ(-1, g4.distance(2,8));

    Graph g5 = ExampleGraphs::graph5();

    EXPECT_EQ(0, g5.distance(1,1));
    EXPECT_EQ(1, g5.distance(1,2));
    EXPECT_EQ(3, g5.distance(1,3));
    EXPECT_EQ(2, g5.distance(1,4));
    EXPECT_EQ(3, g5.distance(2,1));
    EXPECT_EQ(0, g5.distance(2,2));
    EXPECT_EQ(2, g5.distance(2,3));
    EXPECT_EQ(1, g5.distance(2,4));
    EXPECT_EQ(1, g5.distance(3,1));
    EXPECT_EQ(2, g5.distance(3,2));
    EXPECT_EQ(0, g5.distance(3,3));
    EXPECT_EQ(3, g5.distance(3,4));
    EXPECT_EQ(2, g5.distance(4,1));
    EXPECT_EQ(3, g5.distance(4,2));
    EXPECT_EQ(1, g5.distance(4,3));
    EXPECT_EQ(0, g5.distance(4,4));

    // undirected and weighted graphs
    Graph g8 = ExampleGraphs::graph8();

    EXPECT_EQ(0, g8.distance(1,1));
    EXPECT_EQ(5, g8.distance(1,2));
    EXPECT_EQ(4, g8.distance(1,3));
    EXPECT_EQ(11, g8.distance(1,4));
    EXPECT_EQ(14, g8.distance(1,5));
    EXPECT_EQ(19, g8.distance(1,6));
    EXPECT_EQ(23, g8.distance(1,7));
    EXPECT_EQ(25, g8.distance(1,8));
    EXPECT_EQ(20, g8.distance(1,9));
    EXPECT_EQ(11, g8.distance(4,1));
    EXPECT_EQ(8, g8.distance(4,2));
    EXPECT_EQ(7, g8.distance(4,3));
    EXPECT_EQ(0, g8.distance(4,4));
    EXPECT_EQ(3, g8.distance(4,5));
    EXPECT_EQ(8, g8.distance(4,6));
    EXPECT_EQ(12, g8.distance(4,7));
    EXPECT_EQ(14, g8.distance(4,8));
    EXPECT_EQ(9, g8.distance(4,9));
    EXPECT_EQ(9, g8.distance(2,3));
    EXPECT_EQ(17, g8.distance(9,8));
    EXPECT_EQ(11, g8.distance(8,5));
    EXPECT_EQ(22, g8.distance(2,8));

    Graph g9 = ExampleGraphs::graph9();

    EXPECT_EQ(0, g9.distance(1,1));
    EXPECT_EQ(3, g9.distance(1,2));
    EXPECT_EQ(4, g9.distance(1,3));
    EXPECT_EQ(6, g9.distance(1,4));
    EXPECT_EQ(3, g9.distance(2,1));
    EXPECT_EQ(0, g9.distance(2,2));
    EXPECT_EQ(7, g9.distance(2,3));
    EXPECT_EQ(3, g9.distance(2,4));
    EXPECT_EQ(4, g9.distance(3,1));
    EXPECT_EQ(7, g9.distance(3,2));
    EXPECT_EQ(0, g9.distance(3,3));
    EXPECT_EQ(4, g9.distance(3,4));
    EXPECT_EQ(6, g9.distance(4,1));
    EXPECT_EQ(3, g9.distance(4,2));
    EXPECT_EQ(4, g9.distance(4,3));
    EXPECT_EQ(0, g9.distance(4,4));
}

TEST(bfs, shortest_paths){
    // undirected graph
    Graph g1 = ExampleGraphs::graph1();

    std::list<Path> paths = g1.getShortestPath(1, 1, false);

    ASSERT_EQ(1, paths.size());
    for (auto it = paths.begin(); it != paths.end(); ++it){
        EXPECT_EQ(1, it->front());
        EXPECT_EQ(1, it->back());
        EXPECT_EQ(1, it->size());
    }

    paths = g1.getShortestPath(1, 4, false);

    ASSERT_EQ(2, paths.size());
    for (auto it = paths.begin(); it != paths.end(); ++it){
        EXPECT_EQ(1, it->front());
        EXPECT_EQ(4, it->back());
        EXPECT_EQ(3, it->size());
    }

    paths = g1.getShortestPath(4, 1, false);

    ASSERT_EQ(2, paths.size());
    for (auto it = paths.begin(); it != paths.end(); ++it){
        EXPECT_EQ(4, it->front());
        EXPECT_EQ(1, it->back());
        EXPECT_EQ(3, it->size());
    }
}