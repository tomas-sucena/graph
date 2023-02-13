//
// Created by tosul on 07/02/2023.
// Graphs by Pedro Ribeiro (DCC/FCUP) [last update: 27/11/2022]
//

#include "ExampleGraphs.h"

Graph ExampleGraphs::graph1() {
    Graph g(9, false);
    g.addEdge(1,2);
    g.addEdge(1,3);
    g.addEdge(2,4);
    g.addEdge(3,4);
    g.addEdge(4,5);
    g.addEdge(5,6);
    g.addEdge(5,7);
    g.addEdge(6,9);
    g.addEdge(7,8);
    return g;
}

Graph ExampleGraphs::graph2() {
    Graph g(12, false);
    g.addEdge(1,2);
    g.addEdge(2,3);
    g.addEdge(3,4);
    g.addEdge(1,8);
    g.addEdge(2,8);
    g.addEdge(3,8);
    g.addEdge(7,8);
    g.addEdge(8,9);
    g.addEdge(10,11);
    g.addEdge(11,12);
    g.addEdge(11,5);
    g.addEdge(11,6);
    g.addEdge(12,5);
    g.addEdge(12,6);
    g.addEdge(5,6);
    return g;
}

Graph ExampleGraphs::graph3() {
    Graph g(10, false);
    g.addEdge(1,2);
    g.addEdge(2,3);
    g.addEdge(3,4);
    g.addEdge(4,5);
    g.addEdge(1,6);
    g.addEdge(5,10);
    g.addEdge(2,6);
    g.addEdge(4,10);
    g.addEdge(6,7);
    g.addEdge(7,8);
    g.addEdge(8,9);
    g.addEdge(9,10);
    return g;
}

Graph ExampleGraphs::graph4() {
    Graph g(9, true);
    g.addEdge(1,2);
    g.addEdge(1,3);
    g.addEdge(2,4);
    g.addEdge(3,4);
    g.addEdge(4,5);
    g.addEdge(5,6);
    g.addEdge(9,6);
    g.addEdge(7,5);
    g.addEdge(8,7);
    return g;
}

Graph ExampleGraphs::graph5() {
    Graph g(4, true);
    g.addEdge(1,2);
    g.addEdge(2,4);
    g.addEdge(4,3);
    g.addEdge(3,1);
    return g;
}

Graph ExampleGraphs::graph6() {
    Graph g(8, true);
    g.addEdge(1,2);
    g.addEdge(2,4);
    g.addEdge(3,1);
    g.addEdge(5,4);
    g.addEdge(5,6);
    g.addEdge(6,7);
    g.addEdge(6,8);
    g.addEdge(8,7);
    return g;
}

Graph ExampleGraphs::graph7() {
    Graph g(3, true);
    g.addEdge(1,2);
    g.addEdge(2,1);
    g.addEdge(1,3);
    g.addEdge(3,1);
    g.addEdge(2,3);
    g.addEdge(3,2);
    return g;
}

Graph ExampleGraphs::graph8() {
    Graph g(9, false);
    g.addEdge(1,2, 5);
    g.addEdge(1,3, 4);
    g.addEdge(2,4, 8);
    g.addEdge(3,4, 7);
    g.addEdge(4,5, 3);
    g.addEdge(5,6, 5);
    g.addEdge(5,7, 9);
    g.addEdge(6,9, 1);
    g.addEdge(7,8, 2);
    g.addEdge(8,9, 42);
    return g;
}

Graph ExampleGraphs::graph9() {
    Graph g(4, false);
    g.addEdge(1,2, 3);
    g.addEdge(1,3, 4);
    g.addEdge(2,4, 3);
    g.addEdge(3,4, 4);
    return g;
}

Graph ExampleGraphs::graph10() {
    Graph g(8, true);
    g.addEdge(1,3, 3);
    g.addEdge(2,1, 2);
    g.addEdge(4,2, 1);
    g.addEdge(5,4, 1);
    g.addEdge(5,6, 2);
    g.addEdge(6,8, 1);
    g.addEdge(6,7, 2);
    g.addEdge(8,7, 3);
    return g;
}

Graph ExampleGraphs::graph11() {
    Graph g(3, true);
    g.addEdge(2,1, 2);
    g.addEdge(1,3, 8);
    g.addEdge(3,2, 1);
    return g;
}

Graph ExampleGraphs::graph12(){
    Graph g(6, true);
    g.addEdge(1, 2, 3);
    g.addEdge(2, 3, 1);
    g.addEdge(3, 1, 4);
    g.addEdge(3, 4, 1);
    g.addEdge(4, 5, 5);
    g.addEdge(5, 6, 9);
    g.addEdge(6, 4, 2);
    return g;
}