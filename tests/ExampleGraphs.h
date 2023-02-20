//
// Created by tosul on 07/02/2023.
//

#ifndef GRAPH_EXAMPLEGRAPHS_H
#define GRAPH_EXAMPLEGRAPHS_H

#include "../src/DGraph.h"
#include "../src/UGraph.h"

using namespace std;

class ExampleGraphs {

public:
    // undirected and unweighted
    static UGraph graph1();
    static UGraph graph2();
    static UGraph graph3();

    // directed and unweighted
    static DGraph graph4();
    static DGraph graph5();
    static DGraph graph6();
    static DGraph graph7();

    // undirected and weighted
    static UGraph graph8();
    static UGraph graph9();

    // directed and weighted
    static DGraph graph10();
    static DGraph graph11();
    static DGraph graph12();
};


#endif //GRAPH_EXAMPLEGRAPHS_H
