//
// Created by tosul on 05/02/2023.
//

#ifndef GRAPH_EDGE_H
#define GRAPH_EDGE_H

#include "Graph.h"

class Edge {
    int dest;
    int weight;
    bool valid;

    friend class Graph;

    // constructor
    Edge(int dest, int weight, bool valid)
        : dest(dest), weight(weight), valid(valid) {}
};

#endif //GRAPH_EDGE_H
