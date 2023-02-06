//
// Created by tosul on 05/02/2023.
//

#ifndef GRAPH_VERTEX_H
#define GRAPH_VERTEX_H

#include <climits>
#include <list>

#include "Edge.h"
#include "Graph.h"

#define INF INT_MAX

class Vertex {
    int num;
    bool valid;
    int dist;
    std::list<Edge*> adj;

    friend class Graph;

public:
    // constructor
    explicit Vertex(bool valid = true)
        : valid(valid), num(0), dist(INF) {}
};

#endif //GRAPH_VERTEX_H
