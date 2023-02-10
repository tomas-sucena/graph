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
    int index;
    bool valid;
    int dist;
    std::list<Edge*> adj;

    friend class Graph;

public:
    // constructor
    explicit Vertex(bool valid = true)
        : valid(valid), index(0), dist(INF) {}

    // methods
    bool operator<(const Vertex& rhs) const{
        return dist < rhs.dist;
    }

    bool operator>(const Vertex& rhs) const{
        return dist > rhs.dist;
    }
};

#endif //GRAPH_VERTEX_H
