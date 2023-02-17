//
// Created by tosul on 05/02/2023.
//

#ifndef GRAPH_VERTEX_H
#define GRAPH_VERTEX_H

#include <limits>
#include <set>

#include "Edge.h"
#include "Graph.h"

#define INF std::numeric_limits<double>::max()

class Vertex {
    int index;
    bool valid;
    double dist;
    std::set<Edge*> out;
    std::set<Edge*> in;

    friend class Graph;

public:
    // constructor
    explicit Vertex(bool valid = true)
        : valid(valid), index(0), dist(INF) {}

    // methods
    bool operator<(const Vertex& rhs) const{
        if (dist == rhs.dist)
            return index < rhs.index;

        return dist < rhs.dist;
    }

    bool operator>(const Vertex& rhs) const{
        if (dist == rhs.dist)
            return index > rhs.index;

        return dist > rhs.dist;
    }
};

#endif //GRAPH_VERTEX_H
