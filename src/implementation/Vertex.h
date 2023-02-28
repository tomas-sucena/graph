//
// Created by Tomás Sucena Lopes on 05/02/2023.
//

#ifndef GRAPH_VERTEX_H
#define GRAPH_VERTEX_H

#include <limits>
#include <list>

#include "Edge.h"

#define INF std::numeric_limits<double>::max()

class Vertex {
    int index;
    bool valid;
    double dist;
    std::list<Edge*> out;
    std::list<Edge*> in;

    friend class Graph;
    friend class DGraph;
    friend class UGraph;

public:
    // constructors
    explicit Vertex(bool valid = true)
        : valid(valid), index(0), dist(INF) {}

    Vertex(const Vertex& v) : valid(v.valid), index(0), dist(v.dist) {
        // copy the ingoing edges
        for (const Edge* e : v.in)
            in.push_back(new Edge(e));

        // copy the outgoing edges
        for (const Edge* e : v.out)
            out.push_back(new Edge(e));
    }

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
