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
    friend class Graph;
    friend class DGraph;
    friend class UGraph;

protected:
    int index;
    bool valid;
    double dist;
    std::list<Edge*> out;
    std::list<Edge*> in;

public:
    // constructor
    explicit Vertex(bool valid = true)
        : valid(valid), index(0), dist(INF) {}

    // destructor
    virtual ~Vertex() = default;

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
