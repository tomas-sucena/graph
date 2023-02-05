//
// Created by tosul on 05/02/2023.
//

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <vector>

#include "Vertex.h"

class Graph {
    int n;
    bool directed;
    std::vector<Vertex> vertices;

public:
    // constructors
    explicit Graph(bool directed = true, int n = 0);
    explicit Graph(int n, bool directed = true);

    // methods
    bool isDirected() const;
    int numberOfVertices() const;
    std::vector<Vertex> getVertices() const;

    void addVertex(Vertex* v = nullptr);
    int removeVertex(int num);

    bool addEdge(int src, int dest, int weight = 1, bool valid = true);
};

#endif //GRAPH_GRAPH_H
