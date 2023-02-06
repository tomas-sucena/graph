//
// Created by tosul on 05/02/2023.
//

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <list>
#include <vector>

#include "Vertex.h"

class Graph {
    bool directed;
    std::vector<Vertex> vertices;
    std::list<Edge*> edges;

public:
    // constructors
    explicit Graph(bool directed = true, int n = 0);
    explicit Graph(int n, bool directed = true);

    // methods
    bool isDirected() const;
    int numberOfVertices() const;
    int numberOfEdges() const;
    std::vector<Vertex> getVertices() const;

    bool reserve(int num);
    void addVertex(Vertex* v = nullptr);
    int removeVertex(int num);

    bool addEdge(int src, int dest, int weight = 1, bool valid = true);

    int inDegree(int num) const;
    int outDegree(int num) const;
    bool areConnected(int src, int dest) const;
};

#endif //GRAPH_GRAPH_H
