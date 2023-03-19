//
// Created by Tomás Sucena Lopes on 20/02/2023.
//

#ifndef GRAPH_DGRAPH_H
#define GRAPH_DGRAPH_H

#include <list>
#include <unordered_set>

#include "implementation/Graph.h"

#define uSet std::unordered_set

class DGraph : public Graph {
    // search methods
    bool dfs(int src, uSet<int>* seen = nullptr);

    // flow methods
    double edmondsKarp(int src, int sink);

public:
    // constructor
    explicit DGraph(int n = 0);

    // methods
    bool addEdge(Edge* e) override;
    bool addEdge(int src, int dest, double weight = 1, bool valid = true) override;
    bool removeEdge(int src, int dest) override;
    bool isDirected() const override;

    DGraph getSubgraph(std::list<int> vertexIndices);
    bool isDAG();
    std::list<int> topologicalSort();
    double maximumFlow(int src, int sink);
};

#endif //GRAPH_DGRAPH_H
