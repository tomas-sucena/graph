//
// Created by Tomás Sucena Lopes on 20/02/2023.
//

#ifndef GRAPH_DGRAPH_H
#define GRAPH_DGRAPH_H

#include <unordered_set>

#include "implementation/Graph.h"

#define uSet std::unordered_set

class DGraph : public Graph {
    // search methods
    bool dfs(int src, uSet<int>* seen = nullptr);

public:
    // constructor
    explicit DGraph(int n = 0);

    // methods
    bool addEdge(int src, int dest, double weight = 1, bool valid = true) override;
    bool removeEdge(int src, int dest) override;
    bool isDirected() const override;

    DGraph getSubgraph(const list<int>& vertexIndices);
    bool isDAG();
    list<int> topologicalSort();
};

#endif //GRAPH_DGRAPH_H
