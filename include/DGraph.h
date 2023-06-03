//
// Created by Tomás Sucena Lopes on 20/02/2023.
//

#ifndef GRAPH_DGRAPH_H
#define GRAPH_DGRAPH_H

#include <list>
#include <unordered_set>

#include "../src/Graph.h"

#define uSet std::unordered_set

class DGraph : public Graph {
/* CONSTRUCTOR */
public:
    explicit DGraph(int n = 0);

/* METHODS */
protected:
    // search methods
    bool dfs(int src, uSet<int> *seen = nullptr);

public:
    bool isDirected() const override;

    DGraph getSubgraph(std::list<int> vertexIndices);
    bool isDAG();
    std::list<int> topologicalSort();
};

#endif //GRAPH_DGRAPH_H
