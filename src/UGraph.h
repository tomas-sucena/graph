//
// Created by Tomás Sucena Lopes on 20/02/2023.
//

#ifndef GRAPH_UGRAPH_H
#define GRAPH_UGRAPH_H

#include "Graph.h"

class UGraph : public Graph {
public:
    // constructor
    explicit UGraph(int n = 0);

    // methods
    bool addEdge(int src, int dest, double weight = 1, bool valid = true) override;
    bool removeEdge(int src, int dest) override;

    bool isDirected() const override;

    list<list<int>> getConnectedComponents();
    int countConnectedComponents();

    list<int> getArticulationPoints();
    int countArticulationPoints();
};

#endif //GRAPH_UGRAPH_H
