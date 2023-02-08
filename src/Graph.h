//
// Created by tosul on 05/02/2023.
//

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <list>
#include <vector>

#include "Vertex.h"

#define INF INT_MAX
#define Path std::list<int>
#define WPath std::pair<Path, int> // weighted path

class Graph {
    bool directed;
    std::vector<Vertex> vertices;
    std::list<Edge*> edges;

    // search methods
    std::list<int> bfs(int src);

public:
    // constructors
    explicit Graph(bool directed = true, int n = 0);
    explicit Graph(int n, bool directed = true);

    // methods
    bool isDirected() const;
    int countVertices() const;
    int countEdges() const;
    std::vector<Vertex> getVertices() const;
    Vertex& operator[](int index);

    bool reserve(int num);
    void addVertex(Vertex* v = nullptr);
    int removeVertex(int index);

    bool addEdge(int src, int dest, int weight = 1, bool valid = true);

    int inDegree(int index) const;
    int outDegree(int index) const;
    bool areConnected(int src, int dest) const;
    std::list<std::list<int>> getConnectedComponents();
    int countConnectedComponents();

    void reset();

    std::list<Path> unweightedBFS(int src, int dest);
    int distance(int src, int dest);
};

#endif //GRAPH_GRAPH_H
