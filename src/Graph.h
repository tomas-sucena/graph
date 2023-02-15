//
// Created by tosul on 05/02/2023.
//

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <list>
#include <vector>
#include <unordered_set>

#include "Vertex.h"

#define INF INT_MAX
#define uSet std::unordered_set

using std::list;

class Graph {
    bool directed;
    std::vector<Vertex> vertices;
    std::list<Edge*> edges;

    // search methods
    bool dfs(int src, uSet<int>* seen = nullptr);
    list<int> bfs(int src);
    list<int> dijkstra(int src, int dest);
    list<list<int>> bfs(int src, int dest);

public:
    // constructors
    explicit Graph(bool directed = true, int n = 0);
    explicit Graph(int n, bool directed = true);

    // methods
    bool reserve(int num);
    void addVertex(Vertex* v = nullptr);
    int removeVertex(int index);
    bool addEdge(int src, int dest, int weight = 1, bool valid = true);
    bool removeEdge(int src, int dest);

    bool isDirected() const;
    int countVertices() const;
    int countEdges() const;
    std::vector<Vertex> getVertices() const;
    Vertex& operator[](int index);

    int inDegree(int index) const;
    int outDegree(int index) const;
    bool areConnected(int src, int dest) const;
    list<list<int>> getConnectedComponents();
    int countConnectedComponents();
    bool isDAG();
    list<int> topologicalSort();

    void reset();

    int distance(int src, int dest);
    list<int> getShortestPath(int src, int dest);
    list<list<int>> getShortestPaths(int src, int dest);
};

#endif //GRAPH_GRAPH_H
