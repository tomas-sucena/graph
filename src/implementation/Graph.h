//
// Created by Tomás Sucena Lopes on 05/02/2023.
//

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <list>
#include <set>
#include <vector>

#include "Edge.hpp"
#include "Path.hpp"
#include "Vertex.hpp"

class Graph {
protected:
    int weighted;
    std::vector<Vertex> vertices;
    std::set<Edge*> edges;

    void reset();
    bool validIndex(int index) const;

    // search methods
    std::list<int> bfs(int src);
    Path dijkstra(int src, int dest);
    std::list<Path> bfs(int src, int dest);

    // flow methods
    double edmondsKarp(int src, int sink);

public:
    // constructors
    explicit Graph(int n);

    // methods
    bool reserve(int num);
    void addVertex(Vertex* v = nullptr);
    int removeVertex(int index);
    virtual bool addEdge(Edge* e);
    virtual bool addEdge(int src, int dest, double weight, bool valid);
    virtual bool removeEdge(int src, int dest);

    virtual bool isDirected() const = 0;
    bool isWeighted() const;
    int countVertices() const;
    int countEdges() const;
    std::vector<Vertex> getVertices() const;
    std::set<Edge*> getEdges() const;
    virtual Vertex& operator[](int index);

    int inDegree(int index) const;
    int outDegree(int index) const;
    bool areConnected(int src, int dest) const;
    double eccentricity(int index, std::list<int>* farthest = nullptr);
    double diameter(std::list<std::pair<int, int>>* farthest = nullptr);

    double distance(int src, int dest);
    Path getShortestPath(int src, int dest);
    std::list<Path> getShortestPaths(int src, int dest);
    std::list<int> getReachable(int src, double dist, bool weighted = true);

    double maximumFlow(int src, int sink);
};

#endif //GRAPH_GRAPH_H
