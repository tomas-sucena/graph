//
// Created by Tomás Sucena Lopes on 05/02/2023.
//

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <list>
#include <set>
#include <vector>

#include "Edge.hpp"
#include "Vertex.hpp"
#include "Path.hpp"

using std::vector;

class Graph {
/* ATTRIBUTES */
    struct AutoResetSettings {
        bool vertexValid = true;
        bool vertexDist = true;
        bool edgeValid = true;
        bool edgeFlow = true;
    };

protected:
    int weighted;
    std::vector<Vertex *> vertices;
    std::set<Edge *> edges;

public:
    bool autoReset;
    AutoResetSettings autoResetSettings;

/* CONSTRUCTORS */
public:
    explicit Graph(int n);

/* METHODS */
protected:
    void resetVertices();
    void resetEdges();
    void resetAll();

    bool validIndex(int index) const;

    // search methods
    std::list<int> bfs(int src);
    std::list<Path> bfs(int src, int dest);
    Path dijkstra(int src, int dest);
    void dijkstra(int src);

    // flow methods
    double edmondsKarp(int src, int sink, std::list<Path> *augPaths = nullptr);

public:
    bool reserve(int n);
    bool resize(int n);
    virtual void addVertex(Vertex *v = nullptr);
    int removeVertex(int index);
    virtual bool addEdge(Edge *e);
    virtual bool addEdge(int src, int dest, double weight = 1, bool valid = true);
    virtual int removeEdges(int src, int dest);

    virtual bool isDirected() const = 0;
    bool isWeighted() const;
    int countVertices() const;
    int countEdges() const;
    vector<Vertex *> getVertices() const;
    std::set<Edge *> getEdges() const;
    virtual vector<vector<double>> toMatrix(bool fillAll = false);
    virtual Vertex &operator[](int index);

    bool areConnected(int src, int dest) const;
    double eccentricity(int index, std::list<int> *farthest = nullptr);
    double diameter(std::list<std::pair<int, int>> *farthest = nullptr);

    double distance(int src, int dest);
    Path getShortestPath(int src, int dest);
    std::list<Path> getShortestPaths(int src, int dest);
    std::list<int> getReachable(int src, double dist, bool weighted = true);

    double maximumFlow(int src, int sink, std::list<Path> *augPaths = nullptr);
};

#endif //GRAPH_GRAPH_H
