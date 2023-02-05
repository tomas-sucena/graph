//
// Created by tosul on 05/02/2023.
//

#include "Graph.h"

/* CONSTRUCTORS */
Graph::Graph(bool directed, int n) : directed(directed), n(0) {
    if (n <= 0) return;

    this->n = n;
    vertices.resize(n);
}

Graph::Graph(int n, bool directed) : Graph(directed, n) {}

/* METHODS */
bool Graph::isDirected() const{
    return directed;
}

int Graph::numberOfVertices() const{
    return n;
}

std::vector<Vertex> Graph::getVertices() const{
    return vertices;
}

void Graph::addVertex(Vertex *v){
    if (v == nullptr) v = new Vertex();

    v->num = n++;
    vertices.push_back(*v);
}

int Graph::removeVertex(int num){
    if (num < 0 || num >= n) return -1;

    int deletedEdges = 0;

    // find the vertex
    auto it = vertices.begin();
    for (int i = 0; i < num; i++){
        for (auto edgeIt = it->adj.begin(); edgeIt != it->adj.end();){
            if (edgeIt->dest != num){
                ++edgeIt;
                continue;
            }

            edgeIt = it->adj.erase(edgeIt);
            ++deletedEdges;
        }

        ++it;
    }

    // erase the vertex
    it = vertices.erase(it);

    for (; it != vertices.end(); ++it){
        for (auto edgeIt = it->adj.begin(); edgeIt != it->adj.end();){
            if (edgeIt->dest != num){
                ++edgeIt;
                continue;
            }

            edgeIt = it->adj.erase(edgeIt);
            ++deletedEdges;
        }

        it->num--;
    }

    // update the edges
    for (Vertex& v : vertices){
        for (Edge& e : v.adj){
            if (e.dest < num) continue;

            e.dest--;
        }
    }

    return deletedEdges;;
}

bool Graph::addEdge(int src, int dest, int weight, bool valid){
    if (src < 0 || src >= n || dest < 0 || dest >= n)
        return false;

    vertices[src].adj.push_back(Edge(dest, weight, valid));
    if (!directed) vertices[dest].adj.push_back(Edge(dest, weight, valid));

    return true;
}
