//
// Created by Tomás Sucena Lopes on 20/02/2023.
//

#include "../include/UGraph.h"

#include <stack>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "DynamicPQ.hpp"

#define uMap std::unordered_map
#define uSet std::unordered_set

/**
 * creates a new undirected Graph
 * @param n number of vertices that the Graph will be initialized with
 */
UGraph::UGraph(int n) : Graph(n) {}

/* METHODS */
/**
 * @brief adds an edge to the Graph, that is, a connection between two vertices
 * @complexity O(log|E|)
 * @param e edge to be added
 * @return 'true' if the insertion occurs, 'false' otherwise
 */
bool UGraph::addEdge(Edge *e) {
    if (!Graph::addEdge(e))
        return false;

    Edge *e_ = e->clone();
    std::swap(e_->src, e_->dest);

    e->reverse = e_;
    e_->reverse = e;

    return Graph::addEdge(e_);
}

/**
 * adds an edge to the Graph, that is, a connection between two vertices
 * @complexity O(log|E|)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @param weight cost of the connection
 * @param valid bool that indicates if the edge should be considered in Graph traversals
 * @return 'true' if the insertion occurs, 'false' otherwise
 */
bool UGraph::addEdge(int src, int dest, double weight, bool valid) {
    return addEdge(new Edge(src, dest, weight, valid));
}

/**
 * @brief removes all the edges connecting two specific vertices from the Graph
 * @complexity O(|E|)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return number of edges removed
 */
int UGraph::removeEdges(int src, int dest) {
    int removedEdges = Graph::removeEdges(src, dest);
    if (!removedEdges) return 0;

    return removedEdges + Graph::removeEdges(dest, src);
}

/**
 * indicates if the Graph is directed
 * @return 'false' (because this is an undirected Graph)
 */
bool UGraph::isDirected() const {
    return false;
}

/**
 * @brief creates the adjacency matrix of the Graph
 * @complexity O(|V| * |E| * log|V|)
 * @param fillAll indicates if the matrix should be filled with all the distances (which involves computing the
 * shortest paths for all non-adjacent vertices)
 * @return adjacency matrix of the Graph
 */
vector<vector<double>> UGraph::toMatrix(bool fillAll) {
    // initialize the matrix
    vector<vector<double>> matrix(countVertices() + 1);
    for (int i = 1; i <= countVertices(); ++i)
        matrix[i].resize(countVertices() + 1, -1);

    // fill the matrix
    if (!fillAll) {
        for (const Edge *e: edges)
            matrix[e->src][e->dest] = std::min(e->weight, matrix[e->src][e->dest]);

        return matrix;
    }

    for (int i = 1; i <= countVertices(); ++i) {
        dijkstra(i);

        for (int j = i; j <= countVertices(); ++j)
            matrix[i][j] = matrix[j][i] = (*this)[j].dist;
    }

    return matrix;
}

/**
 * @brief returns a subgraph that only contains specific vertices
 * @complexity O(|V| + |E|)
 * @param vertexIndices list containing indices of the vertices to be included in the subgraph
 * @return subgraph containing only specific vertices
 */
UGraph UGraph::getSubgraph(list<int> vertexIndices) {
    UGraph sub;
    uMap<int, int> newIndices;

    // calculate the new indices
    int currIndex = 1;
    for (auto it = vertexIndices.begin(); it != vertexIndices.end();) {
        if (!validIndex(*it))
            throw std::invalid_argument("Invalid index!");

        if (newIndices.insert({*it, currIndex}).second) {
            ++it;
            ++currIndex;
            continue;
        }

        it = vertexIndices.erase(it);
    }

    // create the subgraph
    for (int index: vertexIndices) {
        Vertex *v = vertices[index - 1]->clone(); // copy the vertex
        v->index = newIndices[index];

        int i = (int) v->out.size();
        for (auto it = v->out.begin(); i > 0; --i) {
            Edge *e = (*it)->clone(); // copy the edge
            it = v->out.erase(it);

            if (newIndices.find(e->dest) == newIndices.end()) {
                delete e;
                continue;
            }

            e->src = v->index;
            e->dest = newIndices[e->dest];

            v->out.push_back(e);

            // add the edge to the subgraph
            sub.edges.insert(e);
        }

        i = (int) v->in.size();
        for (auto it = v->in.begin(); i > 0; --i) {
            Edge *e = (*it)->clone(); // copy the edge
            it = v->in.erase(it);

            if (newIndices.find(e->src) == newIndices.end()) {
                delete e;
                continue;
            }

            e->src = newIndices[e->src];
            e->dest = v->index;

            v->in.push_back(e);
        }

        sub.addVertex(v);
    }

    return sub;
}

/**
 * calculates all the connected components of the Graph
 * @complexity O(|V| + |E|)
 * @return list with all the connected components (each component is a list of indices)
 */
list<list<int>> UGraph::getConnectedComponents() {
    list<list<int>> connectedComponents;

    if (autoReset) resetAll();
    for (int i = 1; i <= countVertices(); ++i) {
        if (!(*this)[i].valid) continue;

        connectedComponents.push_back(bfs(i));
    }

    return connectedComponents;
}

/**
 * calculates the number of connected components of the Graph
 * @complexity O(|V| + |E|)
 * @return number of connected components
 */
int UGraph::countConnectedComponents() {
    return (int) getConnectedComponents().size();
}

list<int> UGraph::getArticulationPoints() {
    list<int> articulationPoints;
    if (autoReset) resetAll();

    // setup
    std::vector<int> order(countVertices() + 1), low(countVertices() + 1);

    std::stack<int> s;
    s.push(1);

    uSet<int> onStack;
    onStack.insert(1);


    return articulationPoints;
}

int UGraph::countArticulationPoints() {
    return (int) getArticulationPoints().size();
}

/**
 * computes a Minimum Spanning Tree (MST) of the Graph, using an implementation of Prim's algorithm
 * @complexity O(|E| * log|V|)
 * @param root index of the vertex which will be the root of the MST
 * @return list containing the edges that belong to the MST
 */
list<Edge *> UGraph::getMST(int root) {
    if (autoReset) resetAll();

    DynamicPQ<Vertex> pq;
    uSet<int> notInMST;

    (*this)[root].dist = 0;

    for (int i = 1; i <= countVertices(); ++i) {
        notInMST.insert(i);
        pq.push((*this)[i]);
    }

    std::vector<Edge *> prev(countVertices() + 1, nullptr);

    while (!notInMST.empty()) {
        int curr = pq.pop().index;

        notInMST.erase(curr);
        (*this)[curr].valid = false;

        for (Edge *e: (*this)[curr].out) {
            int next = e->dest;

            if (!e->valid || !(*this)[next].valid || notInMST.find(next) == notInMST.end())
                continue;

            if ((*this)[curr].dist + e->weight >= (*this)[next].dist) continue;

            // notify the PQ that we will alter an element
            pq.notify((*this)[next]);

            (*this)[next].dist = (*this)[curr].dist + e->weight;
            prev[next] = e;

            // update the PQ
            pq.update();
        }
    }

    // build the MST
    list<Edge *> MST;

    for (int i = 1; i <= countVertices(); ++i) {
        if (i == root) continue;

        MST.push_back(prev[i]);
        MST.push_back(prev[i]->reverse);
    }

    return MST;
}
