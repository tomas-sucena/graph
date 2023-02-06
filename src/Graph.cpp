//
// Created by tosul on 05/02/2023.
//

#include "Graph.h"

#include <queue>

/* CONSTRUCTORS */
/**
 * creates a new Graph
 * @param directed bool that indicates if the Graph is directed
 * @param n number of vertices that the Graph will be initialized with
 */
Graph::Graph(bool directed, int n) : directed(directed) {
    if (n <= 0) return;

    vertices.resize(n);
}

/**
 * creates a new Graph
 * @param directed bool that indicates if the Graph is directed
 * @param n number of empty vertices that the Graph will be initialized with
 */
Graph::Graph(int n, bool directed) : Graph(directed, n) {}

/* METHODS */
/**
 * indicates if the Graph is directed
 * @return 'true' if the Graph is directed, 'false' otherwise
 */
bool Graph::isDirected() const{
    return directed;
}

/**
 * returns the number of vertices that the Graph currently has
 * @return number of vertices of the Graph
 */
int Graph::numberOfVertices() const{
    return (int) vertices.size();
}

/**
 * returns the number of edges that the Graph currently has
 * @return number of edges of the Graph
 */
int Graph::numberOfEdges() const{
    return (int) edges.size();
}

/**
 * returns the vector which stores the vertices of the Graph
 * @return std::vector that stores the vertices of the Graph
 */
std::vector<Vertex> Graph::getVertices() const{
    return vertices;
}

/**
 * adds extra empty vertices to the Graph, by resizing the vector which stores them
 * @param num number of empty vertices that will be added to the Graph
 * @return 'true' if the resize occurs, 'false' otherwise
 */
bool Graph::reserve(int num){
    if ((int) vertices.size() >= num) return false;

    vertices.resize(vertices.size() + num);
    return true;
}

/**
 * adds a vertex to the Graph
 * @param v pointer to the vertex that will be added
 */
void Graph::addVertex(Vertex *v){
    if (v == nullptr) v = new Vertex();

    v->num = (int) vertices.size() + 1;
    vertices.push_back(*v);
}

int Graph::removeVertex(int num){
    if (num < 0 || num >= (int) vertices.size()) return -1;

    int deletedEdges = 0;

    // find the vertex
    auto it = vertices.begin();
    for (int i = 0; i < num; i++){
        for (auto edgeIt = it->adj.begin(); edgeIt != it->adj.end();){
            if ((*edgeIt)->dest != num){
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
            if ((*edgeIt)->dest != num){
                ++edgeIt;
                continue;
            }

            edgeIt = it->adj.erase(edgeIt);
            ++deletedEdges;
        }

        it->num--;
    }

    // update the edges
    for (Edge* e : edges){
        if (e->dest < num) continue;

        e->dest--;
    }

    return deletedEdges;
}

bool Graph::addEdge(int src, int dest, int weight, bool valid){
    if (src < 0 || src >= (int) vertices.size() || dest < 0 || dest >= (int) vertices.size())
        return false;

    Edge* e = new Edge(dest, weight, valid);

    vertices[src].adj.push_back(e);
    edges.push_back(e);

    if (!directed){
        Edge* e_ = new Edge(src, weight, valid);

        vertices[dest].adj.push_back(e_);
        edges.push_back(e_);
    }

    return true;
}

/**
 * returns the number of edges in which a vertex is the destination
 * @complexity O(|E|)
 * @param num number of the vertex that is the destination of the edges
 * @return number of edges whose destination is the desired vertex
 */
int Graph::inDegree(int num) const{
    int in = 0;

    for (const Edge* e : edges){
        if (e->dest != num) continue;

        ++in;
    }

    return in;
}

/**
 * returns the number of edges in which a vertex is the source
 * @param num number of the vertex that is the source of the edges
 * @return number of edges whose source is the desired vertex
 */
int Graph::outDegree(int num) const{
    return (int) vertices[num].adj.size();
}

/**
 * verifies if there exists an edge that connects two vertices
 * @param src number of the source vertex
 * @param dest number of the destination vertex
 * @return 'true' if the vertices are connected, 'false' otherwise
 */
bool Graph::areConnected(int src, int dest) const{
    for (const Edge* e : vertices[src].adj){
        if (e->dest != dest) continue;

        return true;
    }

    return false;
}

int Graph::bfs(int a){
    // reset()
    int visitedVertices = 0;

    vertices[a].valid = false;
    vertices[a].dist = 0;

    std::queue<int> q;
    q.push(a);

    while (!q.empty()){
        int curr = q.front();
        q.pop();

        ++visitedVertices;

        for (const Edge* e : vertices[curr].adj){
            int next = e->dest;

            vertices[next].dist = std::min(vertices[curr].dist + e->weight, vertices[next].dist);

            if (!vertices[next].valid || !e->valid) continue;
            vertices[next].valid = false;

            q.push(next);
        }
    }

    return visitedVertices;
}
