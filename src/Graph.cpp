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

Vertex& Graph::operator[](int index){
    return vertices[index - 1];
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

    v->index = (int) vertices.size() + 1;
    vertices.push_back(*v);
}

int Graph::removeVertex(int index){
    if (index < 0 || index >= (int) vertices.size()) return -1;

    int deletedEdges = 0;

    // find the vertex
    auto it = vertices.begin();
    for (int i = 0; i < index; i++){
        for (auto edgeIt = it->adj.begin(); edgeIt != it->adj.end();){
            if ((*edgeIt)->dest != index){
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
            if ((*edgeIt)->dest != index){
                ++edgeIt;
                continue;
            }

            edgeIt = it->adj.erase(edgeIt);
            ++deletedEdges;
        }

        it->index--;
    }

    // update the edges
    for (Edge* e : edges){
        if (e->dest < index) continue;

        e->dest--;
    }

    return deletedEdges;
}

bool Graph::addEdge(int src, int dest, int weight, bool valid){
    if (src <= 0 || src > (int) vertices.size() || dest <= 0 || dest > (int) vertices.size())
        return false;

    Edge* e = new Edge(dest, weight, valid);

    (*this)[src].adj.push_back(e);
    edges.push_back(e);

    if (!directed){
        Edge* e_ = new Edge(src, weight, valid);

        (*this)[dest].adj.push_back(e_);
        edges.push_back(e_);
    }

    return true;
}

/**
 * returns the number of edges in which a vertex is the destination
 * @complexity O(|E|)
 * @param index index of the vertex that is the destination of the edges
 * @return number of edges whose destination is the desired vertex
 */
int Graph::inDegree(int index) const{
    int in = 0;

    for (const Edge* e : edges){
        if (e->dest != index) continue;

        ++in;
    }

    return in;
}

/**
 * returns the number of edges in which a vertex is the source
 * @param index index of the vertex that is the source of the edges
 * @return number of edges whose source is the desired vertex
 */
int Graph::outDegree(int index) const{
    return (int) vertices[index - 1].adj.size();
}

/**
 * verifies if there exists an edge that connects two vertices
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return 'true' if the vertices are connected, 'false' otherwise
 */
bool Graph::areConnected(int src, int dest) const{
    for (const Edge* e : vertices[src - 1].adj){
        if (e->dest != dest) continue;

        return true;
    }

    return false;
}

/**
 * validates all the vertices and edges
 * @complexity O(|V| + |E|)
 */
void Graph::reset(){
    for (Vertex& v : vertices)
        v.valid = true;

    for (Edge* e : edges)
        e->valid = true;
}

/**
 * implementation of the Breadth-First Search Algorithm, which is a graph traversal algorithm
 * @complexity O(|V| + |E|)
 * @param src index of the vertex where the algorithm will start
 * @return number of visited vertices
 */
int Graph::bfs(int src){
    reset();

    int visitedVertices = 0;

    (*this)[src].valid = false;
    (*this)[src].dist = 0;

    std::queue<int> q;
    q.push(src);

    while (!q.empty()){
        int curr = q.front();
        q.pop();

        ++visitedVertices;

        for (const Edge* e : (*this)[curr].adj){
            int next = e->dest;

            (*this)[next].dist = std::min((*this)[curr].dist + e->weight, (*this)[next].dist);

            if (!(*this)[next].valid || !e->valid) continue;
            (*this)[next].valid = false;

            q.push(next);
        }
    }

    return visitedVertices;
}

/**
 * calculates the minimum distance between two vertices
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return minimum distance between the source and the destination
 */
int Graph::distance(int src, int dest){
    bfs(src);
    return (*this)[dest].dist;
}
