//
// Created by tosul on 05/02/2023.
//

#include "Graph.h"

#include <algorithm>
#include <queue>

/**
 * recursive implementation of the Depth-First Search algorithm, which traverses the graph in search of cycles
 * @complexity O(|V| + |E|)
 * @param src index of the vertex where the algorithm will start
 * @param seen set containing the indices of the vertices that have been visited
 * @return 'true' if the a cycle is found, 'false' otherwise
 */
bool Graph::dfs(int src, uSet<int>* seen){
    if (seen == nullptr){
        seen = new uSet<int>();
        seen->insert(src);
    }

    for (const Edge* e : (*this)[src].out){
        int next = e->dest;
        if (!e->valid || !(*this)[next].valid) continue;

        if (!seen->insert(next).second) return true;
        if (dfs(next, seen)) return true;
    }
    (*this)[src].valid = false;

    return false;
}

/**
 * implementation of the Breadth-First Search algorithm, which traverses the graph
 * @complexity O(|V| + |E|)
 * @param src index of the vertex where the algorithm will start
 * @return list containing the indices of all the visited vertices
 */
list<int> Graph::bfs(int src){
    list<int> visitedVertices;

    (*this)[src].valid = false;
    (*this)[src].dist = 0;

    std::queue<int> q;
    q.push(src);

    while (!q.empty()){
        int curr = q.front();
        visitedVertices.push_back(curr);

        for (const Edge* e : (*this)[curr].out){
            int next = e->dest;

            (*this)[next].dist = std::min((*this)[curr].dist + e->weight, (*this)[next].dist);

            if (!(*this)[next].valid || !e->valid) continue;
            (*this)[next].valid = false;

            q.push(next);
        }

        q.pop();
    }

    return visitedVertices;
}

/**
 * implementation of the Breadth-First Search algorithm, which finds ALL the shortest paths between two vertices in a Graph
 * @complexity O(|V| + |E|)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return list containing all the shortest paths that unite the two vertices
 */
list<list<int>> Graph::bfs(int src, int dest){
    list<list<int>> allPaths = {{src}};

    (*this)[src].valid = false;
    (*this)[src].dist = 0;

    std::queue<int> q;
    q.push(src);

    while (!q.empty()){
        int curr = q.front();
        if (curr == dest) break; // destination reached

        for (const Edge* e : (*this)[curr].out){
            int next = e->dest;
            list<int> path = allPaths.front();

            (*this)[next].dist = std::min((*this)[curr].dist + e->weight, (*this)[next].dist);

            path.push_back(next);
            allPaths.push_back(path);

            if (!(*this)[next].valid || !e->valid) continue;
            (*this)[next].valid = false;

            q.push(next);
        }

        q.pop();
        allPaths.pop_front();
    }

    // eliminate the paths that don't end in the destination
    for (auto it = allPaths.begin(); it != allPaths.end();){
        if (it->back() == dest){
            ++it;
            continue;
        }

        it = allPaths.erase(it);
    }

    return allPaths;
}

/**
 * implementation of the Dijkstra algorithm, which finds the shortest path between two vertices in a Graph
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return list containing the indices of the visited vertices that comprise the shortest path
 */
list<int> Graph::dijkstra(int src, int dest){
    (*this)[src].valid = false;
    (*this)[src].dist = 0;

    std::priority_queue<Vertex, std::vector<Vertex>, std::greater<>> pq;
    pq.push((*this)[src]);

    std::vector<int> prev(vertices.size() + 1, -1);
    prev[src] = src;

    while (!pq.empty()){
        int curr = pq.top().index;
        pq.pop();

        if (curr == dest) break;

        for (const Edge* e : (*this)[curr].out){
            int next = e->dest;

            if ((*this)[curr].dist + e->weight < (*this)[next].dist){
                (*this)[next].dist = (*this)[curr].dist + e->weight;
                prev[next] = curr;
            }

            if (!(*this)[next].valid || !e->valid) continue;
            (*this)[next].valid = false;

            pq.push((*this)[next]);
        }
    }

    // reconstruct the shortest path
    list<int> path;

    for (int last = dest; prev[dest] > 0; last = prev[last]){
        path.push_front(last);

        if (last == prev[last]) break;
    }

    return path;
}

/* CONSTRUCTORS */
/**
 * creates a new Graph
 * @param directed bool that indicates if the Graph is directed
 * @param n number of vertices that the Graph will be initialized with
 */
Graph::Graph(bool directed, int n) : directed(directed) {
    if (n <= 0) return;

    vertices.resize(n);
    for (int i = 1; i <= n; ++i)
        (*this)[i].index = i;
}

/**
 * creates a new Graph
 * @param directed bool that indicates if the Graph is directed
 * @param n number of empty vertices that the Graph will be initialized with
 */
Graph::Graph(int n, bool directed) : Graph(directed, n) {}

/* METHODS */
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

/**
 * removes a vertex from the Graph
 * @complexity O(|V| + |E|)
 * @param index index of the vertex that will be removed
 * @return number of edges that were removed (those whose destination was the deleted vertex)
 */
int Graph::removeVertex(int index){
    if (index <= 0 || index > (int) vertices.size()) return -1;

    uSet<int> affectedVertices;

    // update the edges
    int deletedEdges = 0;

    for (auto it = edges.begin(); it != edges.end();){
        bool srcRemoved = ((*it)->src == index);
        bool destRemoved = ((*it)->dest == index);

        if (destRemoved)
            affectedVertices.insert((*it)->src);

        if ((*it)->dest > index)
            (*it)->dest--;

        if ((*it)->src > index)
            (*it)->src--;

        if (!srcRemoved && !destRemoved){
            ++it;
            continue;
        }

        (*it)->dest = -1;
        it = edges.erase(it);
        ++deletedEdges;
    }

    // update the vertices
    for (int i = 1; i <= vertices.size(); ++i){
        if (i > index)
            (*this)[i].index = i - 1;

        if (affectedVertices.find(i) == affectedVertices.end()) continue;

        Vertex& v = (*this)[i];
        for (auto it = v.out.begin(); it != v.out.end();){
            if ((*it)->dest > 0){
                ++it;
                continue;
            }

            delete (*it);
            it = v.out.erase(it);
        }

        for (auto it = v.in.begin(); it != v.in.end();){
            if ((*it)->dest > 0){
                ++it;
                continue;
            }

            delete (*it);
            it = v.in.erase(it);
        }
    }

    // remove the vertex
    vertices.erase(vertices.begin() + index - 1);

    return deletedEdges;
}

/**
 * adds an edge to the Graph, that is, a connection between two vertices
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @param weight cost of the connection
 * @param valid bool that indicates if the edge should be considered in Graph traversals
 * @return 'true' if the insertion occurs, 'false' otherwise
 */
bool Graph::addEdge(int src, int dest, int weight, bool valid){
    if (src <= 0 || src > (int) vertices.size() || dest <= 0 || dest > (int) vertices.size())
        return false;

    Edge* e = new Edge(src, dest, weight, valid);

    (*this)[src].out.push_back(e);
    (*this)[dest].in.push_back(e);
    edges.push_back(e);

    if (!directed){
        Edge* e_ = new Edge(dest, src, weight, valid);

        (*this)[dest].out.push_back(e_);
        (*this)[src].in.push_back(e_);
        edges.push_back(e_);
    }

    return true;
}

/**
 * removes an edge from the Graph, that is, eliminates the connection between two vertices
 * @complexity O(|E|)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return 'true' if the removal occurs, 'false' otherwise
 */
bool Graph::removeEdge(int src, int dest){
    if (!areConnected(src, dest))
        return false;

    for (auto it = edges.begin(); it != edges.end();){
        if ((*it)->src != src || (*it)->dest != dest){
            ++it;
            continue;
        }

        it = edges.erase(it);
    }

    // remove the edge from the outgoing edges list of the source vertex
    for (auto it = (*this)[src].out.begin(); it != (*this)[src].out.end();){
        if ((*it)->src != src || (*it)->dest != dest){
            ++it;
            continue;
        }

        it = (*this)[src].out.erase(it);
    }

    // remove the edge from the ingoing edges list of the destination vertex
    for (auto it = (*this)[dest].in.begin(); it != (*this)[dest].in.end();){
        if ((*it)->src != src || (*it)->dest != dest){
            ++it;
            continue;
        }

        it = (*this)[dest].in.erase(it);
    }

    return true;
}

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
int Graph::countVertices() const{
    return (int) vertices.size();
}

/**
 * returns the number of edges that the Graph currently has
 * @return number of edges of the Graph
 */
int Graph::countEdges() const{
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
 * returns the number of edges in which a vertex is the destination
 * @param index index of the vertex that is the destination of the edges
 * @return number of edges whose destination is the desired vertex (or -1 if the index is invalid)
 */
int Graph::inDegree(int index) const{
    if (index <= 0 || index > (int) vertices.size()) return -1;
    return (int) vertices[index - 1].in.size();
}

/**
 * returns the number of edges in which a vertex is the source
 * @param index index of the vertex that is the source of the edges
 * @return number of edges whose source is the desired vertex (or -1 if the index is invalid)
 */
int Graph::outDegree(int index) const{
    if (index <= 0 || index > (int) vertices.size()) return -1;
    return (int) vertices[index - 1].out.size();
}

/**
 * verifies if there exists an edge that connects two vertices
 * @complexity O(|E|)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return 'true' if the vertices are connected, 'false' otherwise
 */
bool Graph::areConnected(int src, int dest) const{
    if (src <= 0 || src > (int) vertices.size() || dest <= 0 || dest > (int) vertices.size())
        return false;

    for (const Edge* e : vertices[src - 1].out){
        if (e->dest != dest) continue;

        return true;
    }

    return false;
}

/**
 * calculates all the connected components of the Graph<br>
 * <strong>NOTE:</strong> only applicable in undirected graphs
 * @complexity O(|V| + |E|)
 * @return list with all the connected components (each component is a list of indices)
 */
list<list<int>> Graph::getConnectedComponents(){
    list<list<int>> connectedComponents;
    if (directed) return connectedComponents;

    reset();
    for (int i = 1; i <= vertices.size(); ++i){
        if (!(*this)[i].valid) continue;

        connectedComponents.push_back(bfs(i));
    }

    return connectedComponents;
}

/**
 * calculates the number of connected components of the Graph<br>
 * <strong>NOTE:</strong> only applicable in undirected graphs
 * @complexity O(|V| + |E|)
 * @return number of connected components
 */
int Graph::countConnectedComponents(){
    return (int) getConnectedComponents().size();
}

/**
 * computes if a Graph is a Directed Acyclic Graph (DAG), by using the Depth-First Search algorithm
 * @complexity O(|V| + |E|)
 * @return 'true' if the Graph is a DAG, 'false' otherwise
 */
bool Graph::isDAG(){
    if (!directed) return false;
    reset();

    for (int i = 1; i <= vertices.size(); ++i){
        if (!(*this)[i].valid) continue;

        bool cycleFound = dfs(i);
        if (cycleFound) return false;
    }

    return true;
}

/**
 * computes one of the possible topological orders of the Graph, using Kahn's algorithm<br>
 * <strong>NOTE:</strong> only applicable in DAGs
 * @complexity O(|V| + |E|)
 * @return list containing the topologically sorted indices of the vertices
 */
list<int> Graph::topologicalSort(){
    list<int> res;
    if (!isDAG()) return res;

    reset();
    std::vector<int> in_degrees(vertices.size() + 1);

    std::queue<int> q;
    for (int i = 1; i <= (int) vertices.size(); ++i){
        if ((in_degrees[i] = inDegree(i)) > 0) continue;

        q.push(i);
    }

    while (!q.empty()){
        int curr = q.front();
        q.pop();

        for (const Edge* e : (*this)[curr].out){
            int next = e->dest;
            if (!e->valid || !(*this)[next].valid) continue;

            if (!--in_degrees[next]) q.push(next);
        }

        res.push_back(curr);
    }

    return res;
}

/**
 * validates all the vertices and edges
 * @complexity O(|V| + |E|)
 */
void Graph::reset(){
    for (Vertex& v : vertices){
        v.valid = true;
        v.dist = INF;
    }

    for (Edge* e : edges)
        e->valid = true;
}

/**
 * calculates the minimum distance between two vertices
 * @complexity O(|E| * log|V|)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return minimum distance between the source and the destination if they are connected, -1 otherwise
 */
double Graph::distance(int src, int dest){
    reset();
    dijkstra(src, dest);

    double res = (*this)[dest].dist;
    res = (res == INF) ? -1 : res;

    return res;
}

/**
 * calculates (one of) the shortest path between two vertices
 * @complexity O(|E| * log|V|)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return list containing the indices of the vertices that form the path
 */
list<int> Graph::getShortestPath(int src, int dest){
    reset();
    return dijkstra(src, dest);
}

/**
 * calculates ALL the shortest paths between two vertices
 * @complexity O(|V| + |E|)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return list containing the shortest paths (each path is represented by the indices of the vertices that form it)
 */
list<list<int>> Graph::getShortestPaths(int src, int dest){
    reset();
    return bfs(src, dest);
}

/**
 * calculates which vertices can be reached in a given distance from a source vertex, using an implementation of the
 * Breadth-First Search algorithm
 * @complexity O(|V| + |E|)
 * @param src index of the source vertex
 * @param dist distance
 * @param weighted bool that specifies if the given distance is referring to the weight of the edges ('true') or the
 * total number of edges ('false')
 * @return list containing the indices of the reachable vertices
 */
list<int> Graph::getReachable(int src, double dist, bool weighted){
    reset();
    list<int> reachableVertices;

    (*this)[src].valid = false;
    (*this)[src].dist = 0;

    std::queue<std::pair<int, double>> q;
    q.push({src, dist});

    while (!q.empty()){
        int currIndex = q.front().first;
        double currDistance = q.front().second;
        q.pop();

        if (currDistance < 0) continue;
        reachableVertices.push_back(currIndex);

        for (const Edge* e : (*this)[currIndex].out){
            int nextIndex = e->dest;
            double nextDistance = currDistance - (weighted ? e->weight : 1);

            (*this)[nextIndex].dist = std::min((*this)[currIndex].dist + e->weight, (*this)[nextIndex].dist);

            if (!e->valid || !(*this)[nextIndex].valid) continue;
            (*this)[nextIndex].valid = false;

            q.push({nextIndex, nextDistance});
        }
    }

    return reachableVertices;
}
