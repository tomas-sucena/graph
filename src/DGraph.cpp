//
// Created by Tomás Sucena Lopes on 20/02/2023.
//

#include "DGraph.h"

#include <queue>

/**
 * recursive implementation of the Depth-First Search algorithm, which traverses the graph in search of cycles
 * @complexity O(V + E)
 * @param src index of the vertex where the algorithm will start
 * @param seen set containing the indices of the vertices that have been visited
 * @return 'true' if the a cycle is found, 'false' otherwise
 */
bool DGraph::dfs(int src, uSet<int>* seen){
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

/* CONSTRUCTOR */
/**
 * creates a new directed Graph
 * @param n number of vertices that the Graph will be initialized with
 */
DGraph::DGraph(int n) : Graph(n) {}

/* METHODS */
/**
 * indicates if the Graph is directed
 * @return 'true' (because this is an directed Graph)
 */
bool DGraph::isDirected() const{
    return true;
}

/**
 * adds an edge to the Graph, that is, a connection between two vertices
 * @complexity O(logE)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @param weight cost of the connection
 * @param valid bool that indicates if the edge should be considered in Graph traversals
 * @return 'true' if the insertion occurs, 'false' otherwise
 */
bool DGraph::addEdge(int src, int dest, double weight, bool valid){
    return Graph::addEdge(src, dest, weight, valid);
}

/**
 * removes an edge from the Graph, that is, eliminates the connection between two vertices
 * @complexity O(E)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return 'true' if the removal occurs, 'false' otherwise
 */
bool DGraph::removeEdge(int src, int dest){
    return Graph::removeEdge(src, dest);
}

/**
 * computes if a Graph is a Directed Acyclic Graph (DAG), by using the Depth-First Search algorithm
 * @complexity O(V + E)
 * @return 'true' if the Graph is a DAG, 'false' otherwise
 */
bool DGraph::isDAG(){
    reset();

    for (int i = 1; i <= vertices.size(); ++i){
        if (!(*this)[i].valid) continue;

        bool cycleFound = dfs(i);
        if (cycleFound) return false;
    }

    return true;
}

/**
 * computes one of the possible topological orders of the Graph, using Kahn's algorithm
 * @complexity O(V + E)
 * @return list containing the topologically sorted indices of the vertices
 */
list<int> DGraph::topologicalSort(){
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
