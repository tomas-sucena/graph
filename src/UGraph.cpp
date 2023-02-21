//
// Created by Tomás Sucena Lopes on 20/02/2023.
//

#include "UGraph.h"

#include <queue>
#include <stack>

/**
 * creates a new undirected Graph
 * @param n number of vertices that the Graph will be initialized with
 */
UGraph::UGraph(int n) : Graph(n) {}

/* METHODS */
/**
 * indicates if the Graph is directed
 * @return 'false' (because this is an undirected Graph)
 */
bool UGraph::isDirected() const{
    return false;
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
bool UGraph::addEdge(int src, int dest, double weight, bool valid){
    if (!Graph::addEdge(src, dest, weight, valid))
        return false;

    return Graph::addEdge(dest, src, weight, valid);
}

/**
 * removes an edge from the Graph, that is, eliminates the connection between two vertices
 * @complexity O(E)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return 'true' if the removal occurs, 'false' otherwise
 */
bool UGraph::removeEdge(int src, int dest){
    if (!Graph::removeEdge(src, dest))
        return false;

    return Graph::removeEdge(dest, src);
}

/**
 * calculates all the connected components of the Graph
 * @complexity O(V + E)
 * @return list with all the connected components (each component is a list of indices)
 */
list<list<int>> UGraph::getConnectedComponents(){
    list<list<int>> connectedComponents;

    reset();
    for (int i = 1; i <= vertices.size(); ++i){
        if (!(*this)[i].valid) continue;

        connectedComponents.push_back(bfs(i));
    }

    return connectedComponents;
}

/**
 * calculates the number of connected components of the Graph
 * @complexity O(V + E)
 * @return number of connected components
 */
int UGraph::countConnectedComponents(){
    return (int) getConnectedComponents().size();
}

list<int> UGraph::getArticulationPoints(){
    list<int> articulationPoints;
    reset();

    // setup
    std::vector<int> order(vertices.size() + 1), low(vertices.size() + 1);

    std::stack<int> s;
    s.push(1);

    uSet<int> onStack;
    onStack.insert(1);



    return articulationPoints;
}

int UGraph::countArticulationPoints(){
    return (int) getArticulationPoints().size();
}

/**
 * computes a Minimum Spanning Tree (MST) of the Graph, using an implementation of Prim's algorithm
 * @complexity O(E * logV)
 * @return list containing the edges that belong to the MST
 */
list<Edge*> UGraph::getMST(){
    list<Edge*> MST;
    reset();

    uSet<int> notInMST;
    for (int i = 1; i <= (int) vertices.size(); ++i)
        notInMST.insert(i);

    (*this)[1].dist = 0;

    std::priority_queue<Vertex, std::vector<Vertex>, std::greater<>> pq;
    pq.push((*this)[1]);

    list<Edge*> prevEdges;

    while (!notInMST.empty()){
        int curr = pq.top().index;
        pq.pop();

        notInMST.erase(curr);
        (*this)[curr].valid = false;

        for (Edge* e : prevEdges){
            if (e->dest != curr) continue;

            MST.push_back(e);
            break;
        }

        for (Edge* e : (*this)[curr].out){
            int next = e->dest;

            if (!e->valid || !(*this)[next].valid) continue;

            if ((*this)[curr].dist + e->weight < (*this)[next].dist)
                (*this)[next].dist = (*this)[curr].dist + e->weight;

            pq.push((*this)[next]);
            prevEdges.push_back(e);
        }
    }

    return MST;
}
