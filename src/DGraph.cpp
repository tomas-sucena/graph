//
// Created by Tomás Sucena Lopes on 20/02/2023.
//

#include "DGraph.h"

#include <queue>
#include <stdexcept>
#include <unordered_map>

#define uMap std::unordered_map

/**
 * @brief recursive implementation of the Depth-First Search algorithm, which traverses the graph in search of cycles
 * @complexity O(|V| + |E|)
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

/**
 * @brief implementation of the Edmonds-Karp algorithm, which computes the maximum flow between two vertices
 * @complexity O(|V| * |E|^2)
 * @param src index of the source vertex
 * @param sink index of the sink vertex
 * @return maximum flow
 */
double DGraph::edmondsKarp(int src, int sink){
    double flow = 0;

    while (true){
        std::vector<Edge*> prev(vertices.size() + 1, nullptr);

        // BFS
        std::queue<int> q;
        q.push(src);

        while (!q.empty()){
            int curr = q.front();
            q.pop();

            if (curr == sink) break;

            // edges
            for (Edge* e : (*this)[curr].out){
                int next = e->dest;

                if (prev[next] != nullptr || next == src || e->flow >= e->weight)
                    continue;

                e->valid = true;
                prev[next] = e;

                q.push(next);
            }

            // reverse edges
            for (Edge* e : (*this)[curr].in){
                int next = e->src;

                if (prev[next] != nullptr || next == src || e->flow <= 0)
                    continue;

                e->valid = false;
                prev[next] = e;

                q.push(next);
            }
        }

        if (prev[sink] == nullptr) break;

        // path augmentation
        double df = INF;

        int last = sink;
        for (Edge* e = prev[last]; e != nullptr; e = prev[last]){
            df = std::min(df, e->weight - e->flow);
            last = (e->valid) ? e->src : e->dest;
        }

        last = sink;
        for (Edge* e = prev[last]; e != nullptr; e = prev[last]){
            if (e->valid){
                e->flow += df; last = e->src;
                continue;
            }

            e->flow -= df; last = e->dest;
        }

        flow += df;
    }

    return flow;
}

/* CONSTRUCTOR */
/**
 * @brief creates a new directed Graph
 * @param n number of vertices that the Graph will be initialized with
 */
DGraph::DGraph(int n) : Graph(n) {}

/* METHODS */
/**
 * @brief indicates if the Graph is directed
 * @return 'true' (because this is an directed Graph)
 */
bool DGraph::isDirected() const{
    return true;
}

/**
 * @brief adds an edge to the Graph, that is, a connection between two vertices
 * @complexity O(log|E|)
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
 * @brief removes an edge from the Graph, that is, eliminates the connection between two vertices
 * @complexity O(|E|)
 * @param src index of the source vertex
 * @param dest index of the destination vertex
 * @return 'true' if the removal occurs, 'false' otherwise
 */
bool DGraph::removeEdge(int src, int dest){
    return Graph::removeEdge(src, dest);
}

/**
 * @brief returns a subgraph that only contains specific vertices
 * @complexity O(|V| + |E|)
 * @param vertexIndices list containing indices of the vertices to be included in the subgraph
 * @return subgraph containing only
 */
DGraph DGraph::getSubgraph(list<int> vertexIndices){
    DGraph sub;
    uMap<int, int> newIndices;

    // calculate the new indices
    int currIndex = 1;
    for (auto it = vertexIndices.begin(); it != vertexIndices.end();){
        if (*it <= 0 || *it > (int) vertices.size())
            throw std::invalid_argument("Invalid index!");

        if (newIndices.insert({*it, currIndex}).second){
            ++it; ++currIndex;
            continue;
        }

        it = vertexIndices.erase(it);
    }

    // create the subgraph
    for (int index : vertexIndices){
        Vertex* v = new Vertex(vertices[index - 1]);
        v->index = newIndices[index];

        int i = (int) v->out.size();
        for (auto it = v->out.begin(); i > 0; --i){
            Edge* e = new Edge((*it)); // copy the edge
            it = v->out.erase(it);

            if (newIndices.find(e->dest) == newIndices.end()){
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
        for (auto it = v->in.begin(); i > 0; --i){
            Edge* e = new Edge((*it)); // copy the edge
            it = v->in.erase(it);

            if (newIndices.find(e->src) == newIndices.end()){
                delete e;
                continue;
            }

            e->src = newIndices[e->src];
            e->dest = v->index;

            v->in.push_back(e);
        }

        sub.addVertex(v);
        delete v;
    }

    return sub;
}

/**
 * @brief computes if a Graph is a Directed Acyclic Graph (DAG), by using the Depth-First Search algorithm
 * @complexity O(|V| + |E|)
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
 * @brief computes one of the possible topological orders of the Graph, using Kahn's algorithm
 * @complexity O(|V| + |E|)
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

/**
 * @brief computes the maximum flow between two vertices using an implementation of the Edmonds-Karp algorithm
 * @complexity O(|V| * |E|^2)
 * @param src index of the source vertex
 * @param sink index of the sink vertex
 * @return maximum flow
 */
double DGraph::maximumFlow(int src, int sink){
    if (src <= 0 || src > (int) vertices.size() || sink <= 0 || sink > (int) vertices.size())
        return -1;

    reset();
    return edmondsKarp(src, sink);
}
