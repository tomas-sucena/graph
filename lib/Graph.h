//
// Created by Tomás Sucena Lopes on 13/06/2023.
//

#ifndef GRAPH_GRAPH_H
#define GRAPH_GRAPH_H

#include <functional>
#include <limits>
#include <list>
#include <set>
#include <unordered_set>
#include <vector>

#define INF std::numeric_limits<double>::max()
#define uSet std::unordered_set

using std::list;
using std::vector;

/********************************************************
   Edge.hpp
 ********************************************************/

class Edge {
    friend class Graph;
    friend class DGraph;
    friend class UGraph;

/* ATTRIBUTES */
protected:
    int src;
    int dest;
    double weight;
    double flow;
    Edge* reverse;

public:
    bool valid;

/* CONSTRUCTORS */
protected:
    Edge(const Edge &e) = default;

public:
    Edge(int src, int dest, double weight, bool valid)
            : src(src), dest(dest), weight(weight), valid(valid), flow(0), reverse(nullptr) {}

/* DESTRUCTOR */
public:
    virtual ~Edge() = default;

/* METHODS */
public:
    /**
     * @brief creates a new pointer with a copy of the current object
     * @return pointer with a copy of the current object
     */
    virtual Edge *clone() const {
        return new Edge(*this);
    }

    int getSrc() const {
        return src;
    }

    int getDest() const {
        return dest;
    }

    double getWeight() const {
        return weight;
    }

    double getFlow() const {
        return flow;
    }

    Edge* getReverse() const {
        return reverse;
    }

    bool operator<(const Edge &e) const {
        if (weight != e.weight)
            return weight < e.weight;

        if (src != e.src)
            return src < e.src;

        return dest < e.dest;
    }
};

namespace std {
    template<>
    struct less<Edge *> {
        bool operator()(const Edge *lhs, const Edge *rhs) const {
            return *lhs < *rhs;
        }
    };
}


/********************************************************
   Vertex.hpp
 ********************************************************/

class Vertex {
    friend class Graph;
    friend class DGraph;
    friend class UGraph;

/* ATTRIBUTES */
protected:
    int index;
    double dist;
    std::list<Edge *> out;
    std::list<Edge *> in;

public:
    bool valid;

/* CONSTRUCTOR */
protected:
    Vertex(const Vertex &v) = default;

public:
    explicit Vertex(bool valid = true)
            : valid(valid), index(0), dist(INF) {}

/* DESTRUCTOR */
public:
    virtual ~Vertex() = default;

/* METHODS */
public:
    /**
     * @brief creates a new pointer with a copy of the current object
     * @return pointer with a copy of the current object
     */
    virtual Vertex *clone() const {
        return new Vertex(*this);
    }

    /**
     * @brief returns the index of the vertex
     * @return index of the vertex
     */
    int getIndex() const {
        return index;
    }

    /**
     * @brief returns the number of ingoing edges of the vertex (i.e. edges whose destination is the vertex)
     * @return number of ingoing edges of the vertex
     */
    int inDegree() const {
        return (int) in.size();
    }

    /**
     * @brief returns the number of outgoing edges of the vertex (i.e. edges whose source is the vertex)
     * @return number of outgoing edges of the vertex
     */
    int outDegree() const {
        return (int) out.size();
    }

    /**
     * @brief returns the ingoing edges of the vertex (i.e. edges whose destination is the vertex)
     * @return list of ingoing edges of the vertex
     */
    std::list<Edge *> inEdges() const {
        return in;
    }

    /**
     * @brief returns the outgoing edges of the vertex (i.e. edges whose source is the vertex)
     * @return list of outgoing edges of the vertex
     */
    std::list<Edge *> outEdges() const {
        return out;
    }

    bool operator<(const Vertex &rhs) const {
        if (dist == rhs.dist)
            return index < rhs.index;

        return dist < rhs.dist;
    }

    bool operator>(const Vertex &rhs) const {
        if (dist == rhs.dist)
            return index > rhs.index;

        return dist > rhs.dist;
    }
};


/********************************************************
   DynamicPQ.hpp
 ********************************************************/

template<typename T>
class DynamicPQ {
    struct PointerComparator {
        bool operator()(const T *lhs, const T *rhs) {
            return *lhs < *rhs;
        }
    };

/* ATTRIBUTES */
private:
    std::set<T *, PointerComparator> queue;
    bool notified;
    typename std::set<T *>::iterator it;

/* CONSTRUCTOR */
public:
    DynamicPQ() : notified(false), it(queue.begin()) {}

/* METHODS */
public:
    bool empty() {
        return queue.empty();
    }

    int size() {
        return (int) queue.size();
    }

    T &find(T &el) {
        return **queue.find(&el);
    }

    bool push(T &el) {
        return queue.insert(&el).second;
    }

    T &peek() {
        return **queue.begin();
    }

    T &pop() {
        T &head = peek();
        queue.erase(queue.begin());

        return head;
    }

    bool notify(T &el) {
        it = queue.find(&el);
        notified = (it != queue.end());

        return notified;
    }

    bool update() {
        if (!notified) return false;

        T *el = *it;
        queue.erase(it);

        notified = false;
        return queue.insert(el).second;
    }
};


/********************************************************
   Path.hpp
 ********************************************************/

class Path {
/* ATTRIBUTES */
private:
    double weight;
    list<const Edge *> edges;
    list<int> indices;

/* CONSTRUCTORS */
public:
    Path() : weight(0) {}

    explicit Path(int src) : weight(0) {
        indices.push_back(src);
    }

/* METHODS */
public:
    double getWeight() const {
        return weight;
    }

    list<int> getIndices() const {
        return indices;
    }

    bool empty() const {
        return (edges.empty() && indices.empty());
    }

    auto begin() const {
        return edges.begin();
    }

    auto end() const {
        return edges.end();
    }

    const Edge *front() const {
        return edges.front();
    }

    const Edge *back() const {
        return edges.back();
    }

    bool push_back(const Edge *e) {
        if (!indices.empty() && e->getSrc() != indices.back()) return false;
        edges.push_back(e);

        if (indices.empty()) indices.push_back(e->getSrc());
        indices.push_back(e->getDest());

        weight += e->getWeight();
        return true;
    }

    bool push_front(const Edge *e) {
        if (!indices.empty() && e->getDest() != indices.front()) return false;
        edges.push_front(e);

        if (indices.empty()) indices.push_front(e->getDest());
        indices.push_front(e->getSrc());

        weight += e->getWeight();
        return true;
    }

    bool operator<(const Path &rhs) const {
        if (indices.empty() || rhs.indices.empty())
            return false;

        bool sameSrc = indices.front() == rhs.indices.front();
        bool sameDest = indices.back() == rhs.indices.back();

        if (!sameSrc || !sameDest) return false;

        return weight < rhs.getWeight();
    }

    bool operator>(const Path &rhs) const {
        if (indices.empty() || rhs.indices.empty())
            return false;

        bool sameSrc = indices.front() == rhs.indices.front();
        bool sameDest = indices.back() == rhs.indices.back();

        if (!sameSrc || !sameDest) return false;

        return weight > rhs.getWeight();
    }

    bool operator<=(const Path &rhs) const {
        if (indices.empty() || rhs.indices.empty())
            return false;

        bool sameSrc = indices.front() == rhs.indices.front();
        bool sameDest = indices.back() == rhs.indices.back();

        if (!sameSrc || !sameDest) return false;

        return weight <= rhs.getWeight();
    }

    bool operator>=(const Path &rhs) const {
        if (indices.empty() || rhs.indices.empty())
            return false;

        bool sameSrc = indices.front() == rhs.indices.front();
        bool sameDest = indices.back() == rhs.indices.back();

        if (!sameSrc || !sameDest) return false;

        return weight >= rhs.getWeight();
    }

    void operator+=(const Path &rhs) {
        if (!indices.empty() && rhs.front()->getSrc() != indices.back()) return;

        for (const Edge* e : rhs)
            push_back(e);
    }
};


/********************************************************
   Graph.h
 ********************************************************/
 
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
    vector<Vertex *> vertices;
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
    list<int> bfs(int src);
    list<Path> bfs(int src, int dest);
    Path dijkstra(int src, int dest);
    void dijkstra(int src);

    // flow methods
    double edmondsKarp(int src, int sink, list<Path> *augPaths = nullptr);

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
    double eccentricity(int index, list<int> *farthest = nullptr);
    double diameter(list<std::pair<int, int>> *farthest = nullptr);

    double distance(int src, int dest);
    Path getShortestPath(int src, int dest);
    list<Path> getShortestPaths(int src, int dest);
    list<int> getReachable(int src, double dist, bool weighted = true);

    double maximumFlow(int src, int sink, list<Path> *augPaths = nullptr);
};


/********************************************************
   DGraph.h
 ********************************************************/

class DGraph : public Graph {
/* CONSTRUCTOR */
public:
    explicit DGraph(int n = 0);

/* METHODS */
protected:
    // search methods
    bool dfs(int src, uSet<int> *seen = nullptr);

public:
    bool isDirected() const override;

    DGraph getSubgraph(list<int> vertexIndices);
    bool isDAG();
    list<int> topologicalSort();
};


/********************************************************
   UGraph.h
 ********************************************************/

class UGraph : public Graph {
/* CONSTRUCTOR */
public:
    explicit UGraph(int n = 0);

/* METHODS */
public:
    bool addEdge(Edge *e) override;
    bool addEdge(int src, int dest, double weight = 1, bool valid = true) override;
    int removeEdges(int src, int dest) override;

    bool isDirected() const override;
    vector<vector<double>> toMatrix(bool fillAll = false) override;
    UGraph getSubgraph(list<int> vertexIndices);

    list<list<int>> getConnectedComponents();
    int countConnectedComponents();

    list<int> getArticulationPoints();
    int countArticulationPoints();

    list<Edge *> getMST(int root = 1);
};

#endif //GRAPH_GRAPH_H
