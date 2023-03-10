//
// Created by Tomás Sucena Lopes on 05/02/2023.
//

#ifndef GRAPH_EDGE_H
#define GRAPH_EDGE_H

class Edge {
    friend class Graph;
    friend class DGraph;
    friend class UGraph;

protected:
    int src;
    int dest;
    double weight;
    bool valid;
    double flow;

public:
    // constructors
    Edge(int src, int dest, double weight, bool valid)
        : src(src), dest(dest), weight(weight), valid(valid), flow(0) {}

    explicit Edge(const Edge* e)
            : src(e->src), dest(e->dest), weight(e->weight), valid(e->valid), flow(e->flow) {}

    // destructor
    virtual ~Edge() = default;

    // methods
    int getSrc() const{
        return src;
    }

    int getDest() const{
        return dest;
    }

    double getFlow() const{
        return flow;
    }

    bool operator<(const Edge& e) const{
        if (weight != e.weight)
            return weight < e.weight;

        if (src != e.src)
            return src < e.src;

        return dest < e.dest;
    }
};

namespace std {
    template <>
    struct less<Edge*> {
        bool operator()(const Edge* lhs, const Edge* rhs) const{
            return *lhs < *rhs;
        }
    };
}

#endif //GRAPH_EDGE_H
