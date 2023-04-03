//
// Created by Tomás Sucena Lopes on 05/02/2023.
//

#ifndef GRAPH_VERTEX_HPP
#define GRAPH_VERTEX_HPP

#include <limits>
#include <list>

#include "Edge.hpp"

#define INF std::numeric_limits<double>::max()

class Vertex {
    friend class Graph;
    friend class DGraph;
    friend class UGraph;

protected:
    int index;
    bool valid;
    double dist;
    std::list<Edge*> out;
    std::list<Edge*> in;

public:
    // constructor
    explicit Vertex(bool valid = true)
        : valid(valid), index(0), dist(INF) {}

    // destructor
    virtual ~Vertex() = default;

    // methods
    void setValid(bool valid){
        this->valid = valid;
    }

    /**
     * @brief returns the number of ingoing edges of the vertex (i.e. edges whose destination is the vertex)
     * @return number of ingoing edges of the vertex
     */
    int inDegree() const{
        return (int) in.size();
    }

    /**
     * @brief returns the number of outgoing edges of the vertex (i.e. edges whose source is the vertex)
     * @return number of outgoing edges of the vertex
     */
    int outDegree() const{
        return (int) out.size();
    }

    /**
     * @brief returns the ingoing edges of the vertex (i.e. edges whose destination is the vertex)
     * @return list of ingoing edges of the vertex
     */
    std::list<Edge*> inEdges() const{
        return in;
    }

    /**
     * @brief returns the outgoing edges of the vertex (i.e. edges whose source is the vertex)
     * @return list of outgoing edges of the vertex
     */
    std::list<Edge*> outEdges() const{
        return out;
    }

    bool operator<(const Vertex& rhs) const{
        if (dist == rhs.dist)
            return index < rhs.index;

        return dist < rhs.dist;
    }

    bool operator>(const Vertex& rhs) const{
        if (dist == rhs.dist)
            return index > rhs.index;

        return dist > rhs.dist;
    }
};

#endif //GRAPH_VERTEX_HPP
