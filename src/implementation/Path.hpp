//
// Created by Tomás Sucena Lopes on 06/03/2023.
//

#ifndef GRAPH_PATH_HPP
#define GRAPH_PATH_HPP

#include <list>

#include "Edge.hpp"

class Path {
/* ATTRIBUTES */
private:
    double weight;
    std::list<const Edge *> edges;
    std::list<int> indices;

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

    std::list<int> getIndices() const {
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

#endif //GRAPH_PATH_HPP
