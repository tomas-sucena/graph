//
// Created by Tomás Sucena Lopes on 06/03/2023.
//

#ifndef GRAPH_PATH_H
#define GRAPH_PATH_H

#include <list>

#include "Edge.h"

class Path {
    std::list<const Edge*> edges;
    std::list<int> indices;

public:
    // constructor
    Path() = default;

    explicit Path(int src) {
        indices.push_back(src);
    }

    // methods
    std::list<int> getIndices() const{
        return indices;
    }

    auto begin(){
        return edges.begin();
    }

    auto end(){
        return edges.end();
    }

    const Edge* front(){
        return edges.front();
    }

    const Edge* back(){
        return edges.back();
    }

    bool push_back(const Edge* e){
        if (!indices.empty() && e->getSrc() != indices.back()) return false;
        edges.push_back(e);

        if (indices.empty()) indices.push_back(e->getSrc());
        indices.push_back(e->getDest());

        return true;
    }

    bool push_front(const Edge* e){
        if (!indices.empty() && e->getDest() != indices.front()) return false;
        edges.push_front(e);

        if (indices.empty()) indices.push_front(e->getDest());
        indices.push_front(e->getSrc());

        return true;
    }
};

#endif //GRAPH_PATH_H
