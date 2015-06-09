#ifndef COLLISIONAVOIDANCE_H
#define COLLISIONAVOIDANCE_H

#include "../debug_info.h"

#include "AStar.h"
#include "vectorfieldhistogram.h"

class CollisionAvoidance
{
public:
    CollisionAvoidance();

    ~CollisionAvoidance();

    AStar* astar;

    std::vector<std::pair<int, int> > path;
};

#endif // COLLISIONAVOIDANCE_H
