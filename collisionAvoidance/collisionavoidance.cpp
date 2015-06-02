#include "collisionavoidance.h"

CollisionAvoidance::CollisionAvoidance()
{
    astar = new AStar();
}

CollisionAvoidance::~CollisionAvoidance()
{
    delete astar;
}

