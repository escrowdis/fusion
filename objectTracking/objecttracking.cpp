#include "objecttracking.h"

ObjectTracking::ObjectTracking()
{
    ti.reserve(500);
}

ObjectTracking::~ObjectTracking()
{

}

void ObjectTracking::reset()
{

}

void ObjectTracking::track()
{

}

void ObjectTracking::resetObjectTracking(objectTracking &src)
{
    src.fg_update = false;
    src.info.clear();
    src.trajectory.clear();
    src.trajectory_kf.clear();
    src.missed_count = 0;
    src.track_status = TRACK_STATUS::NO_TARGET;
}
