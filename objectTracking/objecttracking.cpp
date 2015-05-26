#include "objecttracking.h"

ObjectTracking::ObjectTracking()
{
    fg_initialized = false;
}

ObjectTracking::~ObjectTracking()
{
    delete[] ti;
}

void ObjectTracking::initialze(int object_size)
{
    if (fg_initialized)
        return;

    obj_size = object_size;

    //**// You may CRASH if the amount of objects is greater than array allocation
    ti = new ObjectTrackingInfo[obj_size];

    fg_initialized = true;
}
