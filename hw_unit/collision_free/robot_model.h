#pragma once


#include "point.h"


struct Body_sphere
{
    int link_id;
    float radius;
    Point3 center;

    Body_sphere()
    {
    }

    Body_sphere(int id, float r, Point3 c)
    {
        link_id = id;
        radius = r;
        center = c;
    }
};