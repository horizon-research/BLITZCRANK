#pragma once


class Point2
{
public:
    float x;
    float y;
    // float z;

    Point2()
    {
    }

    Point2(float x1, float y1) : x(x1), y(y1) {}

    Point2 &operator=(Point2 &p)
    {

        Point2 point;
        point.x = p.x;
        point.y = p.y;

        return point;
    }
};

class Point3
{
public:
    float x;
    float y;
    float z;

    Point3()
    {
    }

    Point3(float x1, float y1, float z1) : x(x1), y(y1), z(z1) {}

    Point3 &operator=(Point3 &p)
    {

        Point3 point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;

        return point;
    }
};