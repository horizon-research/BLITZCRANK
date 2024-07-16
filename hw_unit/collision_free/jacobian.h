#pragma once

#include "../parameter.h"


class J_pose_angle
{
public:
    float J[6][DOF];
};

class J_point_conf
{
public:
    float J[3][DOF];
};