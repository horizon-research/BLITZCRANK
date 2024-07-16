#pragma once

#include "utility.h"
extern Body_sphere body_sphere[SPHERES_NUM];

void obstacle_factor_planar(
    const float joint_angle[DOF],
    const PlanarSDF sdf,
    float error[SPHERES_NUM],
    float Jacobian[SPHERES_NUM][DOF]);