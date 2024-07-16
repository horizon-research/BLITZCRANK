#include "../parameter.h"


void Gaussian_prior_factor(
    const float joint_angle_1[DOF],    
    const float joint_velocity_1[DOF],
    const float joint_angle_2[DOF],    
    const float joint_velocity_2[DOF], 
    float error[2 * DOF],
    float Jacobian_1[2 * DOF][DOF], 
    float Jacobian_2[2 * DOF][DOF], 
    float Jacobian_3[2 * DOF][DOF], 
    float Jacobian_4[2 * DOF][DOF], 
    const float delta_t);
