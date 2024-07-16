#include "Gaussian_prior_factor.h"

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
    const float delta_t)
{

    for (int i = 0; i < DOF; i++)
    {
        error[i] = joint_angle_1[i] + joint_velocity_1[i] * delta_t - joint_angle_2[i]; 
        error[i + DOF] = joint_velocity_1[i] - joint_velocity_2[i];                     
    }

    for (int i = 0; i < 2 * DOF; i++)
    {
        for (int j = 0; j < DOF; j++)
        {
            if (i == j)
                Jacobian_1[i][j] = 1;
            else
                Jacobian_1[i][j] = 0;

            if (i == j)
                Jacobian_2[i][j] = delta_t;
            else if (i - DOF == j)
                Jacobian_2[i][j] = 1;
            else
                Jacobian_2[i][j] = 0;

            if (i == j)
                Jacobian_3[i][j] = -1;
            else
                Jacobian_3[i][j] = 0;

            if (i - DOF == j)
                Jacobian_4[i][j] = -1;
            else
                Jacobian_4[i][j] = 0;
        }
    }
}
