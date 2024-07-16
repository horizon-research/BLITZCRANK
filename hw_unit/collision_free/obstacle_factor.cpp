#include "obstacle_factor.h"



void obstacle_factor_planar(
    float joint_angle[DOF],
    PlanarSDF sdf,
    float error[SPHERES_NUM],
    float Jacobian[SPHERES_NUM][DOF])
{

    Pose3 link_poses[DOF];
    Point3 sph_centers[SPHERES_NUM];

    J_point_conf J_px_jp[SPHERES_NUM];

    sphere_centers(joint_angle, sph_centers, J_px_jp);

    for (int sph_idx = 0; sph_idx < SPHERES_NUM; sph_idx++)
    {
        float total_eps = body_sphere[sph_idx].radius + EPSILON;

        float Jerr_point[2];

        Point2 sph_centers_2d(sph_centers[sph_idx].x, sph_centers[sph_idx].y);
        error[sph_idx] = hinge_loss_obstatle_cost(sph_centers_2d, sdf, total_eps, Jerr_point);

        for (int i = 0; i < DOF; i++)
        {
            Jacobian[sph_idx][i] = Jerr_point[0] * J_px_jp[sph_idx].J[0][i] + Jerr_point[1] * J_px_jp[sph_idx].J[1][i];
        }
    }
}