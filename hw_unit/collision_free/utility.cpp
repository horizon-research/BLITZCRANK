#include "utility.h"

Pose3 link_trans_notheta[DOF];
Pose3 home_pose;
Body_sphere body_sphere[SPHERES_NUM];

void compute_link_trans_notheta()
{
    for (int i = 0; i < DOF; i++)
    {
        link_trans_notheta[i].T[0][0] = 1;
        link_trans_notheta[i].T[0][1] = 0;
        link_trans_notheta[i].T[0][2] = 0;
        link_trans_notheta[i].T[0][3] = a[i];

        link_trans_notheta[i].T[1][0] = 0;
        link_trans_notheta[i].T[1][1] = hls::cos(alpha[i]);
        link_trans_notheta[i].T[1][2] = -hls::sin(alpha[i]);
        link_trans_notheta[i].T[1][3] = 0;

        link_trans_notheta[i].T[2][0] = 0;
        link_trans_notheta[i].T[2][1] = hls::sin(alpha[i]);
        link_trans_notheta[i].T[2][2] = hls::cos(alpha[i]);
        link_trans_notheta[i].T[2][3] = d[i];

        link_trans_notheta[i].T[3][0] = 0;
        link_trans_notheta[i].T[3][1] = 0;
        link_trans_notheta[i].T[3][2] = 0;
        link_trans_notheta[i].T[3][3] = 1;
    }
}

void getH(const int i, const float theta, Pose3 &H)
{
    const float T_theta[4][4] = {hls::cos(theta), -hls::sin(theta), 0, 0, hls::sin(theta), hls::cos(theta), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 4, 4, 4, 4, 4, 4, float, float>(T_theta, link_trans_notheta[i].T, H.T);
}

void getdH(const int i, const float theta, Pose3 &dH)
{
    const float dRot[4][4] = {-hls::sin(theta), -hls::cos(theta), 0, 0, hls::cos(theta), -hls::sin(theta), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 4, 4, 4, 4, 4, 4, float, float>(dRot, link_trans_notheta[i].T, dH.T);
}

void forward_kinematics(float joint_angle[DOF], Pose3 link_pose[DOF], J_pose_angle Jacobian[DOF]) 
{
    Pose3 H[DOF];
    Pose3 Ho[DOF + 1];
    Pose3 dH[DOF];
    Pose3 Hoinv[DOF + 1];

    Ho[0] = home_pose;
    Hoinv[0] = Ho[0].inverse();

    for (int i = 1; i <= DOF; i++)
    {
        getH(i - 1, joint_angle[i - 1], H[i - 1]);
        hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 4, 4, 4, 4, 4, 4, float, float>(Ho[i - 1].T, H[i - 1].T, Ho[i].T);

        getdH(i - 1, joint_angle[i - 1], dH[i - 1]);
        Hoinv[i] = Ho[i].inverse();
    }

    Pose3 dHo_dq[DOF][DOF];
    for (int i = 0; i < DOF; i++)
    {
        for (int j = 0; j < DOF; j++)
        {
            if (i > j)
            {
                Pose3 tmp1;
                Pose3 tmp2;
                hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 4, 4, 4, 4, 4, 4, float, float>(Ho[j].T, dH[j].T, tmp1.T);
                hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 4, 4, 4, 4, 4, 4, float, float>(tmp1.T, Hoinv[j + 1].T, tmp2.T);
                hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 4, 4, 4, 4, 4, 4, float, float>(tmp2.T, Ho[i + 1].T, dHo_dq[i][j].T);
            }
            else 
            {
                hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 4, 4, 4, 4, 4, 4, float, float>(Ho[j].T, dH[j].T, dHo_dq[i][j].T);
            }
        }
    }

    for (int i = 0; i < DOF; i++)
    {
        link_pose[i] = Ho[i + 1];

        J_pose_angle &Jp = Jacobian[i];

        Pose3 inv_jpx_i = link_pose[i].inverse();

        for (int j = 0; j <= i; j++)
        {
            Pose3 sym_se3;
            hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 4, 4, 4, 4, 4, 4, float, float>(inv_jpx_i.T, dHo_dq[i][j].T, sym_se3.T);
            Jp.J[0][j] = sym_se3.T[2][1];
            Jp.J[1][j] = sym_se3.T[0][2];
            Jp.J[2][j] = sym_se3.T[1][0];
            Jp.J[3][j] = sym_se3.T[0][3];
            Jp.J[4][j] = sym_se3.T[1][3];
            Jp.J[5][j] = sym_se3.T[2][3];
        }
    }
}

void sphere_centers(float joint_angle[DOF], Point3 sph_centers[SPHERES_NUM], J_point_conf Jacobian[SPHERES_NUM])
{
    Pose3 link_poses[DOF];
    J_pose_angle J_pose_jp[DOF];

    forward_kinematics(joint_angle, link_poses, J_pose_jp);

    for (int sph_idx = 0; sph_idx < SPHERES_NUM; sph_idx++)
    {
        float J_point_pose[3][6];
        sph_centers[sph_idx] = link_poses[body_sphere[sph_idx].link_id].transform_from(body_sphere[sph_idx].center, J_point_pose);
        hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 3, 6, 6, DOF, 3, DOF, float, float>(J_point_pose, J_pose_jp[body_sphere[sph_idx].link_id].J, Jacobian[sph_idx].J);
    }
}


float hinge_loss_obstatle_cost(Point2 &point, PlanarSDF &sdf, float eps, float Jacobian[2])
{
    Point2 field_gradient;
    float signed_dist = sdf.get_signed_distance(point, field_gradient);

    if(signed_dist > eps)
    {
        Jacobian[0] = 0.0;
        Jacobian[1] = 0.0;
        return 0.0;
    }
    else
    {
        Jacobian[0] = -field_gradient.x;
        Jacobian[1] = -field_gradient.y;
        return eps - signed_dist;
    }
}
