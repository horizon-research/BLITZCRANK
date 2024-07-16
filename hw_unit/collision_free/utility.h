#pragma once

#include "../parameter.h"
#include "hls_math.h"
#include "point.h"
#include "pose.h"
#include "planar_sdf.h"
#include "robot_model.h"
#include "jacobian.h"

void compute_link_trans_notheta();
void forward_kinematics(float joint_angle[DOF], Pose3 link_pose[DOF], J_pose_angle Jacobian[DOF]);
void getH(const int i, const float theta, Pose3 &H);
void getdH(const int i, const float theta, Pose3 &dH);
void sphere_centers(float joint_angle[DOF], Point3 sph_centers[SPHERES_NUM], J_point_conf Jacobian[SPHERES_NUM]);
float hinge_loss_obstatle_cost(Point2 &point, PlanarSDF &sdf, float eps, float Jacobian[2]);


