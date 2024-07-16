#pragma once

#include "hls_linear_algebra.h"

class Pose3
{
public:
    float T[4][4];
    // float R[3][3];
    // float t[3];
    Pose3()
    {
    }

    Pose3(float T1[4][4])
    {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                T[i][j] = T1[i][j];
    }

    Pose3 &inverse()
    {
        Pose3 p;

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                p.T[i][j] = this->T[j][i];
            }
        }
        for (int i = 0; i < 3; i++)
        {
            p.T[i][3] = p.T[i][0] * this->T[0][3] + p.T[i][1] * this->T[1][3] + p.T[i][2] * this->T[2][3];
        }

        p.T[3][0] = p.T[3][1] = p.T[3][2] = 0;
        p.T[3][3] = 1;

        return p;
    }

    Point3 &transform_from(const Point3 &p, float Jacobian[3][6])
    {
        float R[3][3];
        float t[3];

        for (int i = 0; i < 3; i++)
        {
            t[i] = this->T[i][3];
            for (int j = 0; j < 3; j++)
            {
                R[i][j] = this->T[i][j];
            }
        }

        float skew[3][3] = {0, p.z, -p.y, -p.z, 0, p.x, p.y, -p.x, 0};
        float leftcols[3][3];

        hls::matrix_multiply<hls::NoTranspose, hls::NoTranspose, 3, 3, 3, 3, 3, 3, float, float>(R, skew, leftcols);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Jacobian[i][j] = leftcols[i][j];
                Jacobian[i][j + 3] = R[i][j];
            }
        }

        Point3 point;

        point.x = R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z + t[0];
        point.y = R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z + t[1];
        point.z = R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z + t[2];

        return point;
    }

    Pose3 &operator=(Pose3 &p)
    {
        Pose3 pose;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                pose.T[i][j] = p.T[i][j];

        return pose;
    }
};