#include "simulator_utils.h"
#include <ros/console.h>

Matrix3d simulator_utils::hat(Vector3d v)
{
    Matrix3d hat_map(3, 3);
    hat_map << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return hat_map;
}

Vector3d simulator_utils::vee(Matrix3d mat)
{
    Vector3d vee_vec(mat(2, 1), mat(0, 2),mat(1, 0));
    return vee_vec;
}

Vector3d simulator_utils::R2RPY(Matrix3d R)
{
    Vector3d rpy;
    rpy << atan2(R(2, 1), R(2, 2)),
        atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2))),
        atan2(R(1, 0), R(0, 0));
    return rpy;
}

Vector3d simulator_utils::ned_nwu_rotation(Vector3d v)
{
    Matrix3d rot;
    rot << 1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    return rot * v;
}

Matrix3d simulator_utils::ned_nwu_rotation(Matrix3d m)
{
    Matrix3d rot;
    rot << 1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    return rot * m;
}

