/*

Copyright (c) 2024 Malintha Fernando (malintha@onmail.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#pragma once

#include <eigen3/Eigen/Dense>

using namespace Eigen;

typedef struct {
    Vector3d position;
    Matrix3d R;
    Vector3d omega;
    Vector3d velocity;
    double h;
} init_vals_t;

typedef struct {
    double gravity;
    double mass;
    Matrix3d J;
    Matrix3d J_inv;
    double F;
    Vector3d M;
} params_t;

typedef struct {
    Vector3d position;
    Vector3d velocity;
    Vector3d acceleration;
    Matrix3d R;
    Vector3d omega;
} state_space_t;

typedef struct {
    Vector3d x;
    Vector3d b1;
} desired_state_t;

typedef struct 
{
    double kx;
    double kv;
    double kr;
    double komega;
} gains_t;

typedef struct {
    double F;
    Vector3d M;
} control_out_t;

typedef struct {
    Vector3d position;
    Vector3d velocity;
    Vector3d acceleration;
    Vector3d jerk;
} opt_t;

namespace simulator_utils {
    Matrix3d hat(Vector3d v);
    Vector3d R2RPY(Matrix3d R);
    Vector3d ned_nwu_rotation(Vector3d v);
    Matrix3d ned_nwu_rotation(Matrix3d m);
    Vector3d vee(Matrix3d mat);

}

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