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
