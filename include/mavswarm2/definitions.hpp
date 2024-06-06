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

#include <eigen3/Eigen/Dense>

using namespace Eigen;

// compute the derivative of a signal
struct dxdt {
    double tau;
    double ts;
    double a1;
    double a2;
    int order;
    Vector3d dot;
    Vector3d x_d1;
    int it = 1;

dxdt(double tau_, double ts_, int order_) : tau(tau_), ts(ts_), order(order_)
{
    a1 = (2 * tau - ts) / (2 * tau + ts);
    a2 = 2 / (2 * tau + ts);
    dot = Vector3d(0, 0, 0);
    x_d1 = Vector3d(0, 0, 0);
}

dxdt() {}

Vector3d calculate(const Vector3d &x)
{
    if (it > order)
    {
        dot = a1 * dot + a2 * (x - x_d1);
    }
    it++;
    x_d1 = x;
    return dot;
}

};

typedef struct
{
    Vector3d ex;
    Vector3d ev;
    Vector3d ea;
    Vector3d ej;
    Vector3d er;
    Vector3d eomega;
} err_t;

typedef struct {
    // derivatives of desired states
    Vector3d d_xd;
    Vector3d d2_xd;
    Vector3d d3_xd;
    Vector3d d4_xd;
    Vector3d db1;
    Vector3d d2b1;

    // derivatives of current velocity
    Vector3d dv;
    Vector3d d2v;

} derivatives_t;



