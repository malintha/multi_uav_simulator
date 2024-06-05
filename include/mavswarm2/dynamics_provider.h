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

#ifndef CF_SIMULATOR_DYNAMICSPROVIDER_H
#define CF_SIMULATOR_DYNAMICSPROVIDER_H

#include <eigen3/Eigen/Dense>
// #include "tf2/transform_datatypes.h"
#include "simulator_utils/types.h"
#include <gsl/gsl_odeiv2.h>

using namespace Eigen;

class DynamicsProvider {

public:
    DynamicsProvider(params_t params, init_vals_t init_vals);
    std::vector<Vector3d> getCurrentState();
    state_space_t get_state();
    void update(control_out_t control, double sim_time);
    void reset_dynamics();

private:
    const size_t DIMENSIONS = 18;
    const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rkf45;
    // Check if GSL is compatible with ROS 2, otherwise use alternatives
    bool state_set = false;
    state_space_t state_space;
    params_t params;
    init_vals_t init_vals;
    double sim_time, t_start;
    static int step(double t, const double* y, double* f, void *params);
    gsl_odeiv2_step* s;
    gsl_odeiv2_control *c;
    gsl_odeiv2_evolve *e;

};

#endif
