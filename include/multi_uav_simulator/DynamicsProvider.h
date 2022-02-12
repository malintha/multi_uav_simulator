//
// Created by malintha on 7/31/18.
//

#ifndef CF_SIMULATOR_DYNAMICSPROVIDER_H
#define CF_SIMULATOR_DYNAMICSPROVIDER_H

#include <eigen3/Eigen/Dense>
#include "tf/tf.h"
#include "cfSimUtils.h"
#include "simulator_utils/simulator_utils.h"
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