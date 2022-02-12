//
// Created by malintha on 11/27/17.
//

#ifndef PROJECT_TRACKING_CONTROL_H
#define PROJECT_TRACKING_CONTROL_H

#pragma once

#include <eigen3/Eigen/Dense>
#include "simulator_utils/simulator_utils.h"
#include "definitions.h"

using namespace Eigen;

class ControllerImpl
{

public:
    ControllerImpl(params_t params_, init_vals_t init_vals_, gains_t gains_, double dt_);
    control_out_t get_control(state_space_t ss_, const desired_state_t& desired_s);
    
private:

    params_t params;
    init_vals_t init_vals;
    
    double dt;
    gains_t gains;

    // derivative operators
    dxdt dxdt_;
    dxdt d2xdt;
    dxdt d3xdt;
    dxdt d4xdt;
    dxdt db1dt;
    dxdt d2b1dt;
    dxdt dvdt;
    dxdt d2vdt;

    derivatives_t get_derivatives(const desired_state_t& desired_state, const state_space_t& ss);



    
};

#endif
