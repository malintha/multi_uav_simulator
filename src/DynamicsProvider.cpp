//
// Created by malintha on 7/31/18.
//

#include "DynamicsProvider.h"
#include <cstdio>
#include <gsl/gsl_errno.h>
#include <ros/console.h>

/**
 * The equations and state_space use NED inertial frame. Therefore, the initial values are needed to be
 * converted from map frame (NWU) to NED frame. And convert them back to map frame before broadcasting.
 * 
 * Note: the drone is already started upside down in the xacro, matching to the NED. 
 * So no need to convert the rotation matrix and omega.
*/
DynamicsProvider::DynamicsProvider(params_t params, init_vals_t init_vals)
{
    this->params = params;
    this->init_vals = init_vals;
    if (!state_set)
    {
        state_space.position = simulator_utils::ned_nwu_rotation(init_vals.position);
        state_space.velocity = simulator_utils::ned_nwu_rotation(init_vals.velocity);
        state_space.R = (init_vals.R);
        state_space.omega = (init_vals.omega);
        state_set = true;
    }

    s = gsl_odeiv2_step_alloc(T, DIMENSIONS);
    c = gsl_odeiv2_control_y_new(1e-5, 0.0);
    e = gsl_odeiv2_evolve_alloc(DIMENSIONS);

}

/**
 * ODE step for updating the quad state Dynamic equations are obtained from 
 * Geometric Tracking Control of a Quadrotor UAV on SE(3), Lee et al, (2010)
*/
int DynamicsProvider::step(double t, const double *y, double *f, void *params)
{
    auto *param = reinterpret_cast<params_t *>(params);
    (void) (t);
    // copy R values from y
    double R_temp[9];
    for (int i = 6, k = 0; i < 15; i++, k++)
    {
        R_temp[k] = y[i];
    }
    Matrix3d R(R_temp);

    Vector3d omega(y[15], y[16], y[17]);

    // velocity for the next timestep eq(3)
    Vector3d e3(0, 0, 1);
    Vector3d v = param->gravity * e3 - (1 / param->mass) * (param->F * R * e3);

    // Rdot for next timestep eq(4)
    Matrix3d Rdot = R * simulator_utils::hat(omega);

    // Omegadot for the next timestep eq(5)
    Vector3d Omegadot = param->J_inv * (param->M - omega.cross(param->J * omega));

    // writing values to f
    // position of next timestep eq(2)
    int k = 0;
    for (int i = 3; i < 6; i++, k++)
    {
        f[k] = y[i];
    }
    for (int i = 0; i < 3; i++, k++)
    {
        f[k] = v[i];
    }
    for (int i = 0; i < 9; i++, k++)
    {
        f[k] = Rdot.data()[i];
    }
    for (int i = 0; i < 3; i++, k++)
    {
        f[k] = Omegadot[i];
    }
    return GSL_SUCCESS;
}

void DynamicsProvider::update(control_out_t control, double sim_time_)
{
    t_start = sim_time;
    double t_stop = sim_time_;

    // update params
    params.F = control.F;
    params.M = control.M;

    // load current state from state space
    double y[DIMENSIONS];
    int k = 0;
    for (int i = 0; i < 3; i++, k++)
    {
        y[k] = state_space.position[i];
    }

    for (int i = 0; i < 3; i++, k++)
    {
        y[k] = state_space.velocity[i];
    }

    for (int i = 0; i < 9; i++, k++)
    {
        y[k] = state_space.R.data()[i];
    }

    for (int i = 0; i < 3; i++, k++)
    {
        y[k] = state_space.omega[i];
    }

    gsl_odeiv2_system sys = {DynamicsProvider::step, nullptr, DIMENSIONS, &params};
    double t = t_start;
    double h = 1e-5;
    while (t < t_stop)
    {
        // pass the controller c for adaptive step size or nullptr for fixed step size
        int status = gsl_odeiv2_evolve_apply(e, c, s,
                                             &sys,
                                             &t, t_stop,
                                             &h, y);

        if (status != GSL_SUCCESS)
            break;
    }

    // set current state
    k = 0;
    for (int i = 0; i < 3; i++, k++)
    {
        state_space.position[i] = y[k];
    }

    for (int i = 0; i < 3; i++, k++)
    {
        state_space.velocity[i] = y[k];
    }

    double R_[9];
    for (int i = 0; i < 9; i++, k++)
    {
        R_[i] = y[k];
    }
    // Map<Matrix<double,3,3> > mat_(R_);
    state_space.R = Matrix3d(R_);

    for (int i = 0; i < 3; i++, k++)
    {
        state_space.omega[i] = y[k];
    }
    // ROS_DEBUG_STREAM(t_stop<<" R: \n" << state_space.R << " \n v: \n" << state_space.velocity);

    sim_time = t_stop;
}

state_space_t DynamicsProvider::get_state()
{
    return this->state_space;
}

void DynamicsProvider::reset_dynamics() {
    gsl_odeiv2_evolve_reset(this->e);
}
