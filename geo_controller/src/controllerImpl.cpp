#include "controllerImpl.h"
#include <utility>

namespace su = simulator_utils;

ControllerImpl::ControllerImpl(params_t params_, init_vals_t init_vals_, gains_t gains_, double dt_)
                :params(std::move(params_)), init_vals(std::move(init_vals_)), gains(gains_), dt(dt_)
{
    // initialize derivative operators
    dxdt_ = dxdt(0.05, dt, 1);
    d2xdt = dxdt(0.05, dt, 2);
    d3xdt = dxdt(0.05, dt, 3);
    d4xdt = dxdt(0.05, dt, 4);

    db1dt = dxdt(0.05, dt, 1);
    d2b1dt = dxdt(0.05, dt, 2);

    dvdt = dxdt(0.05, dt, 1);
    d2vdt = dxdt(0.05, dt, 2);
}

control_out_t ControllerImpl::get_control(state_space_t ss, const desired_state_t& desired_s)
{
    // compute the derivatives of desired state
    derivatives_t derivatives = get_derivatives(desired_s, ss);

    Vector3d ex = ss.position - desired_s.x;
    Vector3d ev = ss.velocity - derivatives.d_xd;
    Vector3d ea = derivatives.dv - derivatives.d2_xd;
    Vector3d ej = derivatives.d2v - derivatives.d3_xd;
    Vector3d e3(0,0,1);
    Vector3d A = -gains.kx * ex - gains.kv * ev - params.mass * params.gravity * e3 +
                    params.mass * derivatives.d2_xd;

    double f = -A.dot(ss.R*e3);
    double nA = A.norm();

    Vector3d b3c = -A/nA;
    Vector3d C = b3c.cross(desired_s.b1);
    Vector3d b1c = -(1/C.norm())*b3c.cross(C);
    Vector3d b2c = C/C.norm();
    Matrix3d Rc;
    Rc << b1c, b2c, b3c;

    double nC = C.norm();

    Vector3d A_1dot = -gains.kx*ev - gains.kv*ea + params.mass*derivatives.d3_xd;
    Vector3d b3c_1dot = -A_1dot / nA + (A.dot(A_1dot)/pow(nA,3))*A;

    Vector3d C_1dot = b3c_1dot.cross(desired_s.b1) + b3c.cross(derivatives.db1);
    Vector3d b2c_1dot = C/nC - C.dot(C_1dot)/(pow(nC,3))*C;
    Vector3d b1c_1dot = b2c_1dot.cross(b3c) + b2c.cross(b3c_1dot);

    Vector3d A_2dot = -gains.kx*ea -gains.kv*ej + params.mass*derivatives.d4_xd;
    Vector3d b3c_2dot = -A_2dot/nA + (2.0/pow(nA,3))*A.dot(A_1dot)*A_1dot +
            (pow(A_1dot.norm(),2) + A.dot(A_2dot)/(pow(nA,3)))*A - (3.0/pow(nA,5))*(pow(A.dot(A_1dot),2))*A;
    
    Vector3d C_2dot   = b3c_2dot.cross(desired_s.b1) + (b3c.cross(derivatives.d2b1)) + 2*b3c_1dot.cross(derivatives.db1);

    Vector3d b2c_2dot = C_2dot/nC - 2.0/(pow(nC,3))*C.dot(C_1dot)*C_1dot - (((pow(C_2dot.norm(),2) + C.dot(C_2dot)))/pow(nC,3))*C
                    + (3.0/pow(nC,5))*(pow(C.dot(C_1dot),2))*C;
    
    Vector3d b1c_2dot = b2c_2dot.cross(b3c) + b2c.cross(b3c_2dot) + 2.0*b2c_1dot.cross(b3c_1dot);
    Matrix3d Rc_1dot, Rc_2dot;
    Rc_1dot << b1c_1dot, b2c_1dot, b3c_1dot;
    Rc_2dot << b1c_2dot, b2c_2dot, b3c_2dot;

    Vector3d omegac = su::vee(Rc.transpose()*Rc_1dot);
    Vector3d omegac_1dot = su::vee(Rc.transpose()*Rc_2dot -
                                           su::hat(omegac)*su::hat(omegac));

    Vector3d er = 2.0*su::vee(Rc.transpose()*ss.R - ss.R.transpose()*Rc);
    Vector3d eomega = ss.omega - ss.R.transpose()*Rc*omegac;

    Vector3d M = -gains.kr*er - gains.komega*eomega + ss.omega.cross(params.J*ss.omega)
                - params.J*(su::hat(ss.omega)*ss.R.transpose()*Rc*omegac - ss.R.transpose()*Rc*omegac_1dot);

//    ROS_DEBUG_STREAM("\n eR \n"<<er << " \n vee \n"<<su::vee(Rc));

    control_out_t control = {f, M};

    return control;
}

derivatives_t ControllerImpl::get_derivatives(const desired_state_t& desired_s, const state_space_t& ss)
{
    derivatives_t dds;
    // numerical derivative of desired x
    dds.d_xd = dxdt_.calculate(desired_s.x);
    dds.d2_xd = d2xdt.calculate(dds.d_xd);
    dds.d3_xd = d3xdt.calculate(dds.d2_xd);
    dds.d4_xd = d4xdt.calculate(dds.d3_xd);

    dds.db1 = db1dt.calculate(desired_s.b1);
    dds.d2b1 = d2b1dt.calculate(dds.db1);

    dds.dv = dvdt.calculate(ss.velocity);
    dds.d2v = d2vdt.calculate(dds.dv);

    return dds;
}