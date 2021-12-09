//
// Created by malintha on 8/22/18.
//
#include "Quadrotor.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include "simulator_utils/simulator_utils.h"

Quadrotor::Quadrotor(int robot_id, string worldframe, string prefix)
        : worldframe(worldframe), robot_id(robot_id), prefix(prefix) {
    sim_time = 0;
    stringstream ss;
    ss << prefix << to_string(robot_id);
    robot_link_name = ss.str();
}

bool Quadrotor::initialize(double dt, gains_t gains) {
    m_state = State::Idle;
    // load quad params
    if (!load_params(nh)) {
        ROS_ERROR_STREAM("Could not load the drone parameters");
        return false;
    }
    // load init params
    if (!load_init_vals(nh)) {
        ROS_ERROR_STREAM("Could not load the drone initial values");
        return false;
    }

    // set initial values
    state_space.position = init_vals.position;
    state_space.velocity = init_vals.velocity;
    state_space.R = init_vals.R;
    state_space.omega = init_vals.omega;

    dynamics = new DynamicsProvider(params, init_vals);
    controller = new ControllerImpl(params, init_vals, gains, dt);

    ROS_INFO_STREAM("Drone initialized " << robot_link_name);
    return true;
}

void Quadrotor::setState(State m_state_) {
    this->m_state = m_state_;
}

void Quadrotor::move(double dt, desired_state_t d_state) {
    control_out_t control = controller->get_control(dynamics->get_state(), d_state);
    dynamics->update(control, sim_time);

    // updating the model on rviz
    set_state_space();
    send_transform();
    dynamics->reset_dynamics();
    sim_time += dt;
}

/**
 * converts the dynamics state_space (NED) to NWU and stores for the quadrotor
 * This is for broadcasting the transformation
*/
void Quadrotor::set_state_space() {
    state_space_t ss = dynamics->get_state();
    state_space.position = simulator_utils::ned_nwu_rotation(ss.position);
    state_space.R = simulator_utils::ned_nwu_rotation(ss.R);
    state_space.velocity = simulator_utils::ned_nwu_rotation(ss.velocity);
    state_space.omega = simulator_utils::ned_nwu_rotation(ss.omega);
}

void Quadrotor::send_transform() {
//    state_space_t ss = dynamics->get_state();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    Vector3d position = state_space.position;
    Matrix3d R = state_space.R;
    Vector3d rpy = simulator_utils::R2RPY(R);
    transform.setOrigin(tf::Vector3(position[0], position[1], -position[2]));
    tf::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), worldframe, robot_link_name));
}

State Quadrotor::getState() {
    return this->m_state;
}

bool Quadrotor::load_params(ros::NodeHandle &nh) {
    double temp;
    if (!nh.getParam("/uav/gravity", temp)) return false;
    params.gravity = temp;
    vector<double> J_;
    if (!nh.getParam("/uav/J", J_)) return false;
    Matrix3d J;
    J << J_[0], 0, 0,
            0, J_[1], 0,
            0, 0, J_[2];
    params.J = J;
    params.J_inv = J.inverse();
    if (!nh.getParam("/uav/m", temp)) return false;
    params.mass = temp;
    params.F = 0;
    Vector3d M(0, 0, 0);
    params.M = M;
    ROS_DEBUG_STREAM("Loaded the drone parameters");

    return true;
}

bool Quadrotor::load_init_vals(ros::NodeHandle &nh) {
    vector<double> position, vel, R, omega;
    stringstream ss;
    ss << "/robot_" << to_string(this->robot_id);
    string robot_name = ss.str();
    if (!nh.getParam(ros::names::append(robot_name, "position"), position)) return false;
    if (!nh.getParam(ros::names::append(robot_name, "velocity"), vel)) return false;
    if (!nh.getParam(ros::names::append(robot_name, "rotation"), R)) return false;
    if (!nh.getParam(ros::names::append(robot_name, "omega"), omega)) return false;
    init_vals.position = Vector3d(position.data());
    init_vals.velocity = Vector3d(vel.data());
    init_vals.R = Matrix3d(R.data());
    init_vals.omega = Vector3d(omega.data());

    ROS_DEBUG_STREAM("Loaded the drone init values");
    return true;
}

state_space_t Quadrotor::get_state_space() {
    return this->state_space;
}

void Quadrotor::set_desired_state(const desired_state_t& desired_ss_) {
    desired_state.x = simulator_utils::ned_nwu_rotation(desired_ss_.x);
    desired_state.b1 = desired_ss_.b1;

}

init_vals_t Quadrotor::get_init_vals() {
    return init_vals;
}

DynamicsProvider *Quadrotor::get_dynamics() {
    return this->dynamics;
}
