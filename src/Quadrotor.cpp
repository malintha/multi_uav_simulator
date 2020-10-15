//
// Created by malintha on 8/22/18.
//
#include "Quadrotor.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include "simulator_utils/simulator_utils.h"

Quadrotor::Quadrotor(int robot_id, double frequency, ros::NodeHandle &n)
        : robot_id(robot_id), frequency(frequency), nh(n) {
    sim_time = 0;
    this->dt = 1/frequency;
    this->initialize(1 / frequency);
    this->u << 0,0,0;
}

bool Quadrotor::initialize(double dt) {
    m_state = State::Idle;
    // load quad params
    if (!load_params()) {
        ROS_ERROR_STREAM("Could not load the drone parameters");
        return false;
    }
    // load init params
    if (!load_init_vals()) {
        ROS_ERROR_STREAM("Could not load the drone initial values");
        return false;
    }

    stringstream ss;
    ss << localframe << to_string(robot_id);
    robot_link_name = ss.str();

    // set initial values
    state_space.position = init_vals.position;
    state_space.velocity = init_vals.velocity;
    state_space.R = init_vals.R;
    state_space.omega = init_vals.omega;

    dynamics = new DynamicsProvider(params, init_vals);
    controller = new ControllerImpl(params, init_vals, gains, dt);

    this->setState(State::Autonomous);
    initPaths();

    desired_state_sub = nh.subscribe("desired_state",1, &Quadrotor::desired_state_cb, this);
    state_pub = nh.advertise<simulator_utils::Waypoint>("current_state", 1);
    ROS_DEBUG_STREAM("Drone initialized " << robot_id);
    ROS_INFO_STREAM("Desired state subscriber topic: " << "robot_"<<robot_id<<"/desired_state");
    ROS_INFO_STREAM("State publisher topic: " << "robot_"<<robot_id<<"/current_state");
    while (marker_pub.getNumSubscribers() < 1) {
        ROS_INFO_STREAM("Waiting for subscriber: "<<robot_id);
        ros::Duration(1).sleep();
    }

    return true;
}

void Quadrotor::initPaths() {
    stringstream ss;

    marker_pub = nh.advertise<visualization_msgs::Marker>(
            ros::names::append(robot_link_name, "path"), 10);

    m.header.stamp = ros::Time::now();
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.header.frame_id = worldframe;
    m.action = visualization_msgs::Marker::ADD;
    m.id = this->robot_id;
    m.color.r = 1;
    m.color.g = 0;
    m.color.b = 0;
    m.color.a = 1;
    m.scale.x = 0.02;
    m.pose.orientation.w = 1.0;
}

void Quadrotor::setState(State m_state_) {
    this->m_state = m_state_;
}

void Quadrotor::move(const desired_state_t &d_state) {
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

bool Quadrotor::load_params() {
    double temp;
    if (!nh.getParam("/uav/gravity", temp)) return false;
    params.gravity = temp;
    vector<double> J_;
    if (!nh.getParam("/uav/J", J_)) return false;
    Matrix3d J;
    J <<    J_[0], 0, 0,
            0, J_[1], 0,
            0, 0, J_[2];
    params.J = J;
    params.J_inv = J.inverse();
    if (!nh.getParam("/uav/m", temp)) return false;
    params.mass = temp;
    params.F = 0;
    Vector3d M(0, 0, 0);
    params.M = M;
    vector<double> gains_;
    if (!nh.getParam("/controller/gains", gains_)) return false;
    this->gains = {gains_[0], gains_[1], gains_[2], gains_[3]};

    ROS_DEBUG_STREAM("Loaded control parameters");
    return true;
}

bool Quadrotor::load_init_vals() {
    vector<double> position, vel, R, omega;
    stringstream ss;
    ss << "/robot_" << to_string(this->robot_id);
    string robot_name = ss.str();
    if (!nh.getParam(ros::names::append(robot_name, "position"), position)) return false;
    if (!nh.getParam(ros::names::append(robot_name, "velocity"), vel)) return false;
    if (!nh.getParam(ros::names::append(robot_name, "rotation"), R)) return false;
    if (!nh.getParam(ros::names::append(robot_name, "omega"), omega)) return false;
    if (!nh.getParam("/frame/fixed", worldframe)) return false;
    if (!nh.getParam("drone/frame/prefix", localframe)) return false;
    init_vals.position = Vector3d(position.data());
    init_vals.velocity = Vector3d(vel.data());
    init_vals.R = Matrix3d(R.data());
    init_vals.omega = Vector3d(omega.data());

    this->x0.position = simulator_utils::ned_nwu_rotation(init_vals.position);
    this->x0.velocity = simulator_utils::ned_nwu_rotation(init_vals.velocity);

    ROS_DEBUG_STREAM("Loaded the drone initialization values");
    return true;
}

void Quadrotor::desired_state_cb(const geometry_msgs::PointConstPtr &pt) {
    this->x0.position = this->dynamics->get_state().position;
    this->x0.velocity = this->dynamics->get_state().velocity;
    this->u << pt->x, pt->y, pt->z;
    this->u = simulator_utils::ned_nwu_rotation(this->u);
    this->tau = 0;
}

// xd should be in NED frame and so does dynamics and controller.
void Quadrotor::iteration(const ros::TimerEvent &e) {
    // compute the next desired state using incoming control signal and current state
    Vector3d b1d(1, 0, 0);
    tau = tau + dt;
    // get desired state from topic
    Vector3d xd = 0.5 * u * pow(tau, 2);
    xd = xd +  x0.position + x0.velocity*tau;
    xd[2] = 0;
//    if(robot_id==1)
//    ROS_DEBUG_STREAM("xd: "<<xd[0]<<" "<<xd[1]<<" "<<xd[2]);
    desired_state_t dss = {xd, b1d};
    this->move(dss);
    this->publish_path();
    this->publish_state();
}

void Quadrotor::publish_path() {
    Vector3d x = this->dynamics->get_state().position;
    x[1] = -x[1];
    geometry_msgs::Point p;
    p.x = x[0];
    p.y = x[1];
    p.z = x[2];
    m.points.push_back(p);

    if (m.points.size() > 100)
        m.points.pop_back();

    marker_pub.publish(m);
}

void Quadrotor::run() {
    ros::Timer timer = nh.createTimer(ros::Duration(1 / frequency), &Quadrotor::iteration, this);
    ros::spin();
}

void Quadrotor::publish_state() {
    simulator_utils::Waypoint wp;
    geometry_msgs::Point p,v;
    p.x = state_space.position[0];
    p.y = state_space.position[1];
    p.z = state_space.position[2];
    v.x = state_space.velocity[0];
    v.y = state_space.velocity[1];
    v.z = state_space.velocity[2];
    wp.position = p;
    wp.velocity = v;
    wp.acceleration.x = wp.acceleration.y = wp.acceleration.z = 0;
    this->state_pub.publish(wp);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "~");
    ros::NodeHandle n;
    int robot_id;
    n.getParam("drone/robot_id", robot_id);

    ROS_DEBUG_STREAM("initializing : " << robot_id << endl);
    stringstream ss;
    double frequency = 100;

    Quadrotor quad(robot_id, frequency, n);
    quad.run();

    return 0;
}