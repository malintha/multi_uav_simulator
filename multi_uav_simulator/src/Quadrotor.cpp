//
// Created by malintha on 8/22/18.
//
#include "Quadrotor.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include "simulator_utils/simulator_utils.h"

mav_trajectory_generation::Trajectory Quadrotor::get_opt_traj(const opt_t &ps, const Vector3d& pe) {
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex v_s(3), v_e(3);
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;

    v_s.addConstraint(mav_trajectory_generation::derivative_order::POSITION, ps.position);
    v_s.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, ps.velocity);
    v_s.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, ps.acceleration);

    v_e.makeStartOrEnd(pe, derivative_to_optimize);

    vertices.push_back(v_s);
    vertices.push_back(v_e);

    mav_trajectory_generation::PolynomialOptimization<8> opt(3);
    std::vector<double> segment_times;
    const double v_max = 1;
    const double a_max = 4;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    ROS_DEBUG_STREAM("segement times: "<<segment_times[0]);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);
    return trajectory;
}

Quadrotor::Quadrotor(int robot_id, double frequency, ros::NodeHandle &n)
        : robot_id(robot_id), frequency(frequency), nh(n) {
    sim_time = 0;
    this->dt = 1/frequency;
    this->initialize(1 / frequency);
    this->u << 0,0,0;

    while (marker_pub.getNumSubscribers() < 1) {
        ROS_INFO("Waiting for subscriber");
        ros::Duration(1).sleep();
    }
}

bool Quadrotor::initialize(double dt_) {
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
    controller = new ControllerImpl(params, init_vals, gains, dt_);

    this->setState(State::Autonomous);
    initPaths();

    desired_state_sub = nh.subscribe("desired_state", 10, &Quadrotor::desired_pos_cb, this);
    state_pub = nh.advertise<simulator_utils::Waypoint>("current_state", 10);
    ROS_DEBUG_STREAM("Drone initialized " << robot_id);
    ROS_INFO_STREAM("Desired state subscriber topic: " << "robot_"<<robot_id<<"/desired_state");
    ROS_INFO_STREAM("State publisher topic: " << "robot_"<<robot_id<<"/current_state");

    this->set_init_target = false;
    return true;
}

void Quadrotor::initPaths() {
    marker_pub = nh.advertise<visualization_msgs::Marker>(
            ros::names::append(robot_link_name, "path"), 10);

    goal_pub = nh.advertise<visualization_msgs::Marker>(
            ros::names::append(robot_link_name, "goal"), 10);
    g.header.stamp = m.header.stamp = ros::Time::now();
    g.header.frame_id = m.header.frame_id = worldframe;
    m.type = visualization_msgs::Marker::LINE_STRIP;

    m.action = visualization_msgs::Marker::ADD;
    m.id = this->robot_id;
    m.color.r = 1;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1;
    m.scale.x = 0.05;
    m.pose.orientation.w = 1.0;

    g.id = this->robot_id + 20;
    g.type = visualization_msgs::Marker::CUBE;
    g.action = visualization_msgs::Marker::ADD;
    g.color.r = 1;
    g.color.g = 0;
    g.color.b = 0;
    g.color.a = 0.5;
    g.scale.x = 0.1;
    g.scale.y = 0.1;
    g.scale.z = 0.1;

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
    if(quad_initialized && !isnan(q.x()))
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), worldframe, robot_link_name));
}

State Quadrotor::getState() {
    return this->m_state;
}

bool Quadrotor::load_params() {
    stringstream ss;
    ss << "/robot_" << to_string(this->robot_id);
    string robot_name = ss.str();
    double temp;
    if (!nh.getParam(ros::names::append(robot_name, "drone/model/gravity"), params.gravity)) return false;
    //  = temp;
    vector<double> J_;
    if (!nh.getParam(ros::names::append(robot_name, "drone/model/J"), J_)) return false;
    // Matrix3d J;
    params.J << J_[0], 0, 0,
                0, J_[1], 0,
                0, 0, J_[2];
    params.J_inv = params.J.inverse();
    if (!nh.getParam(ros::names::append(robot_name, "drone/model/m"), params.mass)) return false;
    //  = temp;
    params.F = 0;
    Vector3d M(0, 0, 0);
    params.M = M;
    vector<double> gains_;
    if (!nh.getParam(ros::names::append(robot_name, "drone/controller/gains"), gains_)) return false;
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
    if (!nh.getParam("/frame/prefix", localframe)) return false;
    init_vals.position = Vector3d(position.data());
    init_vals.velocity = Vector3d(vel.data());
    init_vals.R = Matrix3d(R.data());
    init_vals.omega = Vector3d(omega.data());

    this->x0.position = simulator_utils::ned_nwu_rotation(init_vals.position);
    this->x0.velocity = simulator_utils::ned_nwu_rotation(init_vals.velocity);
    this->xd0.position = this->x0.position;
    this->target_pos = simulator_utils::ned_nwu_rotation(init_vals.position);
    ROS_DEBUG_STREAM("Loaded the drone initialization values");
    return true;
}

void Quadrotor::desired_pos_cb(const geometry_msgs::Point &pt) {
    Vector3d p1 = {pt.x, pt.y, pt.z};
    Vector3d p2 = target_pos;
    if((p2 - p1).norm() >= 0.2) {
        if(!set_init_target) {
            set_init_target = true;
            // set init trajectory
            Vector3d init_z = {0,0,0};
            opt_t ps = {target_pos, init_z, init_z, init_z};
            Vector3d pe = {pt.x, pt.y, pt.z};
            this->traj = get_opt_traj(ps, pe);
            this->target_pos = pe;
            ROS_DEBUG_STREAM(robot_id<<" Set init trajectory");
        }
        else {
            this->target_next = pt;
            this->target_pos = {pt.x, pt.y, pt.z};
            this->set_next_target = true;
            ROS_DEBUG_STREAM(robot_id<<" Set new target");
        }
    }
}

// if there is another target available, calculate a new trajectory
void Quadrotor::do_rhp() {

    if (set_next_target) {
        Vector3d pt = {target_next.x, target_next.y, target_next.z};
        Vector3d ps, vel(0,0,0), acc(0,0,0);
        // changing to a new trajectory midway
        ROS_DEBUG_STREAM("calculating a new trajectory");
        ps = this->dynamics->get_state().position;
        vel = this->dynamics->get_state().velocity;
        acc = this->dynamics->get_state().acceleration;

        opt_t wp = {ps, vel, acc};

        this->traj = get_opt_traj(wp, pt);
        ROS_DEBUG_STREAM("Setting a new trajectory of length: "<<traj.getMaxTime() << "for drone: "<<robot_id);

        this->set_next_target = false;
        tau = 0;
    };
}

// xd should be in NED frame and so does dynamics and controller.
void Quadrotor::iteration(const ros::TimerEvent &e) {
    // compute the next desired state using incoming control signal and current state
    Vector3d b1d(1, 0, 0);
    Vector3d xd = simulator_utils::ned_nwu_rotation(init_vals.position);

    // use traj optimization to navigate to the desired state
    if(this->set_init_target) {
        if(set_next_target) {
            do_rhp();
        }
        xd = traj.evaluate(tau, mav_trajectory_generation::derivative_order::POSITION);

        if(abs(tau - traj.getMaxTime()) >= dt) {
            tau = tau + dt;
        }
    }

    desired_state_t dss = {xd, b1d};
    this->move(dss);
    this->publish_path();
    this->publish_state();
}

void Quadrotor::publish_path() {
    constexpr int arrowLength = 0.05;
    Vector3d x = this->dynamics->get_state().position;
    x[1] = -x[1];
    geometry_msgs::Point p;
    if(!isnan(x[0]) && !isnan(x[1])) {
        p.x = x[0];
        p.y = x[1];
        p.z = x[2];
        quad_initialized = true;
    }
    else
    {
        p.x = 0;
        p.y = 0;
        p.z = 0;
    }
    
    m.points.push_back(p);
    if (m.points.size() > 1000)
        m.points.erase(m.points.begin());

    g.pose.position.x = this->target_pos[0];
    g.pose.position.y = -this->target_pos[1];
    g.pose.position.z = this->target_pos[2];
    g.pose.orientation.x = 0.0;
    g.pose.orientation.y = 0.0;
    g.pose.orientation.z = 0.0;
    g.pose.orientation.w = 1.0;

    goal_pub.publish(g);
    marker_pub.publish(m);
}

void Quadrotor::run() {
    ros::Timer timer = nh.createTimer(ros::Duration(1 / frequency), &Quadrotor::iteration, this);
    ros::spin();
}

void Quadrotor::publish_state() {
    simulator_utils::Waypoint wp;
    geometry_msgs::Point p,v;
    state_space_t ned_state = this->dynamics->get_state();
    p.x = ned_state.position[0];
    p.y = ned_state.position[1];
    p.z = ned_state.position[2];
    v.x = ned_state.velocity[0];
    v.y = ned_state.velocity[1];
    v.z = ned_state.velocity[2];
    wp.position = p;
    wp.velocity = v;
    wp.acceleration.x = wp.acceleration.y = wp.acceleration.z = 0;
    this->state_pub.publish(wp);
}

int main(int argc, char **argv) {

    int robot_id = std::atoi(argv[1]);
    double frequency = (double)std::atof(argv[2]);

    ROS_DEBUG_STREAM("initializing : " << robot_id << endl);
    ros::init(argc, argv, "~");
    ros::NodeHandle n;
    Quadrotor quad(robot_id, frequency, n);
    quad.run();

    return 0;
}