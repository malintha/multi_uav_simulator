//
// Created by malintha on 8/22/18.
//
#include "Quadrotor.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include "simulator_utils/simulator_utils.h"

mav_trajectory_generation::Trajectory Quadrotor::get_opt_traj(const opt_t &ps, Vector3d pe) {
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex v_s(3), v_e(3);
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;

    v_s.addConstraint(mav_trajectory_generation::derivative_order::POSITION, ps.position);
    v_s.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, ps.velocity);
    v_s.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, ps.acceleration);
    v_s.addConstraint(mav_trajectory_generation::derivative_order::JERK, ps.jerk);

    v_e.makeStartOrEnd(pe, mav_trajectory_generation::derivative_order::POSITION);
    vertices.push_back(v_s);
    vertices.push_back(v_e);

    mav_trajectory_generation::PolynomialOptimization<8> opt(3);
    std::vector<double> segment_times;
    const double v_max = 1.0;
    const double a_max = 1.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
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

    desired_state_sub = nh.subscribe("desired_state", 1, &Quadrotor::desired_pos_cb, this);
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

    this->target_pos = simulator_utils::ned_nwu_rotation(init_vals.position);
    ROS_DEBUG_STREAM("Loaded the drone initialization values");
    return true;
}

void Quadrotor::desired_pos_cb(const geometry_msgs::Point &pt) {
    // first get current state
//    simulator_utils::Waypoint wp_c;
//    Vector3d p1 = {pt.x, pt.y, pt.z};
    //accept iff incoming pos is at least 0.25 further away from current position
//    if((p1-p_c).norm() > 0.5)
//        if (set_init_target) {
//            // existing target
//
//
//        }
//        else {
//            // initial state, when no traj is set
//            this->target_pos = pt;
//            wp_c.acceleration.x = 0;
//            wp_c.acceleration.y = 0;
//            wp_c.acceleration.z = 0;
//            mav_trajectory_generation::Trajectory tr = get_opt_traj(wp_c, pt);
//            this->traj = tr;
//            this->tau = 0;
//            ROS_DEBUG_STREAM("set init traj");
//            this->set_init_target = true;
//
//        }

    Vector3d p1 = {pt.x, pt.y, pt.z};
    Vector3d p2 = target_pos;
    if((p2 - p1).norm() >= 0.5) {
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
            this->set_next_target = true;
            ROS_DEBUG_STREAM(robot_id<<" Set new target");
        }
    }
}

// xd should be in NED frame and so does dynamics and controller.
void Quadrotor::iteration(const ros::TimerEvent &e) {
    // compute the next desired state using incoming control signal and current state
    Vector3d b1d(1, 0, 0);
    Vector3d xd;
    xd << simulator_utils::ned_nwu_rotation(init_vals.position);
    if(this->set_init_target) {
        if(set_next_target) {
            do_rhp();
        }
        xd = traj.evaluate(tau, mav_trajectory_generation::derivative_order::POSITION);

        if(abs(tau - traj.getMaxTime()) > dt) {
            tau = tau + dt;
        }
    }
    // get desired state from topic
    desired_state_t dss = {xd, b1d};
    this->move(dss);
    this->publish_path();
    this->publish_state();
}
// if there is another target available, calculate a new trajectory
// if the current tau is larger than T and there is another trajectory, switch to it
void Quadrotor::do_rhp() {
    double T = 0.5;
    if (tau >= T || abs(tau - traj.getMaxTime()) < 1e-2) {
        Vector3d pt = {target_next.x, target_next.y, target_next.z};
        Vector3d ps = traj.evaluate(tau, mav_trajectory_generation::derivative_order::POSITION);
        Vector3d vel = traj.evaluate(tau, mav_trajectory_generation::derivative_order::VELOCITY);
        Vector3d acc = traj.evaluate(tau, mav_trajectory_generation::derivative_order::ACCELERATION);
        Vector3d jerk = traj.evaluate(tau, mav_trajectory_generation::derivative_order::JERK);
        opt_t wp = {ps, vel, acc, jerk};

        mav_trajectory_generation::Trajectory tr_temp = get_opt_traj(wp, pt);
        this->traj = tr_temp;
        ROS_DEBUG_STREAM(robot_id << " Set new traj. Tau: "<<tau);

        this->set_next_target = false;
        tau = 0;
    };

//    mav_trajectory_generation::Trajectory tr_temp;
//    // compute a traj iff the incoming pos is 0.25 away from the current target
//        simulator_utils::Waypoint wp;
////                if (tau < T) {
//        // evaluate current traj at T
//        Vector3d p = traj.evaluate(tau, mav_trajectory_generation::derivative_order::POSITION);
//        Vector3d v = traj.evaluate(tau, mav_trajectory_generation::derivative_order::VELOCITY);
//        Vector3d a = traj.evaluate(tau, mav_trajectory_generation::derivative_order::ACCELERATION);
//        wp.position.x = p[0]; wp.position.y = p[1]; wp.position.z = p[2];
//        wp.velocity.x = v[0]; wp.velocity.y = v[1]; wp.velocity.z = v[2];
//        wp.acceleration.x = a[0]; wp.acceleration.y = a[1]; wp.acceleration.z = a[2];
//                } else {
//                    // use cuttent state
//                    Vector3d a = traj.evaluate(tau, mav_trajectory_generation::derivative_order::ACCELERATION);
//                    wp = wp_c;
//                    wp.acceleration.x = a[0]; wp.acceleration.y = a[1]; wp.acceleration.z = a[2];
//                    ROS_DEBUG_STREAM("computed new traj. Tau: "<<tau);
//
//                }

//    }

//    if ((tau > T || abs(tau - traj.getMaxTime())<dt) && !traj_temp.empty()) {
//        ROS_DEBUG_STREAM("robot: "<<robot_id<<" setting new traj. current: "<<traj.getMaxTime()<<" tau: "<<tau);
//        Vector3d pt0 = traj_temp.evaluate(0, mav_trajectory_generation::derivative_order::POSITION);
//
//        this->traj = traj_temp;
//        traj_temp.clear();
//
//        // set new target position
//        Vector3d pos = traj.evaluate(traj.getMaxTime(),mav_trajectory_generation::derivative_order::POSITION);
//        this->target_pos.x = pos[0];
//        this->target_pos.y = pos[1];
//        this->target_pos.z = pos[2];
//
//        this->tau = 0;
//    }
}

void Quadrotor::publish_path() {
    Vector3d x = this->dynamics->get_state().position;
    x[1] = -x[1];
    geometry_msgs::Point p;
    p.x = x[0];
    p.y = x[1];
    p.z = x[2];
    m.points.push_back(p);

    if (m.points.size() > 500)
        m.points.erase(m.points.begin());

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