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

#include "quadrotor.hpp"
#include "rclcpp/rclcpp.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
#include "rclcpp/logging.hpp"
// #include "simulator_utils/simulator_utils.h"



/**
 * converts the dynamics state_space (NED) to NWU and stores for the quadrotor
 * This is for broadcasting the transformation
*/
// void Quadrotor::set_state_space() {
//     state_space_t ss = dynamics->get_state();
//     state_space.position = simulator_utils::ned_nwu_rotation(ss.position);
//     state_space.R = simulator_utils::ned_nwu_rotation(ss.R);
//     state_space.velocity = simulator_utils::ned_nwu_rotation(ss.velocity);
//     state_space.omega = simulator_utils::ned_nwu_rotation(ss.omega);
// }

// void Quadrotor::send_transform() {
//     static tf::TransformBroadcaster br;
//     tf::Transform transform;
//     Vector3d position = state_space.position;
//     Matrix3d R = state_space.R;
//     Vector3d rpy = simulator_utils::R2RPY(R);
//     transform.setOrigin(tf::Vector3(position[0], position[1], -position[2]));
//     tf::Quaternion q;
//     q.setRPY(rpy[0], rpy[1], rpy[2]);
//     transform.setRotation(q);
//     if(quad_initialized && !isnan(q.x()))
//         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), worldframe, robot_link_name));
// }

// State Quadrotor::getState() {
//     return this->m_state;
// }

// bool Quadrotor::load_params() {
//     stringstream ss;
//     ss << "/robot_" << to_string(this->robot_id);
//     string robot_name = ss.str();
//     double temp;
//     if (!nh.getParam(ros::names::append(robot_name, "drone/model/gravity"), params.gravity)) return false;
//     //  = temp;
//     vector<double> J_;
//     if (!nh.getParam(ros::names::append(robot_name, "drone/model/J"), J_)) return false;
//     // Matrix3d J;
//     params.J << J_[0], 0, 0,
//                 0, J_[1], 0,
//                 0, 0, J_[2];
//     params.J_inv = params.J.inverse();
//     if (!nh.getParam(ros::names::append(robot_name, "drone/model/m"), params.mass)) return false;
//     //  = temp;
//     params.F = 0;
//     Vector3d M(0, 0, 0);
//     params.M = M;
//     vector<double> gains_;
//     if (!nh.getParam(ros::names::append(robot_name, "drone/controller/gains"), gains_)) return false;
//     this->gains = {gains_[0], gains_[1], gains_[2], gains_[3]};

//     ROS_DEBUG_STREAM(this->logger, "Loaded control parameters");
//     return true;
// }

// bool Quadrotor::load_init_vals() {
//     vector<double> position, vel, R, omega;
//     stringstream ss;
//     ss << "/robot_" << to_string(this->robot_id);
//     string robot_name = ss.str();
//     if (!nh.getParam(ros::names::append(robot_name, "position"), position)) return false;
//     if (!nh.getParam(ros::names::append(robot_name, "velocity"), vel)) return false;
//     if (!nh.getParam(ros::names::append(robot_name, "rotation"), R)) return false;
//     if (!nh.getParam(ros::names::append(robot_name, "omega"), omega)) return false;
//     if (!nh.getParam("/frame/fixed", worldframe)) return false;
//     if (!nh.getParam("/frame/prefix", localframe)) return false;
//     init_vals.position = Vector3d(position.data());
//     init_vals.velocity = Vector3d(vel.data());
//     init_vals.R = Matrix3d(R.data());
//     init_vals.omega = Vector3d(omega.data());

//     this->x0.position = simulator_utils::ned_nwu_rotation(init_vals.position);
//     this->x0.velocity = simulator_utils::ned_nwu_rotation(init_vals.velocity);
//     this->xd0.position = this->x0.position;
//     this->target_pos = simulator_utils::ned_nwu_rotation(init_vals.position);
//     ROS_DEBUG_STREAM(this->logger, "Loaded the drone initialization values");
//     return true;
// }

// void Quadrotor::desired_pos_cb(const geometry_msgs::Point &pt) {
//     Vector3d xd = {pt.x, pt.y, pt.z};
//     Vector3d b1d(1, 0, 0);
//     this->dss = {xd, b1d};

// }


// xd should be in NED frame and so does dynamics and controller.
// void Quadrotor::iteration(const ros::TimerEvent &e) {
//     this->move(this->dss);
//     this->publish_path();
//     this->publish_state();
// }

// void Quadrotor::publish_path() {
//     constexpr int arrowLength = 0.05;
//     Vector3d x = this->dynamics->get_state().position;
//     x[1] = -x[1];
//     geometry_msgs::Point p;
//     if(!isnan(x[0]) && !isnan(x[1])) {
//         p.x = x[0];
//         p.y = x[1];
//         p.z = x[2];
//         quad_initialized = true;
//     }
//     else
//     {
//         p.x = 0;
//         p.y = 0;
//         p.z = 0;
//     }
    
//     m.points.push_back(p);
//     if (m.points.size() > 1000)
//         m.points.erase(m.points.begin());

//     g.pose.position.x = this->target_pos[0];
//     g.pose.position.y = -this->target_pos[1];
//     g.pose.position.z = this->target_pos[2];
//     g.pose.orientation.x = 0.0;
//     g.pose.orientation.y = 0.0;
//     g.pose.orientation.z = 0.0;
//     g.pose.orientation.w = 1.0;

//     goal_pub.publish(g);
//     marker_pub.publish(m);
// }

// void Quadrotor::run() {
//     ros::Timer timer = nh.createTimer(ros::Duration(1 / frequency), &Quadrotor::iteration_rhp, this);
//     ros::spin();
// }

// void Quadrotor::publish_state() {
//     simulator_utils::Waypoint wp;
//     geometry_msgs::Point p,v;
//     state_space_t ned_state = this->dynamics->get_state();
//     p.x = ned_state.position[0];
//     p.y = ned_state.position[1];
//     p.z = ned_state.position[2];
//     v.x = ned_state.velocity[0];
//     v.y = ned_state.velocity[1];
//     v.z = ned_state.velocity[2];
//     wp.position = p;
//     wp.velocity = v;
//     wp.acceleration.x = wp.acceleration.y = wp.acceleration.z = 0;
//     this->state_pub.publish(wp);
// }

// bool Quadrotor::initialize(double dt_) {
//     m_state = State::Idle;
//     // load drone params
//     if (!load_params()) {
//         RCLCPP_ERROR(this->get_logger(), "Could not load the drone parameters");
//         return false;
//     }
//     // load init params
//     if (!load_init_vals()) {
//         RCLCPP_ERROR(this->get_logger(), "Could not load the drone initial values");
//         return false;
//     }

//     std::stringstream ss;
//     ss << localframe << robot_id;
//     robot_link_name = ss.str();

//     // set initial values
//     state_space.position = init_vals.position;
//     state_space.velocity = init_vals.velocity;
//     state_space.R = init_vals.R;
//     state_space.omega = init_vals.omega;

//     dynamics = new DynamicsProvider(params, init_vals);
//     controller = new ControllerImpl(params, init_vals, gains, dt_);

//     this->setState(State::Autonomous);
//     initPaths();

//     desired_state_sub = this->create_subscription<geometry_msgs::msg::Point>(
//         "desired_state", 10, std::bind(&Quadrotor::desired_pos_cb, this, std::placeholders::_1));
//     state_pub = this->create_publisher<simulator_utils::msg::Waypoint>("current_state", 10);
//     RCLCPP_INFO(this->get_logger(), "Drone initialized %d", robot_id);
//     RCLCPP_INFO(this->get_logger(), "Desired state subscriber topic: robot_%d/desired_state", robot_id);
//     RCLCPP_INFO(this->get_logger(), "State publisher topic: robot_%d/current_state", robot_id);

//     this->set_init_target = false;
//     return true;
// }

// void Quadrotor::setState(State m_state_) {
//     this->m_state = m_state_;
// }



int main(int argc, char **argv) {

    int robot_id = std::atoi(argv[1]);
    double frequency = (double)std::atof(argv[2]);
    auto logger = rclcpp::get_logger("logger");

    RCLCPP_DEBUG(logger, "initializing : ");
    rclcpp::init(argc, argv);
    stringstream ss;
    // ss << "robot_"<<robot_id;
    // rclcpp::Node n = std::make_shared<rclcpp::Node>(ss.str());
    Quadrotor quad(robot_id, frequency);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Quadrotor>(0, 500));
    rclcpp::shutdown();

    return 0;
}