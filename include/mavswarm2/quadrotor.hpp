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

#ifndef PROJECT_QUADROTOR_H
#define PROJECT_QUADROTOR_H

#include <string>
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

#include "trajectory_t.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "dynamics_provider.hpp"
#include "controller/geometric_controller.hpp"
#include "simulator_interfaces/msg/waypoint.hpp"

using namespace std::chrono_literals;
#ifndef STATE_H
#define STATE_H
enum State {
    Idle,
    TakingOff,
    Landing,
    Hover,
    Autonomous
};
#endif

using namespace Eigen;
using namespace std;
using namespace std::chrono_literals;

class Quadrotor : public rclcpp::Node {
public:
    Quadrotor(int robot_id, double frequency)
        :Node("robot_"+to_string(robot_id)), frequency(frequency),robot_id(robot_id) {
    sim_time = 0;
    this->dt = 1/frequency;
    // this->initialize(1 / frequency);
    m_state = State::Idle;

    this->u << 0,0,0;
    timer_ = this->create_wall_timer(500ms, std::bind(&Quadrotor::iteration, this));

    if (!load_params()) {
        RCLCPP_ERROR(this->get_logger(), "Could not load the drone parameters");
        rclcpp::shutdown();
    }
    //     // load init params
    // if (!load_init_vals()) {
    //     RCLCPP_ERROR(this->get_logger(), "Could not load the drone initial values");
    //     return false;
    // }

    // logger = this->get_logger();
    // while (marker_pub.getNumSubscribers() < 1) {
        // RCLCPP_INFO(logger, "Waiting for subscriber");
    //     ros::Duration(1).sleep();
    // }
    // controller = std::make_shared<ControllerImpl>(params, init_vals, gains, dt);





}
    void setState(State m_state_) {
    this->m_state = m_state_;
}

    // State getState();

    // bool initialize(double dt);

    // void desired_pos_cb(const geometry_msgs::msg::Point &pt);

    // xd should be in NED frame and so does dynamics and controller.

    // void run() {
    //     // ros::Timer timer = nh.createTimer(ros::Duration(1 / frequency), &Quadrotor::iteration, this);
    //     ros::spin();
    // }


private:

    State m_state;
    double sim_time, tau, dt, frequency;
    int robot_id;
    std::string worldframe, localframe, robot_link_name;
    gains_t gains_;
    std::vector<Eigen::Vector3d> traj_piece;
    int xd_it;
    Vector3d u, target_pos;
    state_space_t x0, xd0;
    desired_state_t dss;
    bool set_init_target;
    geometry_msgs::msg::Point target_next;
    bool set_next_target = false;

    std::shared_ptr<ControllerImpl> controller;
    state_space_t state_space;
    // rclcpp::Logger logger;
    std::shared_ptr<DynamicsProvider> dynamics_;
    params_t params_;
    init_vals_t init_vals;
    rclcpp::TimerBase::SharedPtr timer_;

    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, goal_pub, state_pub;
    // visualization_msgs::msg::Marker m, g;
    // std::vector<geometry_msgs::msg::Point> points;
    // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_state_sub;
    // vector<Vector3d> positions;

    // bool quad_initialized = false;

    // rclcpp::TimerBase::SharedPtr timer;

    // void initPaths();
    // void set_state_space();

bool load_params() {
    stringstream ss;
    ss << "/robot_" << to_string(this->robot_id);
    string robot_name = ss.str();

    vector<double> gains(4);
    this->get_parameter(robot_name + ".drone.model.gravity", params_.gravity);
    this->get_parameter(robot_name + ".drone.model.m", params_.mass);

    this->gains_ = {gains[0], gains[1], gains[2], gains[3]};
    this->get_parameter(robot_name + ".drone.controller.gains.kx", gains[0]);
    this->get_parameter(robot_name + ".drone.controller.gains.kv", gains[1]);
    this->get_parameter(robot_name + ".drone.controller.gains.kr", gains[2]);
    this->get_parameter(robot_name + ".drone.controller.gains.komega", gains[3]);
    
    vector<double> J_(3);
    this->get_parameter(robot_name + ".drone.model.J.jxx", J_[0]);
    this->get_parameter(robot_name + ".drone.model.J.jyy", J_[1]);
    this->get_parameter(robot_name + ".drone.model.J.jzz", J_[2]);
    
    params_.J << J_[0], 0, 0,
                    0, J_[1], 0,
                    0, 0, J_[2];
    params_.J_inv = params_.J.inverse();

    // Load mass
    params_.F = 0;
    params_.M = Vector3d(0, 0, 0);

    RCLCPP_DEBUG(this->get_logger(), "Loaded control parameters");
    return true;
}
    // void send_transform();
    // void publish_path();
    // void publish_state();

    void move(const desired_state_t &d_state) {
    control_out_t control = controller->get_control(dynamics_->get_state(), d_state);
    // dynamics->update(control, sim_time);
    RCLCPP_INFO(this->get_logger(), "IN MOVE");
    // updating the model on rviz
    // set_state_space();
    // send_transform();
    // dynamics->reset_dynamics();
    sim_time += dt;
}

    void iteration() {
        Vector3d xd = simulator_utils::ned_nwu_rotation(Vector3d{0,0,0});
        Vector3d b1d(1, 0, 0);
        desired_state_t dss = {xd, b1d};
        this->move(dss);
        RCLCPP_INFO(this->get_logger(), "Iteration: ");

        // this->publish_path();
        // this->publish_state();
    }

};

#endif
