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
        :Node("robot_"+to_string(robot_id)), frequency(frequency), robot_id(robot_id) {
    sim_time = 0;
    this->dt = 1/frequency;
    // this->initialize(1 / frequency);
    m_state = State::Idle;

    this->u << 0,0,0;
    timer_ = this->create_wall_timer(2ms, std::bind(&Quadrotor::iteration, this));
    
    RCLCPP_INFO(this->get_logger(), "Loading parameters");

    if (!load_params()) {
        RCLCPP_ERROR(this->get_logger(), "Could not load the drone parameters");
        rclcpp::shutdown();
    }

    
    // logger = this->get_logger();
    // while (marker_pub.getNumSubscribers() < 1) {
        // RCLCPP_INFO(logger, "Waiting for subscriber");
    //     ros::Duration(1).sleep();
    // }
    controller = std::make_shared<Geometric_Controller>(params_, gains_, dt);
    dynamics = std::make_shared<DynamicsProvider>(params_, init_vals);
    this->setState(State::Autonomous);

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

    std::shared_ptr<Geometric_Controller> controller;
    state_space_t state_space;
    // rclcpp::Logger logger;
    std::shared_ptr<DynamicsProvider> dynamics;
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

    declare_parameter("controller_gains", std::vector<double>(4, 0.0));
    vector<double> gains = get_parameter("controller_gains").as_double_array();
    
    declare_parameter("model.m", 0.00);
    declare_parameter("model.gravity", 9.81);
    declare_parameter("model.d", 0.08);
    declare_parameter("model.ctf", 0.0037);
    declare_parameter("model.J", std::vector<double>(3, 0.0));
    params_.mass = get_parameter("model.m").as_double();
    params_.gravity = get_parameter("model.gravity").as_double();

    vector<double> J_ = get_parameter("model.J").as_double_array();
    params_.J <<    J_[0], 0, 0,
                    0, J_[1], 0,
                    0, 0, J_[2];
    params_.J_inv = params_.J.inverse();
    params_.F = 0;
    params_.M = Vector3d(0, 0, 0);

    declare_parameter(to_string(robot_id)+".position", std::vector<double>(3, 0.0));
    declare_parameter(to_string(robot_id)+".velocity", std::vector<double>(3, 0.0));
    declare_parameter(to_string(robot_id)+".rotation", std::vector<double>(3, 0.0));
    declare_parameter(to_string(robot_id)+".omega", std::vector<double>(3, 0.0));
    vector<double> pos_ = get_parameter(to_string(robot_id)+".position").as_double_array();
    vector<double> vel_ = get_parameter(to_string(robot_id)+".velocity").as_double_array();
    vector<double> rot_ = get_parameter(to_string(robot_id)+".rotation").as_double_array();
    vector<double> omega_ = get_parameter(to_string(robot_id)+".omega").as_double_array();
    init_vals.position = Vector3d(pos_.data());
    init_vals.velocity << Vector3d(vel_.data());
    init_vals.R << Matrix3d(rot_.data());
    init_vals.omega << Vector3d(omega_.data());

    RCLCPP_INFO(this->get_logger(), "Param: Gravity: %s ",to_string(params_.gravity).c_str());
    RCLCPP_INFO(this->get_logger(), "Param: Mass: %s ",to_string(params_.mass).c_str());
    RCLCPP_INFO(this->get_logger(), "Param: MOI: %6f %6f %6f",J_[0], J_[1], J_[2]);
    RCLCPP_INFO(this->get_logger(), "Param: Controller Gains: %4f %4f %4f %4f", gains[0], gains[1],
                                                                                gains[2], gains[3]);
    RCLCPP_INFO(this->get_logger(), "Loaded control parameters");
    return true;
}
    // void send_transform();
    // void publish_path();
    // void publish_state();

    void move(const desired_state_t &d_state) {
        state_space_t s = dynamics->get_state();
        control_out_t control = controller->get_control(dynamics->get_state(), d_state);
        dynamics->update(control, sim_time);
        RCLCPP_INFO(this->get_logger(), "control %4f %4f %4f", control.F, control.M[0], control.M[1]);
        // updating the model on rviz
        set_state_space();
        // send_transform();
        dynamics->reset_dynamics();
        RCLCPP_INFO(this->get_logger(), "position: %4f %4f %4f", s.position[0], 
                                        s.position[1], s.position[2]);
        RCLCPP_INFO(this->get_logger(), "rotation: %4f %4f %4f", state_space.R(0, 0), 
                                        state_space.R(1,1), state_space.R(2,2));
        sim_time += dt;
}

    void set_state_space() {
        state_space_t ss = dynamics->get_state();
        state_space.position = simulator_utils::ned_nwu_rotation(ss.position);
        state_space.R = simulator_utils::ned_nwu_rotation(ss.R);
        state_space.velocity = simulator_utils::ned_nwu_rotation(ss.velocity);
        state_space.omega = simulator_utils::ned_nwu_rotation(ss.omega);
    }

    void iteration() {
        Vector3d xd = simulator_utils::ned_nwu_rotation(Vector3d{2,2,-2.5});
        Vector3d b1d(1, 0, 0);
        desired_state_t dss = {xd, b1d};
        this->move(dss);
        // RCLCPP_INFO(this->get_logger(), "Iteration: ");

        // this->publish_path();
        // this->publish_state();
    }

};

#endif
