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
#include "simulator_interfaces/msg/geometric_ctrl.hpp"

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
    dt = 1/frequency;
    RCLCPP_INFO(this->get_logger(), "Initialing drone: %d %3f", robot_id, frequency);

    m_state = State::Idle;

    timer_ = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&Quadrotor::iteration, this));
    
    RCLCPP_INFO(this->get_logger(), "Loading parameters");

    if (!load_params()) {
        RCLCPP_ERROR(this->get_logger(), "Could not load the drone parameters");
        rclcpp::shutdown();
    }

    d_state_sub = this->create_subscription<simulator_interfaces::msg::GeometricCtrl>(
      "robot_"+to_string(this->robot_id)+"/desired_state", 10, std::bind(&Quadrotor::desired_state_cb, this, std::placeholders::_1));
    
    // logger = this->get_logger();
    // while (marker_pub.getNumSubscribers() < 1) {
        // RCLCPP_INFO(logger, "Waiting for subscriber");
    //     ros::Duration(1).sleep();
    // }
    controller = std::make_shared<Geometric_Controller>(params_, gains, dt);
    dynamics = std::make_shared<DynamicsProvider>(params_, x0);
    set_state_space();
    this->setState(State::Autonomous);

}
    void setState(State m_state_) {
    this->m_state = m_state_;
}

    // State getState();

    // bool initialize(double dt);

    // void desired_pos_cb(const geometry_msgs::msg::Point &pt);

    // xd should be in NED frame and so does dynamics and controller.



private:

    State m_state;
    double sim_time, dt, frequency;
    int robot_id;
    std::string worldframe, localframe, robot_link_name;
    gains_t gains;
    state_space_t x0;
    // geometry_msgs::msg::Point target_next;
    // bool set_next_target = false;

    std::shared_ptr<Geometric_Controller> controller;
    state_space_t state_space;
    // rclcpp::Logger logger;
    std::shared_ptr<DynamicsProvider> dynamics;
    params_t params_;
    init_vals_t init_vals;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<simulator_interfaces::msg::GeometricCtrl>::SharedPtr d_state_sub;


void desired_state_cb(const simulator_interfaces::msg::GeometricCtrl::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", to_string(msg->heading.x).c_str());
    }
    

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
    vector<double> gains_ = get_parameter("controller_gains").as_double_array();
    
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

    declare_parameter("position", std::vector<double>(3, 0.0));
    declare_parameter("velocity", std::vector<double>(3, 0.0));
    declare_parameter("rotation", std::vector<double>(3, 0.0));
    declare_parameter("omega", std::vector<double>(3, 0.0));
    vector<double> pos_ = get_parameter("position").as_double_array();
    vector<double> vel_ = get_parameter("velocity").as_double_array();
    vector<double> rot_ = get_parameter("rotation").as_double_array();
    vector<double> omega_ = get_parameter("omega").as_double_array();
    x0.position = simulator_utils::ned_nwu_rotation(Vector3d(pos_.data()));
    x0.velocity << simulator_utils::ned_nwu_rotation(Vector3d(vel_.data()));
    x0.R << simulator_utils::ned_nwu_rotation(Matrix3d(rot_.data()));
    x0.omega << simulator_utils::ned_nwu_rotation(Vector3d(omega_.data()));
    gains = {gains_[0], gains_[1], gains_[3], gains_[3]};
    RCLCPP_INFO(this->get_logger(), "Init Params: : %4f %4f %4f ", x0.position[0], x0.position[1],x0.position[2]);

    RCLCPP_INFO(this->get_logger(), "Param: Gravity: %s ",to_string(params_.gravity).c_str());
    RCLCPP_INFO(this->get_logger(), "Param: Mass: %s ",to_string(params_.mass).c_str());
    RCLCPP_INFO(this->get_logger(), "Param: MOI: %6f %6f %6f",J_[0], J_[1], J_[2]);
    RCLCPP_INFO(this->get_logger(), "Param: Controller Gains: %4f %4f %4f %4f", gains.kx, gains.kv,
                                                                                gains.kr, gains.komega);
    RCLCPP_INFO(this->get_logger(), "Loaded control parameters");
    return true;
}
    // void send_transform();
    // void publish_path();
    // void publish_state();

    void move(const desired_state_t &d_state) {
        state_space_t s = dynamics->get_state();
        control_out_t control = controller->get_control(s, d_state);
        dynamics->update(control, sim_time);

        // updating the model on rviz
        set_state_space();
        // send_transform();

        dynamics->reset_dynamics();
        RCLCPP_DEBUG(this->get_logger(), "%2f position %4f %4f %4f", sim_time, s.position[0], s.position[1], s.position[2]);
        RCLCPP_DEBUG(this->get_logger(), "%2f control %4f %4f %4f %4f", sim_time, control.F, control.M[0], control.M[1], control.M[2]);
       
        // RCLCPP_INFO(this->get_logger(), "position: %4f %4f %4f", s.position[0], 
        //                                 s.position[1], s.position[2]);
        // RCLCPP_INFO(this->get_logger(), "rotation: %4f %4f %4f", state_space.R(0, 0), 
        //                                 state_space.R(1,1), state_space.R(2,2));
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
        Vector3d xd = simulator_utils::ned_nwu_rotation(x0.position);
        Vector3d b1d(1, 0, 0);
        desired_state_t dss = {xd, b1d};

        this->move(dss);

        // this->publish_path();
        // this->publish_state();
    }

};

#endif
