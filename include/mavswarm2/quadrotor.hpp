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

#include <controller/controllerImpl.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include "dynamics_provider.h"
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "simulator_interfaces/msg/waypoint.hpp"
#include "trajectory_t.h"

#include "std_msgs/msg/float32_multi_array.hpp"

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

class Quadrotor : public rclcpp::Node {
public:
    Quadrotor(int robot_id, double frequency)
        :Node("robot_"+to_string(robot_id)), frequency(frequency),robot_id(robot_id) {
    sim_time = 0;
    this->dt = 1/frequency;
    // this->initialize(1 / frequency);
    this->u << 0,0,0;

    // while (marker_pub.getNumSubscribers() < 1) {
    //     RCLCPP_INFO("Waiting for subscriber");
    //     ros::Duration(1).sleep();
    // }
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
    gains_t gains;
    std::vector<Eigen::Vector3d> traj_piece;
    int xd_it;
    Vector3d u, target_pos;
    state_space_t x0, xd0;
    desired_state_t dss;
    bool set_init_target;
    geometry_msgs::msg::Point target_next;
    bool set_next_target = false;

    ControllerImpl *controller;
    state_space_t state_space;

    DynamicsProvider *dynamics;
    params_t params;
    init_vals_t init_vals;

    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, goal_pub, state_pub;
    // visualization_msgs::msg::Marker m, g;
    // std::vector<geometry_msgs::msg::Point> points;
    // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_state_sub;
    // vector<Vector3d> positions;

    // bool quad_initialized = false;

    // rclcpp::TimerBase::SharedPtr timer;

    // void initPaths();
    // void set_state_space();
    // bool load_params();
    // bool load_init_vals();
    // void send_transform();
    // void publish_path();
    // void publish_state();

    void move(const desired_state_t &d_state) {
    control_out_t control = controller->get_control(dynamics->get_state(), d_state);
    // dynamics->update(control, sim_time);
    RCLCPP_DEBUG(this->get_logger(), "got control: ");
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
        // this->publish_path();
        // this->publish_state();
    }

};

#endif
