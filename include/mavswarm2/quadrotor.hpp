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
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "scp/scp_planner.hpp"

#include "trajectory_t.hpp"
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

class Quadrotor : public rclcpp::Node {
public:
    Quadrotor(int robot_id, double frequency)
        : Node("robot_" + to_string(robot_id)), frequency(frequency), robot_id(robot_id)
    {
        sim_time = 0;
        tau = 0;
        dt = 1.0 / frequency;
        m_state = State::Idle;
        set_init_target = false;

        RCLCPP_INFO(this->get_logger(), "dt: %4f ", dt);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&Quadrotor::iteration, this));

        RCLCPP_INFO(this->get_logger(), "Loading parameters");
        if (!load_params()) {
            RCLCPP_ERROR(this->get_logger(), "Could not load the drone parameters");
            rclcpp::shutdown();
            return;
        }

        controller = std::make_shared<Geometric_Controller>(params_, gains, dt);
        dynamics   = std::make_shared<DynamicsProvider>(params_, init_vals);
        set_state_space();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        worldframe      = "map";
        // per-robot TF child frame so multiple drones don't all broadcast map->base_link
        robot_link_name = "robot_" + std::to_string(robot_id) + "/base_link";

        // initialise hover target to the drone's starting position (NWU)
        target_pos = simulator_utils::ned_nwu_rotation(init_vals.position);

        // per-robot desired_state topic, e.g. /mavswarm2/robot_3/desired_state
        std::string desired_topic = "robot_" + std::to_string(robot_id) + "/desired_state";
        desired_state_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            desired_topic, 10,
            std::bind(&Quadrotor::desired_pos_cb, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to goal topic: %s", desired_topic.c_str());

        // SCP planner owns neighbour-trajectory communication; quad just feeds
        // it goals and reads back the reference each tick.
        scp_planner_ = std::make_shared<scp::ScpPlanner>(this, robot_id, dt);

        // trajectory trail: a LINE_STRIP marker (last N points) on this robot's
        // own topic, e.g. /mavswarm2/robot_3/trajectory.
        path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "robot_" + std::to_string(robot_id) + "/trajectory", 10);
        // distinct colour per robot from the id, via the golden-ratio hue trick:
        // well-spread, unique colours for ANY number of robots (no fixed palette).
        hsv_to_rgb(std::fmod(robot_id * 0.6180339887f, 1.0f), 0.85f, 1.0f,
                   base_r_, base_g_, base_b_);
        path_marker_.header.frame_id = worldframe;        // "map"
        path_marker_.ns   = "trajectory";
        path_marker_.id   = robot_id;
        path_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker_.action = visualization_msgs::msg::Marker::ADD;
        path_marker_.scale.x = 0.03;                      // line width
        path_marker_.pose.orientation.w = 1.0;
        path_marker_.color.r = base_r_;
        path_marker_.color.g = base_g_;
        path_marker_.color.b = base_b_;
        path_marker_.color.a = 1.0;

        this->setState(State::Autonomous);
    }

    void setState(State m_state_) { this->m_state = m_state_; }

private:
    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------
    State  m_state;
    double sim_time, tau, dt, frequency;
    int    robot_id;
    std::string worldframe, robot_link_name;

    gains_t       gains;
    Vector3d      u, target_pos;
    state_space_t state_space;
    bool          set_init_target;

    std::shared_ptr<scp::ScpPlanner> scp_planner_;

    std::shared_ptr<Geometric_Controller>              controller;
    std::shared_ptr<DynamicsProvider>                  dynamics;
    std::unique_ptr<tf2_ros::TransformBroadcaster>     tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_state_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
    visualization_msgs::msg::Marker path_marker_;
    int   path_tick_ = 0;
    float base_r_ = 1.0f, base_g_ = 1.0f, base_b_ = 1.0f;
    rclcpp::TimerBase::SharedPtr                       timer_;

    params_t    params_;
    init_vals_t init_vals;

    // -----------------------------------------------------------------------
    // Parameter loading
    // -----------------------------------------------------------------------
    bool load_params() {
        declare_parameter("controller_gains", std::vector<double>(4, 0.0));
        vector<double> gains_ = get_parameter("controller_gains").as_double_array();

        declare_parameter("model.m",       0.00);
        declare_parameter("model.gravity", 9.81);
        declare_parameter("model.d",       0.08);
        declare_parameter("model.ctf",     0.0037);
        declare_parameter("model.J",       std::vector<double>(3, 0.0));

        params_.mass    = get_parameter("model.m").as_double();
        params_.gravity = get_parameter("model.gravity").as_double();
        vector<double> J_ = get_parameter("model.J").as_double_array();
        params_.J << J_[0], 0, 0,
                     0, J_[1], 0,
                     0, 0, J_[2];
        params_.J_inv = params_.J.inverse();
        params_.F = 0;
        params_.M = Vector3d(0, 0, 0);

        declare_parameter("position", std::vector<double>(3, 0.0));
        declare_parameter("velocity", std::vector<double>(3, 0.0));
        declare_parameter("rotation", std::vector<double>(9, 0.0));
        declare_parameter("omega",    std::vector<double>(3, 0.0));

        vector<double> pos_   = get_parameter("position").as_double_array();
        vector<double> vel_   = get_parameter("velocity").as_double_array();
        vector<double> rot_   = get_parameter("rotation").as_double_array();
        vector<double> omega_ = get_parameter("omega").as_double_array();

        // store raw values (NWU position/velocity, the NED-matched "upside-down" R).
        // ned_nwu_rotation is applied downstream (dynamics start state, iteration xd,
        // and broadcast) — converting here would double-convert and break frames.
        init_vals.position = Vector3d(pos_.data());
        init_vals.velocity = Vector3d(vel_.data());
        init_vals.R        = Matrix3d(rot_.data());
        init_vals.omega    = Vector3d(omega_.data());

        gains = {.kx = gains_[0], .kv = gains_[1], .kr = gains_[2], .komega = gains_[3]};

        RCLCPP_INFO(this->get_logger(), "Init Params: : %4f %4f %4f ",
                    init_vals.position[0], init_vals.position[1], init_vals.position[2]);
        RCLCPP_INFO(this->get_logger(), "Param: Gravity: %s ", to_string(params_.gravity).c_str());
        RCLCPP_INFO(this->get_logger(), "Param: Mass: %s ",    to_string(params_.mass).c_str());
        RCLCPP_INFO(this->get_logger(), "Param: MOI: %6f %6f %6f", J_[0], J_[1], J_[2]);
        RCLCPP_INFO(this->get_logger(), "Param: Controller Gains: %4f %4f %4f %4f",
                    gains.kx, gains.kv, gains.kr, gains.komega);
        RCLCPP_INFO(this->get_logger(), "Loaded control parameters");
        return true;
    }

    // -----------------------------------------------------------------------
    // Goal subscription -> hand the goal to the SCP planner (NED frame)
    // -----------------------------------------------------------------------
    void desired_pos_cb(const geometry_msgs::msg::Point::SharedPtr pt)
    {
        Vector3d p1(pt->x, pt->y, pt->z);
        if ((target_pos - p1).norm() < 0.2) return;
        target_pos      = p1;
        set_init_target = true;
        state_space_t ss = dynamics->get_state();
        scp_planner_->set_goal(p1, ss.position, ss.velocity);
        RCLCPP_INFO(this->get_logger(), "New goal [%.2f %.2f %.2f]", p1[0], p1[1], p1[2]);
    }

    // -----------------------------------------------------------------------
    // TF broadcasting
    // -----------------------------------------------------------------------
    void send_transform()
    {
        Vector3d position = state_space.position;
        Vector3d rpy      = simulator_utils::R2RPY(state_space.R);

        tf2::Quaternion q;
        q.setRPY(rpy[0], rpy[1], rpy[2]);
        if (std::isnan(q.x())) return;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = this->get_clock()->now();
        t.header.frame_id = worldframe;
        t.child_frame_id  = robot_link_name;
        t.transform.translation.x = position[0];
        t.transform.translation.y = position[1];
        t.transform.translation.z = position[2];
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        // RCLCPP_INFO(this->get_logger(), " %4f %4f %4f %4f %4f %4f %4f", position[0], position[1],position[2], q.x(), q.y(), q.z(), q.w());

        tf_broadcaster_->sendTransform(t);
    }

    // -----------------------------------------------------------------------
    // Control loop
    // -----------------------------------------------------------------------
    void move(const desired_state_t &d_state)
    {
        state_space_t s = dynamics->get_state();
        RCLCPP_DEBUG(this->get_logger(), "%2f position %4f %4f %4f",
                     sim_time, s.position[0], s.position[1], s.position[2]);

        control_out_t control = controller->get_control(s, d_state);
        Vector3d rpy      = simulator_utils::R2RPY(state_space.R);

        // RCLCPP_INFO(this->get_logger(), "%2f current vs desired %4f %4f %4f | %4f %4f %4f",
        //              sim_time, rpy[0], rpy[1], rpy[2], d_state.b1[0],d_state.b1[1], d_state.b1[2]);
        dynamics->update(control, sim_time);
        set_state_space();
        send_transform();
        publish_path();
        dynamics->reset_dynamics();
        sim_time += dt;
    }

    // Trajectory trail: append the current (map-frame) position to a LINE_STRIP
    // marker, keep only the last N points, and publish. Mirrors the ROS 1
    // Quadrotor::publish_path().
    void publish_path()
    {
        constexpr size_t kMaxPoints = 20;
        if (++path_tick_ % 5 != 0) return;     // sample ~20 Hz -> ~50 s of trail

        const Vector3d pos = state_space.position;   // NWU / map frame
        if (std::isnan(pos[0]) || std::isnan(pos[1]) || std::isnan(pos[2])) return;

        geometry_msgs::msg::Point p;
        p.x = pos[0]; p.y = pos[1]; p.z = pos[2];
        path_marker_.points.push_back(p);
        if (path_marker_.points.size() > kMaxPoints)
            path_marker_.points.erase(path_marker_.points.begin());

        // per-point alpha ramp: oldest point faint, newest solid. The tail then
        // fades out smoothly so dropping the front point is barely visible.
        const size_t n = path_marker_.points.size();
        path_marker_.colors.resize(n);
        for (size_t i = 0; i < n; ++i) {
            auto &c = path_marker_.colors[i];
            c.r = base_r_; c.g = base_g_; c.b = base_b_;
            c.a = 0.05f + 0.95f * (static_cast<float>(i + 1) / static_cast<float>(n));
        }

        path_marker_.header.stamp = this->get_clock()->now();
        path_marker_pub_->publish(path_marker_);
    }

    // HSV (h,s,v in [0,1]) -> RGB, for procedural per-robot trail colours.
    static void hsv_to_rgb(float h, float s, float v, float &r, float &g, float &b)
    {
        const float i = std::floor(h * 6.0f);
        const float f = h * 6.0f - i;
        const float p = v * (1.0f - s);
        const float q = v * (1.0f - f * s);
        const float t = v * (1.0f - (1.0f - f) * s);
        switch (static_cast<int>(i) % 6) {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            default: r = v; g = p; b = q; break;
        }
    }

    void set_state_space()
    {
        // convert the internal NED state back to map frame (NWU) for broadcasting
        state_space_t ss = dynamics->get_state();
        state_space.position = simulator_utils::ned_nwu_rotation(ss.position);
        state_space.R        = simulator_utils::ned_nwu_rotation(ss.R);
        state_space.velocity = simulator_utils::ned_nwu_rotation(ss.velocity);
        state_space.omega    = simulator_utils::ned_nwu_rotation(ss.omega);
    }

    void iteration()
    {
        Vector3d b1d(1, 0, 0);
        Vector3d xd = simulator_utils::ned_nwu_rotation(init_vals.position);

        if (set_init_target) {
            // SCP planner advances playback and replans (with collision avoidance) internally
            xd = scp_planner_->reference();
        }

        desired_state_t dss = {xd, b1d};
        this->move(dss);
    }
};

#endif
