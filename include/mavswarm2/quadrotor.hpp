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

#include "mav_trajectory_generation_ros2/polynomial_optimization_linear.h"
#include "mav_trajectory_generation_ros2/trajectory.h"
#include "mav_trajectory_generation_ros2/motion_defines.h"
#include "mav_trajectory_generation_ros2/vertex.h"

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
        set_next_target = false;

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
    bool          set_next_target;
    geometry_msgs::msg::Point target_next;

    mav_trajectory_generation::Trajectory traj_;

    std::shared_ptr<Geometric_Controller>              controller;
    std::shared_ptr<DynamicsProvider>                  dynamics;
    std::unique_ptr<tf2_ros::TransformBroadcaster>     tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_state_sub_;
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
    // Trajectory optimisation
    // -----------------------------------------------------------------------
    mav_trajectory_generation::Trajectory get_opt_traj(const opt_t &ps, const Vector3d &pe)
    {
        namespace mtg = mav_trajectory_generation;
        mtg::Vertex::Vector vertices;
        mtg::Vertex v_s(3), v_e(3);
        const int deriv = mtg::derivative_order::JERK;

        v_s.addConstraint(mtg::derivative_order::POSITION,     ps.position);
        v_s.addConstraint(mtg::derivative_order::VELOCITY,     ps.velocity);
        v_s.addConstraint(mtg::derivative_order::ACCELERATION, ps.acceleration);
        v_e.makeStartOrEnd(pe, deriv);

        vertices.push_back(v_s);
        vertices.push_back(v_e);

        const double v_max = 1.0, a_max = 4.0;
        std::vector<double> segment_times = mtg::estimateSegmentTimes(vertices, v_max, a_max);

        mtg::PolynomialOptimization<8> opt(3);
        opt.setupFromVertices(vertices, segment_times, deriv);
        opt.solveLinear();

        mtg::Trajectory trajectory;
        opt.getTrajectory(&trajectory);
        return trajectory;
    }

    // Receding horizon replanning: replan from current dynamic state to pending target.
    void do_rhp()
    {
        Vector3d pt(target_next.x, target_next.y, target_next.z);
        state_space_t ss = dynamics->get_state();
        opt_t wp = {ss.position, ss.velocity, ss.acceleration, Vector3d(0, 0, 0)};
        traj_          = get_opt_traj(wp, pt);
        set_next_target = false;
        tau             = 0;
        RCLCPP_INFO(this->get_logger(), "Replanned trajectory, duration: %.2f s", traj_.getMaxTime());
    }

    // -----------------------------------------------------------------------
    // Goal subscription
    // -----------------------------------------------------------------------
    void desired_pos_cb(const geometry_msgs::msg::Point::SharedPtr pt)
    {
        Vector3d p1(pt->x, pt->y, pt->z);
        if ((target_pos - p1).norm() < 0.2) return;

        if (!set_init_target) {
            set_init_target = true;
            Vector3d zero(0, 0, 0);
            opt_t ps = {target_pos, zero, zero, zero};
            traj_      = get_opt_traj(ps, p1);
            target_pos = p1;
            RCLCPP_INFO(this->get_logger(), "Set initial trajectory to [%.2f %.2f %.2f]",
                        p1[0], p1[1], p1[2]);
        } else {
            target_next    = *pt;
            target_pos     = p1;
            set_next_target = true;
            RCLCPP_INFO(this->get_logger(), "Queued new target [%.2f %.2f %.2f]",
                        p1[0], p1[1], p1[2]);
        }
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
        dynamics->reset_dynamics();
        sim_time += dt;
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
            if (set_next_target) {
                do_rhp();
            }
            xd = traj_.evaluate(tau, mav_trajectory_generation::derivative_order::POSITION);
            // advance, clamping to the trajectory end so we hold exactly at the
            // zero-velocity endpoint instead of freezing a fraction of dt short.
            tau = std::min(tau + dt, traj_.getMaxTime());
        }

        desired_state_t dss = {xd, b1d};
        this->move(dss);
    }
};

#endif
