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

// ---------------------------------------------------------------------------
// 
// The trajectory is a discretised double integrator (states = position +
// velocity, input = acceleration). We minimise the control effort while
// honouring the boundary conditions (start state, goal at rest) and the
// per-axis velocity / acceleration limits.
//
// The pairwise keep-out  ||p_i[k] - p_j[k]|| >= R  is non-convex. Following the
// paper it is convexified about the previous iterate into the affine
// separating-hyperplane constraint
//        xi/||xi|| . (p_i[k] - p_j[k]) >= R,   xi = p_i^prev[k] - p_j[k],
// and the resulting convex problem is solved; re-linearising and re-solving a
// few times is the SCP loop. (Solved here as a weighted least-squares / penalty
// problem with Eigen, so no external QP solver is required.)
//
// This module is SELF-CONTAINED: it owns the ROS publisher/subscribers used to
// exchange predicted trajectories between robots, so the quadrotor code does not
// need to know about neighbour state at all. Drop-in for the old trajectory
// optimiser: set_goal(...) once, then call reference() every control tick.
// ---------------------------------------------------------------------------

#ifndef MAVSWARM2_SCP_PLANNER_HPP
#define MAVSWARM2_SCP_PLANNER_HPP

#include <vector>
#include <utility>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace scp {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

struct SCPParams {
    double v_max       = 1.0;    // m/s   velocity limit
    double a_max       = 2.0;    // m/s^2 acceleration limit
    double R           = 0.5;    // m     collision keep-out radius
    double downwash    = 0.5;    // ellipsoid z-scaling (1 = isotropic sphere)
    double activation  = 0.1;    // m     outward overshoot past the shell
    double dt_grid     = 0.15;   // s     trajectory discretisation step
    int    K_min       = 10;     // min / max number of segments
    int    K_max       = 60;
    int    max_iters   = 8;      // SCP re-linearisations
    int    replan_every= 30;     // control ticks between receding-horizon replans
    // penalty weights
    double w_smooth    = 10.0;    // control effort (acceleration)
    double w_init      = 1.0e6;  // start state (hard)
    double w_vel       = 1.0e2;  // start velocity
    double w_goal      = 1.0e3;  // terminal position = goal
    double w_term_vel  = 1.0e2;  // terminal velocity = 0
    double w_vlim      = 1.0e3;  // velocity limit (one-sided)
    double w_alim      = 5.0e2;  // acceleration limit (one-sided)
    double w_col       = 1.0e3;  // collision keep-out
};

// ---------------------------------------------------------------------------
// Discrete trajectory with a smooth (Catmull-Rom) interpolant.
// ---------------------------------------------------------------------------
class Trajectory {
public:
    Trajectory() : h_(0.1), T_(0.0) {}
    Trajectory(std::vector<Vector3d> pts, double h)
        : pts_(std::move(pts)), h_(h),
          T_(h * static_cast<double>(pts_.empty() ? 0 : pts_.size() - 1)) {}

    double getMaxTime() const { return T_; }
    bool   empty()      const { return pts_.empty(); }

    // derivative_order: 0 = position, 1 = velocity, 2 = acceleration
    Vector3d evaluate(double t, int derivative_order = 0) const
    {
        const int n = static_cast<int>(pts_.size());
        if (n == 0) return Vector3d::Zero();
        if (n == 1) return (derivative_order == 0) ? pts_[0] : Vector3d::Zero();

        if (t < 0.0) t = 0.0;
        if (t > T_)  t = T_;
        int i = static_cast<int>(std::floor(t / h_));
        if (i > n - 2) i = n - 2;
        const double u = (t - i * h_) / h_;

        const Vector3d &P1 = pts_[i];
        const Vector3d &P2 = pts_[i + 1];
        const Vector3d P0 = (i - 1 >= 0)     ? pts_[i - 1] : (2.0 * P1 - P2);
        const Vector3d P3 = (i + 2 <= n - 1) ? pts_[i + 2] : (2.0 * P2 - P1);

        const Vector3d a =  2.0 * P1;
        const Vector3d b = -P0 + P2;
        const Vector3d c =  2.0 * P0 - 5.0 * P1 + 4.0 * P2 - P3;
        const Vector3d d = -P0 + 3.0 * P1 - 3.0 * P2 + P3;

        if (derivative_order == 0) return 0.5 * (a + b * u + c * (u * u) + d * (u * u * u));
        if (derivative_order == 1) return 0.5 * (b + 2.0 * c * u + 3.0 * d * (u * u)) / h_;
        return 0.5 * (2.0 * c + 6.0 * d * u) / (h_ * h_);
    }

private:
    std::vector<Vector3d> pts_;
    double h_;
    double T_;
};

// ---------------------------------------------------------------------------
// Per-robot SCP planner. Owns the neighbour-trajectory communication.
// ---------------------------------------------------------------------------
class ScpPlanner {
public:
    ScpPlanner(rclcpp::Node *node, int robot_id, double control_dt,
               const SCPParams &prm = SCPParams())
        : node_(node), robot_id_(robot_id), dt_(control_dt), prm_(prm)
    {
        if (!node_->has_parameter("num_robots"))
            node_->declare_parameter("num_robots", 10);
        if (!node_->has_parameter("safety_radius"))
            node_->declare_parameter("safety_radius", prm_.R);
        num_robots_ = node_->get_parameter("num_robots").as_int();
        prm_.R      = node_->get_parameter("safety_radius").as_double();

        path_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "robot_" + std::to_string(robot_id_) + "/scp_path", 10);
        for (int j = 0; j < num_robots_; ++j) {
            if (j == robot_id_) continue;
            auto sub = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
                "robot_" + std::to_string(j) + "/scp_path", 10,
                [this, j](const std_msgs::msg::Float32MultiArray::SharedPtr m) { on_path(j, m); });
            path_subs_.push_back(sub);
        }
    }

    // Set / update the goal (NED). Plans immediately from the supplied state.
    void set_goal(const Vector3d &goal, const Vector3d &p0, const Vector3d &v0)
    {
        goal_     = goal;
        has_goal_ = true;
        traj_     = solve(p0, v0, goal_);
        t_play_   = 0.0;
        ticks_    = 0;
        publish_path();
    }

    // Desired position for this control tick (advances playback, replans as needed).
    Vector3d reference()
    {
        if (!has_goal_) return goal_;

        if (++ticks_ >= prm_.replan_every && !traj_.empty()) {
            // receding horizon: re-solve from the current reference (keeps xd smooth)
            const Vector3d p0 = traj_.evaluate(t_play_, 0);
            const Vector3d v0 = traj_.evaluate(t_play_, 1);
            traj_   = solve(p0, v0, goal_);
            t_play_ = 0.0;
            ticks_  = 0;
        }
        const Vector3d xd = traj_.evaluate(t_play_, 0);
        t_play_ = std::min(t_play_ + dt_, traj_.getMaxTime());
        publish_path();
        return xd;
    }

private:
    // ----- neighbour communication -----------------------------------------
    void on_path(int j, const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) return;
        const double dtj = msg->data[0];
        std::vector<Vector3d> s;
        for (size_t i = 1; i + 2 < msg->data.size(); i += 3)
            s.emplace_back(msg->data[i], msg->data[i + 1], msg->data[i + 2]);
        neighbor_paths_[j] = {dtj, std::move(s)};
    }

    void publish_path()
    {
        std_msgs::msg::Float32MultiArray msg;
        const double dtp = prm_.dt_grid;
        const int    M   = 20;
        msg.data.push_back(static_cast<float>(dtp));
        for (int m = 0; m <= M; ++m) {
            Vector3d q = traj_.empty() ? goal_
                                       : traj_.evaluate(std::min(t_play_ + m * dtp, traj_.getMaxTime()), 0);
            msg.data.push_back(static_cast<float>(q[0]));
            msg.data.push_back(static_cast<float>(q[1]));
            msg.data.push_back(static_cast<float>(q[2]));
        }
        path_pub_->publish(msg);
    }

    // neighbour j's predicted position at relative time t (clamped)
    Vector3d neighbor_at(int j, double t) const
    {
        const auto it = neighbor_paths_.find(j);
        if (it == neighbor_paths_.end() || it->second.second.empty())
            return Vector3d::Constant(1e9);          // unknown -> effectively absent
        const double dtj = it->second.first;
        const auto  &s   = it->second.second;
        const double fm  = t / std::max(dtj, 1e-6);
        const int    m   = static_cast<int>(std::floor(fm));
        if (m >= static_cast<int>(s.size()) - 1) return s.back();
        const double a = fm - m;
        return (1.0 - a) * s[m] + a * s[m + 1];
    }

    // ----- the SCP solve ---------------------------------------------------
    Trajectory solve(const Vector3d &p0, const Vector3d &v0, const Vector3d &goal)
    {
        const double dist = (goal - p0).norm();
        // horizon sized to actually reach the goal at <= v_max (no overspeed)
        const double T = std::max(2.0, 1.2 * dist / std::max(prm_.v_max, 1e-3));
        const int    K = std::clamp(static_cast<int>(std::round(T / prm_.dt_grid)),
                                    prm_.K_min, prm_.K_max);
        const double h  = T / K;
        const int    nv = 3 * (K + 1);

        std::vector<Vector3d> p(K + 1);
        for (int k = 0; k <= K; ++k)
            p[k] = p0 + (static_cast<double>(k) / K) * (goal - p0);

        auto idx = [](int k, int d) { return 3 * k + d; };
        const double cz = prm_.downwash;

        for (int iter = 0; iter < prm_.max_iters; ++iter) {
            MatrixXd H = MatrixXd::Zero(nv, nv);
            VectorXd c = VectorXd::Zero(nv);
            auto addRes = [&](double w, const std::vector<std::pair<int, double>> &t, double tgt) {
                for (const auto &ta : t) {
                    c(ta.first) += w * ta.second * tgt;
                    for (const auto &tb : t) H(ta.first, tb.first) += w * ta.second * tb.second;
                }
            };

            // boundary conditions
            for (int d = 0; d < 3; ++d) addRes(prm_.w_init, {{idx(0, d), 1.0}}, p0[d]);
            for (int d = 0; d < 3; ++d) addRes(prm_.w_vel, {{idx(1, d), 1.0}, {idx(0, d), -1.0}}, h * v0[d]);
            for (int d = 0; d < 3; ++d) addRes(prm_.w_goal, {{idx(K, d), 1.0}}, goal[d]);
            for (int d = 0; d < 3; ++d) addRes(prm_.w_term_vel, {{idx(K, d), 1.0}, {idx(K - 1, d), -1.0}}, 0.0);

            // control effort: minimise acceleration (double-integrator input)
            for (int k = 1; k <= K - 1; ++k)
                for (int d = 0; d < 3; ++d)
                    addRes(prm_.w_smooth, {{idx(k + 1, d), 1.0}, {idx(k, d), -2.0}, {idx(k - 1, d), 1.0}}, 0.0);

            // velocity limit (one-sided)
            for (int k = 0; k <= K - 1; ++k) {
                const Vector3d seg = p[k + 1] - p[k];
                if (seg.norm() / h > prm_.v_max && seg.norm() > 1e-9) {
                    const Vector3d s = seg / seg.norm();
                    addRes(prm_.w_vlim,
                           {{idx(k + 1, 0), s[0]}, {idx(k, 0), -s[0]},
                            {idx(k + 1, 1), s[1]}, {idx(k, 1), -s[1]},
                            {idx(k + 1, 2), s[2]}, {idx(k, 2), -s[2]}}, h * prm_.v_max);
                }
            }

            // acceleration limit (one-sided)
            for (int k = 1; k <= K - 1; ++k) {
                const Vector3d acc = p[k + 1] - 2.0 * p[k] + p[k - 1];
                if (acc.norm() / (h * h) > prm_.a_max && acc.norm() > 1e-9) {
                    const Vector3d s = acc / acc.norm();
                    addRes(prm_.w_alim,
                           {{idx(k + 1, 0), s[0]}, {idx(k, 0), -2.0 * s[0]}, {idx(k - 1, 0), s[0]},
                            {idx(k + 1, 1), s[1]}, {idx(k, 1), -2.0 * s[1]}, {idx(k - 1, 1), s[1]},
                            {idx(k + 1, 2), s[2]}, {idx(k, 2), -2.0 * s[2]}, {idx(k - 1, 2), s[2]}},
                           h * h * prm_.a_max);
                }
            }

            // collision: separating hyperplane vs higher-priority neighbours (lower id).
            // Prioritised planning: a robot only avoids smaller-id robots, which
            // breaks the symmetry that causes reciprocal dodging / deadlock.
            const double shell = prm_.R;
            for (int j = 0; j < robot_id_; ++j) {
                if (neighbor_paths_.find(j) == neighbor_paths_.end()) continue;
                for (int k = 1; k <= K; ++k) {
                    const Vector3d o = neighbor_at(j, k * h);
                    const Vector3d e = p[k] - o;
                    const double ne = std::sqrt(e[0]*e[0] + e[1]*e[1] + cz*cz*e[2]*e[2]);
                    if (ne < shell && ne > 1e-6) {
                        const Vector3d a(e[0] / ne, e[1] / ne, cz * cz * e[2] / ne);
                        addRes(prm_.w_col,
                               {{idx(k, 0), a[0]}, {idx(k, 1), a[1]}, {idx(k, 2), a[2]}},
                               shell + prm_.activation + a.dot(o));
                    }
                }
            }

            for (int i = 0; i < nv; ++i) H(i, i) += 1e-9;
            const VectorXd x = H.ldlt().solve(c);

            double step = 0.0;
            for (int k = 0; k <= K; ++k) {
                const Vector3d pk(x(idx(k, 0)), x(idx(k, 1)), x(idx(k, 2)));
                step = std::max(step, (pk - p[k]).norm());
                p[k] = pk;
            }
            if (step < 1e-3) break;
        }
        return Trajectory(std::move(p), h);
    }

    rclcpp::Node *node_;
    int    robot_id_;
    double dt_;
    SCPParams prm_;
    int    num_robots_ = 10;

    bool      has_goal_ = false;
    Vector3d  goal_     = Vector3d::Zero();
    Trajectory traj_;
    double    t_play_ = 0.0;
    int       ticks_  = 0;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr path_pub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr> path_subs_;
    std::map<int, std::pair<double, std::vector<Vector3d>>> neighbor_paths_;
};

} // namespace scp

#endif // MAVSWARM2_SCP_PLANNER_HPP
