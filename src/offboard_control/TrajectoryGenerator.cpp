
// 来自 https://www.cnblogs.com/21203-iHome/p/10505369.html
// S型速度规划

// #include "traj.h"
#include "TrajectoryGenerator.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>


TrajectoryGenerator::TrajectoryGenerator(double speed_factor, const std::array<double, 3> q_goal)
    : q_goal_(q_goal.data()) 
{
    dq_max_ *= speed_factor;
    ddq_max_ *= speed_factor;
    dq_max_sync_.setZero();
    q_start_.setZero();
    delta_q_.setZero();
    t_1_sync_.setZero();
    t_2_sync_.setZero();
    t_f_sync_.setZero();
    q_1_.setZero();
    dq.setZero(); // 初始化速度向量
    time_ = 0.0;
}


bool TrajectoryGenerator::calculateDesiredValues(double t, Vector3d* delta_q_d)  
{
    Vector3i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>(); // sign(D_j)
    Vector3d t_d = t_2_sync_ - t_1_sync_;             // h'
    std::array<bool, 3> joint_motion_finished{};      // motion falgs
    dq.setZero(); // 初始化速度向量

    // 添加时间和同步参数的调试信息
    if (static_cast<int>(t * 10) % 20 == 0) { // 每2秒输出一次
        // std::cout << "t=" << t << ", t_f_sync_: " << t_f_sync_.transpose() << std::endl;
        // std::cout << "delta_q_ abs: " << delta_q_.cwiseAbs().transpose() << std::endl;
    }


    for (size_t i = 0; i < 3; i++) // calculate joint positions
    {
        if (std::abs(delta_q_[i]) < DeltaQMotionFinished){ // target approaches the goal
            (*delta_q_d)[i] = 0;
            joint_motion_finished[i] = true;
            // 添加调试信息
            if (static_cast<int>(t * 10) % 20 == 0) {
                // std::cout << "Joint " << i << " finished due to small delta_q: " << std::abs(delta_q_[i]) << std::endl;
            }
        } 
        else {
            if (t < t_1_sync_[i]) {
                (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] * (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
                dq[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] * (2.0 * t - 3 * t_1_sync_[i]) * std::pow(t, 2.0);
            }
            else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
                (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
                dq[i] = dq_max_sync_[i];
            }
            else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
                (*delta_q_d)[i] = delta_q_[i] + 0.5 *(1.0 / std::pow(t_1_sync_[i], 3.0) *(t - 3.0 * t_1_sync_[i] - t_d[i]) *std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) + (2.0 * t - 3.0 * t_1_sync_[i] - 2.0 * t_d[i])) *dq_max_sync_[i] * sign_delta_q[i];
                dq[i] = (1.0 / std::pow(t_1_sync_[i], 3.0) *(2 * t - 5.0 * t_1_sync_[i] - 2 * t_d[i])*std::pow((t - t_1_sync_[i] - t_d[i]), 2.0) + 1) * dq_max_sync_[i] * sign_delta_q[i];
            }
            else {
                (*delta_q_d)[i] = delta_q_[i];    // reach the goal
                joint_motion_finished[i] = true;
                // 添加调试信息 (仅在第一次到达时输出)
                static std::array<bool, 3> joint_reached = {false, false, false};
                if (!joint_reached[i]) {
                    // std::cout << "Joint " << i << " reached goal at t=" << t << ", t_f_sync_[" << i << "]=" << t_f_sync_[i] << std::endl;
                    joint_reached[i] = true;
                }
            }
        }
    }

    // 添加整体完成状态的调试信息 (仅在状态变化时输出)
    static bool last_all_finished = false;
    bool all_finished = std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),[](bool x) { return x; });
    if (all_finished != last_all_finished) {
        // std::cout << "t=" << t << " Joint status: [" << joint_motion_finished[0] << ", " << joint_motion_finished[1] << ", " << joint_motion_finished[2] << "]";
        // std::cout << " All finished: " << all_finished << std::endl;
        last_all_finished = all_finished;
    }

    return all_finished;
}



void TrajectoryGenerator::calculateSynchronizedValues() 
{
    Vector3d dq_max_reach(dq_max_);
    Vector3d t_f = Vector3d::Zero();
    Vector3d t_1 = Vector3d::Zero();
    Vector3i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();

    // only consider single axis
    for (size_t i = 0; i < 3; i++) {
        if (std::abs(delta_q_[i]) > DeltaQMotionFinished) {
            if ( std::abs(delta_q_[i]) < 3.0 / 2.0 * std::pow(dq_max_[i], 2.0) / ddq_max_[i] ) { // the goal not far enough from start position
                dq_max_reach[i] = std::sqrt( 2.0 / 3.0 * delta_q_[i] * sign_delta_q[i] * ddq_max_[i] ); // recalculate the maximum velocity 
            }
            t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_[i];
            t_f[i] = t_1[i] + std::abs(delta_q_[i]) / dq_max_reach[i];
        } else {
            // 对于小的增量，设置很小的时间以便快速完成
            t_f[i] = 0.01; // 设置一个很小的完成时间
        }
    }

    // take account of the slowest axis
    double max_t_f = t_f.maxCoeff();

    // consider the synchronization of multiple axises
    for (size_t i = 0; i < 3; i++) {
        if (std::abs(delta_q_[i]) > DeltaQMotionFinished) {
            double a = 3.0 / 2.0 * ddq_max_[i];
            double b = -1.0 * max_t_f * std::pow(ddq_max_[i] , 2.0);
            double c = std::abs(delta_q_[i]) * std::pow(ddq_max_[i], 2.0);
            double delta = b * b - 4.0 * a * c;
            if (delta < 0.0) {
                delta = 0.0;
            }
            // according to the area under velocity profile, solve equation "a * Kv^2 + b * Kv + c = 0" for Kv
            dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a); // Kv: maximum synchronization velocity

            t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_[i];
            t_f_sync_[i] =(t_1_sync_)[i] + std::abs(delta_q_[i] / dq_max_sync_[i]);
            t_2_sync_[i] = (t_f_sync_)[i] - t_1_sync_[i];
            q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
        } else {
            // 对于小增量，设置简单的同步参数
            dq_max_sync_[i] = 0.0;
            t_1_sync_[i] = 0.0;
            t_f_sync_[i] = 0.01; // 很快完成
            t_2_sync_[i] = 0.0;
            q_1_[i] = 0.0;
        }
    }

    // 添加调试信息
    // std::cout << "Sync parameters calculated:" << std::endl;
    // std::cout << "t_f_sync_: " << t_f_sync_.transpose() << std::endl;
    // std::cout << "dq_max_sync_: " << dq_max_sync_.transpose() << std::endl;
}



bool TrajectoryGenerator::operator()(const RobotState& robot_state, double time)
{
    time_ = time;

    if (time_ == 0.0) 
    {
        q_start_ = Vector3d(robot_state.q_d.data());
        delta_q_ = q_goal_ - q_start_;
        calculateSynchronizedValues();
        // 添加调试信息
        // std::cout << "TrajectoryGenerator initialized:" << std::endl;
        // std::cout << "q_start_: " << q_start_.transpose() << std::endl;
        // std::cout << "q_goal_: " << q_goal_.transpose() << std::endl;
        // std::cout << "delta_q_: " << delta_q_.transpose() << std::endl;
        // std::cout << "delta_q_ abs: " << delta_q_.cwiseAbs().transpose() << std::endl;
    }

    // Vector3d delta_q_d;
    bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

    // 添加当前状态的调试信息
    if (static_cast<int>(time * 10) % 50 == 0) { // 每5秒输出一次
        // std::cout << "Time: " << time << ", motion_finished: " << motion_finished << std::endl;
        // std::cout << "Current delta_q_ abs: " << delta_q_.cwiseAbs().transpose() << std::endl;
        // std::cout << "DeltaQMotionFinished threshold: " << DeltaQMotionFinished << std::endl;
    }

    std::array<double, 3> joint_positions;
    Eigen::VectorXd::Map(&joint_positions[0], 3) = (q_start_ + delta_q_d);

    return motion_finished;
}

void TrajectoryGenerator::set_dq_max(const std::array<double, 3> d_q_max){
    dq_max_ = Eigen::Map<const Vector3d>(d_q_max.data());
}

void TrajectoryGenerator::set_dqq_max(const std::array<double, 3> d_qq_max){
    ddq_max_ = Eigen::Map<const Vector3d>(d_qq_max.data());
}