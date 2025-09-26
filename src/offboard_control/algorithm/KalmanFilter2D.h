#pragma once

#include <eigen3/Eigen/Eigen>


// 简单的2D卡尔曼滤波器类
class KalmanFilter2D {
public:
    KalmanFilter2D(double process_noise = 0.01, double measurement_noise = 0.1) {
        // 状态向量: [x, y, vx, vy]
        state = Eigen::Vector4d::Zero();
        
        // 状态转移矩阵 (假设恒定速度模型)
        F = Eigen::Matrix4d::Identity();
        
        // 观测矩阵 (只观测位置)
        H = Eigen::Matrix<double, 2, 4>::Zero();
        H(0, 0) = 1.0;  // 观测x
        H(1, 1) = 1.0;  // 观测y
        
        // 过程噪声协方差矩阵
        Q = Eigen::Matrix4d::Identity() * process_noise;
        
        // 测量噪声协方差矩阵
        R = Eigen::Matrix2d::Identity() * measurement_noise;
        
        // 误差协方差矩阵
        P = Eigen::Matrix4d::Identity() * 1.0;
        
        initialized = false;
    }
    
    void update(double x, double y, double dt) {
        if (!initialized) {
            // 首次初始化
            state << x, y, 0.0, 0.0;
            initialized = true;
            last_time = std::chrono::steady_clock::now();
            return;
        }
        
        // 更新状态转移矩阵的时间相关部分
        F(0, 2) = dt;  // x = x + vx * dt
        F(1, 3) = dt;  // y = y + vy * dt
        
        // 预测步骤
        state = F * state;
        P = F * P * F.transpose() + Q;
        
        // 更新步骤
        Eigen::Vector2d measurement(x, y);
        Eigen::Vector2d innovation = measurement - H * state;
        Eigen::Matrix2d S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
        
        state = state + K * innovation;
        P = (Eigen::Matrix4d::Identity() - K * H) * P;
    }
    
    double getX() const { return state(0); }
    double getY() const { return state(1); }
    double getVX() const { return state(2); }
    double getVY() const { return state(3); }
    
    bool isInitialized() const { return initialized; }
    
private:
    Eigen::Vector4d state;           // 状态向量 [x, y, vx, vy]
    Eigen::Matrix4d F;               // 状态转移矩阵
    Eigen::Matrix<double, 2, 4> H;   // 观测矩阵
    Eigen::Matrix4d Q;               // 过程噪声协方差
    Eigen::Matrix2d R;               // 测量噪声协方差
    Eigen::Matrix4d P;               // 误差协方差矩阵
    bool initialized;
    std::chrono::steady_clock::time_point last_time;
};