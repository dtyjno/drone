#ifndef MYPID_H
#define MYPID_H

#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

#include "Readyaml.h"

class MYPID 
{
public:
    MYPID(){};

    void readPIDParameters(const std::string &filename, const std::string &pid_name)
    {
        YAML::Node config = Readyaml::readYAML(filename);
        kp_ = config[pid_name]["p"].as<double>();
        ki_ = config[pid_name]["i"].as<double>();
        kd_ = config[pid_name]["d"].as<double>();
        output_limit_ = config[pid_name]["outputlimit"].as<double>();
        integral_limit = config[pid_name]["integrallimit"].as<double>();
    };
    float read_goal(const std::string &filename,const std::string &goal_name)
    {
        YAML::Node config = Readyaml::readYAML(filename);
        return config[goal_name].as<float>();
	};
    float velocity_x;
    float velocity_y;
    float velocity_z;
   
    void Mypid(float x_target,float y_target,float z_target,float x_now,float y_now,float z_now,float dt) 
    {
        velocity_x = -compute( x_target, x_now, dt);
        velocity_y = -compute( y_target, y_now, dt);
        velocity_z = compute( z_target, z_now, dt);
    }

    float compute(float setpoint, float measured, float dt) 
    {
        float error = setpoint - measured;
        integral_ += error * dt;
        float derivative = (error - prev_error_) / dt;

        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 限制输出
        
        if (output > output_limit_)
        { 
        output = output_limit_;
        }
        if (output < -output_limit_) 
        {
            output = -output_limit_;
        }
        //限制积分
        if (integral_ > integral_limit)
        {
            integral_ = integral_limit;
        }
        if (integral_ < -integral_limit)
        {
            integral_ = -integral_limit;
        }

        prev_error_ = error;
        return output;
    }
    float kp_, ki_, kd_;
    float output_limit_;
    float integral_limit;

private:
    
    float prev_error_;
    float integral_;
    
};
#endif