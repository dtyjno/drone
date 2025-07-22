#include "PID.h"
#include <cmath>
#include "math.h"

#ifndef MATH_H
bool is_zero(float _ki)
{
    return _ki == 0;
}
bool is_positive(float dt)
{
    return dt >= 0;
}
bool is_negative(float _error)
{
    return _error <= 0;
}
template <typename T>
bool is_equal(T a, T b, T tolerance = 0.001)
{
    return std::fabs(a - b) < tolerance;
}

float constrain_float(float _integrator, float _max, float _min)
{
    if (_integrator > _max)
    {
        return _max;
    }
    else if (_integrator < _min)
    {
        return _min;
    }
    else
    {
        return _integrator;
    }
}
#endif

PID::PID(std::string pid_name, float kp, float ki, float kd, float kff, float kdff, float kimax, float srmax)
{
    this->pid_name = pid_name;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _kff = kff;
    _kdff = kdff;
    _kimax = kimax;
    _integrator = 0.0f;
    _error = 0.0f;
    _derivative = 0.0f;
    
    // 初始化历史数据
    _history_index = 0;
    _history_full = false;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        _error_history[i] = 0.0f;
        _time_history[i] = 0.0f;
    }
    
    _pid_info._kP = _kp;
    _pid_info._kI = _ki;
    _pid_info._kD = _kd;
    _pid_info.P = 0.0f;
    _pid_info.I = 0.0f;
    _pid_info.D = 0.0f;
    _pid_info.FF = 0.0f;
    _pid_info.DFF = 0.0f;
    _pid_info.error = 0.0f;
    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    _pid_info.reset = false;
    _pid_info.PD_limit = false;
    _pid_info.slew_rate = srmax;
    _pid_info.limit = false;
    _pid_info.Dmod = 0.0f;
}

PID::PID(float kp, float ki, float kd, float kff, float kdff, float kimax, float srmax)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _kff = kff;
    _kdff = kdff;
    _kimax = kimax;
    _integrator = 0.0f;
    _error = 0.0f;
    _derivative = 0.0f;
    
    // 初始化历史数据
    _history_index = 0;
    _history_full = false;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        _error_history[i] = 0.0f;
        _time_history[i] = 0.0f;
    }
    
    _pid_info._kP = _kp;
    _pid_info._kI = _ki;
    _pid_info._kD = _kd;
    _pid_info.P = 0.0f;
    _pid_info.I = 0.0f;
    _pid_info.D = 0.0f;
    _pid_info.FF = 0.0f;
    _pid_info.DFF = 0.0f;
    _pid_info.error = 0.0f;
    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    _pid_info.reset = false;
    _pid_info.PD_limit = false;
    _pid_info.slew_rate = srmax;
    _pid_info.limit = false;
    _pid_info.Dmod = 0.0f;
}

PID::PID(const PID::Defaults &defaults)
{
    _kp = defaults.p;
    _ki = defaults.i;
    _kd = defaults.d;
    _kff = defaults.ff;
    _kdff = defaults.dff;
    _kimax = defaults.imax;
    _integrator = 0.0f;
    _error = 0.0f;
    _derivative = 0.0f;
    
    // 初始化历史数据
    _history_index = 0;
    _history_full = false;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        _error_history[i] = 0.0f;
        _time_history[i] = 0.0f;
    }
    
    _pid_info._kP = _kp;
    _pid_info._kI = _ki;
    _pid_info._kD = _kd;
    _pid_info.P = 0.0f;
    _pid_info.I = 0.0f;
    _pid_info.D = 0.0f;
    _pid_info.FF = 0.0f;
    _pid_info.DFF = 0.0f;
    _pid_info.error = 0.0f;
    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    _pid_info.reset = false;
    _pid_info.PD_limit = false;
    _pid_info.slew_rate = defaults.srmax;
    _pid_info.limit = false;
    _pid_info.Dmod = 0.0f;
}

void PID::set_pid_info()
{
    _pid_info._kP = _kp;
    _pid_info._kI = _ki;
    _pid_info._kD = _kd;
    _pid_info.P = 0;
    _pid_info.I = 0;
    _pid_info.D = 0;
    _pid_info.Dmod = 0.0f;
    _pid_info.FF = 0;
    _pid_info.DFF = 0;
    // _pid_info.slew_rate = srmax;
    // _pid_info.limit = false;
    // _pid_info.PD_limit = false;
    // _pid_info.reset = false;
    // _pid_info.I_term_set = false;

    _pid_info.error = 0.0f;
    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    // _pid_info.reset = false;
    // _pid_info.PD_limit = false;
    // _pid_info.limit = false;
    // _pid_info.slew_rate = srmax;
}

void PID::set_gains(const PID::Defaults &defaults)
{
    _pid_info._kP = defaults.p;
    _pid_info._kI = defaults.i;
    _pid_info._kD = defaults.d;

    // _pid_info.P = 0;
    // _pid_info.I = 0;
    // _pid_info.D = 0;
    // _pid_info.FF = 0;
    // _pid_info.DFF = 0;

    // _integrator = 0.0f;
    // _error = 0.0f;
    // _derivative = 0.0f;
    // _pid_info.P = 0.0f;
    // _pid_info.I = 0.0f;
    // _pid_info.D = 0.0f;
    // _pid_info.FF = 0.0f;
    // _pid_info.DFF = 0.0f;
    // _pid_info.error = 0.0f;
    // _pid_info.target = 0.0f;
    // _pid_info.actual = 0.0f;
    // _pid_info.reset = false;
    // _pid_info.PD_limit = false;
    // _pid_info.slew_rate = defaults.srmax;
    // _pid_info.limit = false;
    // _pid_info.Dmod = 0.0f;
}

void PID::set_gains(float kp, float ki, float kd)
{
    _pid_info._kP = kp;
    _pid_info._kI = ki;
    _pid_info._kD = kd;
    // _pid_info.P = 0;
    // _pid_info.I = 0;
    // _pid_info.D = 0;
}

void PID::set_pid(const PID::Defaults &defaults)
{
    _kp = defaults.p;
    _ki = defaults.i;
    _kd = defaults.d;
    _kff = defaults.ff;
    _kdff = defaults.dff;
    _kimax = defaults.imax;
}


void PID::set_pid(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::get_pid(float &kp, float &ki, float &kd)
{
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

#include <iostream>
float PID::update_all(float measurement, float target, float dt, float limit, float velocity, bool use_increment)
{
    // 计算当前时间（累积时间）
    static float current_time = 0.0f;
    current_time += dt;
    
#ifdef pid_debug_print
    // printf("p:%3.2f i:%3.2f d:%3.2f ", _pid_info._kP, _pid_info._kI, _pid_info._kD);
#endif
    _pid_info.target = target;
    _pid_info.actual = measurement;
    _pid_info.output_max = limit;
    // Calculate the error
    _error = target - measurement;
    if(use_increment == true)
    {
        
        _pid_info.last_last_error = _pid_info.last_error;
        _pid_info.last_error = _pid_info.error;
        _pid_info.error = _error;
    }
    else
    {
        _pid_info.error = _error; // Absolute error
    }
   

#ifdef fuzzy_pid_dead_zone
    float dead_zone = 100.0f;
    if (_error < dead_zone && _error > -dead_zone)
    {
        _error = 0;
    }
    else
    {
        if (_error > dead_zone)
            _error = _error - dead_zone;
        else
        {
            if (_error < -dead_zone)
                _error = _error + dead_zone;
        }
    }
#endif

    // Calculate the proportional term
    if (use_increment == true)
    {
         _pid_info.P = (_error - _pid_info.last_error) * _pid_info._kP;
    } else
    {
         _pid_info.P = _error * _pid_info._kP;
    }

    // Calculate the integral term
    update_i(dt, limit);

    // Calculate the derivative term
    if (isfinite(velocity))
    {
        if (use_increment == true)
        {
            _derivative = ((_error - 2 * _pid_info.last_error + _pid_info.last_last_error) / dt);
        } else
        {
            _derivative = -velocity;
            _pid_info.D = velocity * _pid_info._kD;
        }

    }
    else
    {
        // 使用改进的微分计算方法（基于历史数据）
        _derivative = calculate_improved_derivative(current_time, dt);
        
#ifdef pid_debug_print
        printf("PID%s: improved_deri:%10.6f, ", pid_name.c_str(), _derivative);
#endif
        if (!is_equal(_derivative, 0.0f, 0.0001f))
        {
            _pid_info.D = _derivative * _pid_info._kD;
        }
        else
        {
            _pid_info.D = 0.0f;
        }
    }
    // Calculate the feed forward term
    _pid_info.FF = target * _kff;

    // Calculate the derivative feed forward term
    _pid_info.DFF = _derivative * _kdff;

    // Calculate the total output
    _pid_info.output = _pid_info.P + _pid_info.I + _pid_info.D + _pid_info.FF + _pid_info.DFF;

    // Update the Dmod value
    _pid_info.Dmod = _error;

    // Set the target value
    _pid_info.target = target;

    // Set the actual value
    _pid_info.actual = measurement;

    // Set the reset flag
    _pid_info.reset = false;

    // Set the PD limit flag
    _pid_info.PD_limit = false;

    // Set the slew rate
    _pid_info.slew_rate = srmax;
#ifdef pid_debug_print
    printf("PID%s: tar:%+10f mea:%+5f kp:%+5f ki:%+5f kd:%+5f\n", pid_name.c_str(), target, measurement, _pid_info._kP, _pid_info._kI, _pid_info._kD);
    printf("PID%s: err:%+5f P:%+10f I:%+10f D:%+10f Out:%f\n", pid_name.c_str(), _pid_info.error, _pid_info.P, _pid_info.I, _pid_info.D, _pid_info.output);
// std::cout <<"target:"<<target<<" meadurement:"<<measurement<<" error:"<<_pid_info.error <<" P:"
// <<_pid_info.P<<" I:"
// <<_pid_info.I<<" D:"
// <<_pid_info.D<<" Out:"
// << output <<std::endl;
#endif
    // Limit the output
    if (limit > 0)
    {
        if (_pid_info.output > limit)
        {
            return limit;
        }
        else if (_pid_info.output < -limit)
        {
            return -limit;
        }
    }

    _pid_info.last_output = _pid_info.output;
    return _pid_info.output;
}

float PID::update_all_increment(float measurement, float target, float dt, float limit)
{
#ifdef pid_debug_print
    // printf("p:%3.2f i:%3.2f d:%3.2f ", _pid_info._kP, _pid_info._kI, _pid_info._kD);
#endif
    
    _pid_info.target = target;
    _pid_info.actual = measurement;
    _pid_info.output_max = limit;
    // Calculate the error
    _error = target - measurement;
    _pid_info.error = _error;
    _pid_info.last_last_error = _pid_info.last_error;
    _pid_info.last_error = _pid_info.error;
    _pid_info.last_output = 0.0f;
    static float output = 0.0f;
#ifdef fuzzy_pid_dead_zone
    float dead_zone = 100.0f;
    if (_error < dead_zone && _error > -dead_zone)
    {
        _error = 0;
    }
    else
    {
        if (_error > dead_zone)
            _error = _error - dead_zone;
        else
        {
            if (_error < -dead_zone)
                _error = _error + dead_zone;
        }
    }
#endif
    // Calculate the proportional term
    _pid_info.P = (_error - _pid_info.last_error) * _pid_info._kP;

    // Calculate the integral term
    update_i(dt, limit);

    // Calculate the derivative term
    _derivative = ((_error - 2 * _pid_info.last_error + _pid_info.last_last_error) / dt);
    
    // Calculate the feed forward term
    _pid_info.FF = target * _kff;

    // Calculate the derivative feed forward term
    _pid_info.DFF = _derivative * _kdff;

    // Calculate the total output
    _pid_info.output_increment = _pid_info.P + _pid_info.I + _pid_info.D + _pid_info.FF + _pid_info.DFF;
    output = _pid_info.last_output + _pid_info.output_increment;
    // Update the Dmod value
    _pid_info.Dmod = _error;

    // Set the target value
    _pid_info.target = target;

    // Set the actual value
    _pid_info.actual = measurement;

    // Set the reset flag
    _pid_info.reset = false;

    // Set the PD limit flag
    _pid_info.PD_limit = false;

    // Set the slew rate
    _pid_info.slew_rate = srmax;
#ifdef pid_debug_print
    printf("PID_INCRECEMENT: tar:%+10f mea:%+5f kp:%+5f ki:%+5f kd:%+5f\n", target, measurement, _pid_info._kP, _pid_info._kI, _pid_info._kD);
    printf("PID_INCRECEMENT: err:%+5f P:%+10f I:%+10f D:%+10f Out:%f\n",  _pid_info.error, _pid_info.P, _pid_info.I, _pid_info.D, _pid_info.output_increment);
// std::cout <<"target:"<<target<<" meadurement:"<<measurement<<" error:"<<_pid_info.error <<" P:"
// <<_pid_info.P<<" I:"
// <<_pid_info.I<<" D:"
// <<_pid_info.D<<" Out:"
// << output <<std::endl;
#endif
    // Limit the output
    if (limit > 0)
    {
        if (_pid_info.output_increment > limit)
        {
            return limit;
        }
        else if (_pid_info.output_increment < -limit)
        {
            return -limit;
        }
    }

    _pid_info.last_output = output;

    return _pid_info.output_increment = output;
}

//  update_i - update the integral
//  If the limit flag is set the integral is only allowed to shrink
void PID::update_i(float dt, float limit)
{
    if (!is_zero(_ki) && is_positive(dt))
    {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!limit || (_pid_info.I <= 0 && _error <= 0) || (_pid_info.I >= 0 && _error >= 0))
        {
            // Update the integrator

            //     float _kb = _pid_info._kI / _pid_info._kP;
            //     // 计算抗饱和项
            //     float anti_windup = (_pid_info.output - constrain_float(_pid_info.output, _pid_info.output_max, -_pid_info.output_max)) * _kb;
            // 更新积分项，加入抗饱和项
            _pid_info.I += ((float)_error * _pid_info._kI) * dt;
            // Limit the integrator
            _pid_info.I = constrain_float(_pid_info.I, _kimax, -_kimax);
        }
        else
        {
            // Reset the integrator
            _pid_info.I = 0.0f;
        }
    }
    else
    {
        _pid_info.I = 0.0f;
    }
    // _pid_info.I = _integrator;
    _pid_info.limit = limit;

    // // Set I set flag for logging and clear
    // _pid_info.I_term_set = _flags._I_set;
    // _flags._I_set = false;
}

void PID::print_update_info()
{
    printf("PID%s:tar:%+10f mea:%+5f err:%+5f P:%+10f I:%+10f D:%+10f Out:%f _MAX:%f\n", pid_name.c_str(), _pid_info.target, _pid_info.actual, _pid_info.error, _pid_info.P, _pid_info.I, _pid_info.D, _pid_info.output, _pid_info.output_max);
}
/**
 * 使用滑动平均滤波器平滑数据。
 *
 * @param current_value 当前的数据点。
 * @param alpha 平滑因子，决定了新数据点在平滑过程中的权重。
 * @return 平滑后的数据值。
 */
float PID::smooth_data(float current_value, float alpha)
{
    // 假设这是一个全局变量，用于存储上一次的平滑值
    static float last_smoothed_value = 0;
    // 计算平滑后的值
    float smoothed_value = alpha * current_value + (1 - alpha) * last_smoothed_value;

    // 更新上一次的平滑值
    last_smoothed_value = smoothed_value;

    return smoothed_value;
}

// 使用历史数据计算改进的微分项
float PID::calculate_improved_derivative(float current_time, float dt)
{
    // 更新历史数据
    _error_history[_history_index] = _error;
    _time_history[_history_index] = current_time;
    
    // 移动到下一个索引
    _history_index = (_history_index + 1) % HISTORY_SIZE;
    if (!_history_full && _history_index == 0) {
        _history_full = true;
    }
    
    // 如果历史数据不足，使用传统方法
    if (!_history_full) {
        return -(_pid_info.Dmod - _error) / dt;
    }
    
    // 使用最小二乘法拟合直线来计算微分
    // 计算时间和误差的平均值
    float sum_time = 0.0f, sum_error = 0.0f;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        sum_time += _time_history[i];
        sum_error += _error_history[i];
    }
    float mean_time = sum_time / HISTORY_SIZE;
    float mean_error = sum_error / HISTORY_SIZE;
    
    // 计算斜率（微分）
    float numerator = 0.0f, denominator = 0.0f;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        float time_diff = _time_history[i] - mean_time;
        float error_diff = _error_history[i] - mean_error;
        numerator += time_diff * error_diff;
        denominator += time_diff * time_diff;
    }
    
    // 避免除零
    if (fabs(denominator) < 1e-6) {
        return -(_pid_info.Dmod - _error) / dt;
    }
    
    // 返回负的斜率（因为我们需要 -d(error)/dt）
    return -(numerator / denominator);
}

#ifndef POSCONTROL_H

#define POSCONTROL_XY_P 0.8f    // horizontal velocity controller P gain default 0.5
#define POSCONTROL_XY_I 0.3f    // horizontal velocity controller I gain default 0.2
#define POSCONTROL_XY_D 0.1f    // horizontal velocity controller D gain default 0.1
#define POSCONTROL_XY_IMAX 1.0f // horizontal velocity controller IMAX gain default
#endif

// int main()
// {
//     PID pid = PID(
//         POSCONTROL_XY_P,
//         POSCONTROL_XY_I,
//         POSCONTROL_XY_D,
//         0,
//         0,
//         POSCONTROL_XY_IMAX,
//         0);
//     float now = 0;
//     float target = 50;
//     pid.set_pid_info();
//     for (int i = 0; i < 100; i++)
//     {
//         now += pid.update_all(now, target, 1, 30);
//     }
//     return 0;
// }
