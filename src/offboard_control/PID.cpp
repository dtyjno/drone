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

// void PID::set_pid_info()
// {
//     _pid_info._kP = _kp;
//     _pid_info._kI = _ki;
//     _pid_info._kD = _kd;
//     _pid_info.P = 0;
//     _pid_info.I = 0;
//     _pid_info.D = 0;
//     _pid_info.Dmod = 0.0f;
//     _pid_info.FF = 0;
//     _pid_info.DFF = 0;
//     // _pid_info.slew_rate = srmax;
//     // _pid_info.limit = false;
//     // _pid_info.PD_limit = false;
//     // _pid_info.reset = false;
//     // _pid_info.I_term_set = false;

//     _pid_info.error = 0.0f;
//     _pid_info.target = 0.0f;
//     _pid_info.actual = 0.0f;
//     // _pid_info.reset = false;
//     // _pid_info.PD_limit = false;
//     // _pid_info.limit = false;
//     // _pid_info.slew_rate = srmax;
// }

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
    if(use_increment == true)
    {
        printf("PID%s: update_all_increment called with measurement: %f, target: %f, dt: %f, limit: %f\n", pid_name.c_str(), measurement, target, dt, limit);
    }
    else
    {
        printf("PID%s: update_all called with measurement: %f, target: %f, dt: %f, limit: %f\n", pid_name.c_str(), measurement, target, dt, limit);
    }
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
            _pid_info.D = _derivative * _pid_info._kD;
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
    printf("PID%s: kp:%+5f ki:%+5f kd:%+5f\n", pid_name.c_str(), _pid_info._kP, _pid_info._kI, _pid_info._kD);
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
    // 检查Ki和dt的有效性
    if (is_zero(_pid_info._kI) || !is_positive(dt))
    {
        _pid_info.I = 0.0f;
        _pid_info.limit = (limit > 0);
        return;
    }
    
    // 计算基本积分项增量
    float integral_increment = _error * _pid_info._kI * dt;
    // printf("PID%s: integral_increment:%+10f, dt:%+10f, kI:%+10f\n", pid_name.c_str(), integral_increment, dt, _pid_info._kI);

    if ((_pid_info.I >= 0 && _error <= 0) || (_pid_info.I <= 0 && _error >= 0))
    {
        _pid_info.I *= 0.8f;
    }

    // 如果没有输出限制，直接更新积分项
    if (limit <= 0)
    {
        _pid_info.I += integral_increment;
        _pid_info.I = constrain_float(_pid_info.I, _kimax, -_kimax);
        _pid_info.limit = false;
        return;
    }
    
    // 计算当前输出的饱和状态
    float saturated_output = constrain_float(_pid_info.output, limit, -limit);
    bool is_saturated = (fabs(_pid_info.output - saturated_output) > 1e-6);
    
    // 抗饱和处理
    if (is_saturated)
    {
        // 输出饱和时的抗积分饱和策略
        if ((_pid_info.output > limit && _error > 0) || (_pid_info.output < -limit && _error < 0))
        {
            // 如果误差会使饱和更严重，则不增加积分项
            // 但允许积分项朝减少饱和的方向变化
            if ((_pid_info.I > 0 && _error < 0) || (_pid_info.I < 0 && _error > 0))
            {
                _pid_info.I += integral_increment;
            }
            // 否则保持积分项不变或使用条件积分
            else
            {
                // 使用条件积分：如果积分项很大，允许其缓慢衰减
                if (fabs(_pid_info.I) > _kimax * 0.9f)
                {
                    _pid_info.I *= 0.97f; // 缓慢衰减
                }
            }
        }
        else
        {
            // 误差有助于减少饱和，正常更新积分项
            _pid_info.I += integral_increment;
        }
        
        // 使用抗饱和反馈（基于饱和量）
        if (!is_zero(_pid_info._kP))
        {
            float kb = _pid_info._kI / _pid_info._kP; // 抗饱和增益
            float saturation_error = _pid_info.output - saturated_output;
            float anti_windup_correction = -saturation_error * kb * dt;
            _pid_info.I += anti_windup_correction;
            // printf("PID%s: Anti-windup correction: %f\n", pid_name.c_str(), anti_windup_correction);
        }
    }
    else
    {
        // 输出未饱和，正常更新积分项
        _pid_info.I += integral_increment;
    }
    
    // 限制积分项幅值
    _pid_info.I = constrain_float(_pid_info.I, _kimax, -_kimax);
    
    // 积分项衰减机制（当误差接近零时）
    if (fabs(_error) < 0.020f && fabs(_pid_info.I) > 0.010f)
    {
        // printf("PID%s: Integral decay applied: %f\n", pid_name.c_str(), _pid_info.I);
        _pid_info.I *= 0.97f; // 轻微衰减，避免长期偏差
    }
    
    // 设置限制标志
    _pid_info.limit = (limit > 0);
    
#ifdef pid_debug_print
    if (is_saturated)
    {
        printf("PID%s: SATURATED - output:%f, limit:%f, I:%f\n", 
               pid_name.c_str(), _pid_info.output, limit, _pid_info.I);
    }
#endif
}

void PID::print_update_info()
{
    printf("PID%s:tar:%+10f mea:%+5f err:%+5f P:%+10f I:%+10f D:%+10f Out:%f _MAX:%f\n", pid_name.c_str(), _pid_info.target, _pid_info.actual, _pid_info.error, _pid_info.P, _pid_info.I, _pid_info.D, _pid_info.output, _pid_info.output_max);
}

// 重置积分项
void PID::reset_I()
{
    _pid_info.I = 0.0f;
    _integrator = 0.0f;
}

// 重置所有PID状态
void PID::reset_all()
{
    _pid_info.I = 0.0f;
    _pid_info.D = 0.0f;
    _pid_info.P = 0.0f;
    _pid_info.output = 0.0f;
    _integrator = 0.0f;
    _error = 0.0f;
    _derivative = 0.0f;
    _pid_info.Dmod = 0.0f;
    
    // 重置历史数据
    _history_index = 0;
    _history_full = false;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        _error_history[i] = 0.0f;
        _time_history[i] = 0.0f;
    }
}

// 获取积分项值
float PID::get_integrator() const
{
    return _pid_info.I;
}

// 设置积分项值（用于外部控制）
void PID::set_integrator(float integrator)
{
    _pid_info.I = constrain_float(integrator, _kimax, -_kimax);
    _integrator = _pid_info.I;
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
