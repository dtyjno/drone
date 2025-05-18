#include "PID.h"
#include "math.h"

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

PID::PID(const PID::Defaults& defaults){
    _kp = defaults.p;
    _ki = defaults.i;
    _kd = defaults.d;
    _kff = defaults.ff;
    _kdff = defaults.dff;
    _kimax = defaults.imax;
    _integrator = 0.0f;
    _error = 0.0f;
    _derivative = 0.0f;
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

void PID::set_pid_info(){
    _pid_info.P = _kp;
    _pid_info.I = _ki;
    _pid_info.D = _kd;
    _pid_info.FF = _kff;
    _pid_info.DFF = _kdff;
    _pid_info.slew_rate = srmax;
    _pid_info.limit = false;
    _pid_info.PD_limit = false;
    _pid_info.reset = false;
    _pid_info.I_term_set = false;

    _pid_info.error = 0.0f;
    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    _pid_info.reset = false;
    _pid_info.PD_limit = false;
    _pid_info.limit = false;
    _pid_info.Dmod = 0.0f;
    _pid_info.slew_rate = srmax;
}

void PID::set_gains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

#include <iostream>
float PID::update_all(float measurement, float target, float dt, float limit, float velocity){
    // Calculate the error
    _error = target - measurement;
    _pid_info.error = _error;

    // Calculate the proportional term
    _pid_info.P = _error * _kp;

    // Calculate the integral term
    update_i(dt, limit);

    // Calculate the derivative term
    if(!is_equal(velocity,DEFAULT_VELOCITY)){
        _derivative = - velocity;
        _pid_info.D = velocity * _kd;
    }else{//timer_callback 250ms
        _derivative = (_pid_info.Dmod - _error) / dt;
        if(!is_equal(_derivative, 0.0f, 0.0001f)){
            _pid_info.D = _derivative * _kd;
        }
    }
    // Calculate the feed forward term
    _pid_info.FF = target * _kff;

    // Calculate the derivative feed forward term
    _pid_info.DFF = _derivative * _kdff;

    // Calculate the total output
    float output = _pid_info.P + _integrator + _pid_info.D + _pid_info.FF + _pid_info.DFF;

    // Limit the output
    if (limit > 0) {
        if (output > limit) {
            output = limit;
        } else if (output < -limit) {
            output = -limit;
        }
    }

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

    printf("target:%10f meadurement:%10f error:%10f P:%10f I:%10f D:%10f Out:%f\n",target,measurement,_pid_info.error,_pid_info.P,_pid_info.I,_pid_info.D,output);
    // std::cout <<"target:"<<target<<" meadurement:"<<measurement<<" error:"<<_pid_info.error <<" P:"
    // <<_pid_info.P<<" I:"
    // <<_pid_info.I<<" D:"
    // <<_pid_info.D<<" Out:"
    // << output <<std::endl;
    return output;

}

//  update_i - update the integral
//  If the limit flag is set the integral is only allowed to shrink
void PID::update_i(float dt, bool limit)
{
    if (!is_zero(_ki) && is_positive(dt)) {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * dt;
            _integrator = constrain_float(_integrator, _kimax, -_kimax);
            // std::cout <<_error <<" ((float)_error * _ki) * dt"<<((float)_error * _ki) * dt<<" I:"<<_integrator<<std::endl;
        }
        //
        //
    } else {
        _integrator = 0.0f;
    }
    _pid_info.I = _integrator;
    _pid_info.limit = limit;

    // // Set I set flag for logging and clear
    // _pid_info.I_term_set = _flags._I_set;
    // _flags._I_set = false;
}


/**
 * 使用滑动平均滤波器平滑数据。
 * 
 * @param current_value 当前的数据点。
 * @param alpha 平滑因子，决定了新数据点在平滑过程中的权重。
 * @return 平滑后的数据值。
 */
float PID::smooth_data(float current_value, float alpha) {
    // 假设这是一个全局变量，用于存储上一次的平滑值
    static float last_smoothed_value = 0;
    // 计算平滑后的值
    float smoothed_value = alpha * current_value + (1 - alpha) * last_smoothed_value;
    
    // 更新上一次的平滑值
    last_smoothed_value = smoothed_value;
    
    return smoothed_value;
}