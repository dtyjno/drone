#ifndef PID_H
#define PID_H
#include <cfloat>
#define DEFAULT_VELOCITY FLT_MAX
class PID{
    public:

    struct Defaults {
        float p=0;
        float i=0;
        float d=0;
        float ff=0;
        float dff=0;
        float imax=10;
        // bool _use_vel = true;
        float filt_T_hz =0;
        float filt_E_hz =0;
        float filt_D_hz =0;
        float srmax =0;
        float srtau =0;
    };

    PID(float kp, float ki, float kd, float kff=0, float kdff=0, float kimax=1000,float srmax=0);
    PID(const PID::Defaults &defaults);
    PID(){};
    
    void set_pid_info();

    void set_gains(float kp, float ki, float kd);
    float update_all(float measurement, float target, float dt, float limit, float velocity = DEFAULT_VELOCITY);
    void update_i(float dt, bool limit);
    
float smooth_data(float current_value, float alpha);

    // internal variables
    float _integrator;        // integrator value
    float _target;            // target value to enable filtering
    float _error;             // error value to enable filtering
    float _derivative;        // derivative value to enable filtering

    struct PIDInfo {
        float target;
        float actual;
        float error;
        float P;
        float I;
        float D;
        float FF;
        float DFF;
        float Dmod;
        float slew_rate;
        bool limit;
        bool PD_limit;
        bool reset;
        bool I_term_set;
    };
    PIDInfo _pid_info;

    private:
        float _kp, _ki, _kd;
        float _kff;
        float _kimax;
        float filt_T_hz;
        float filt_E_hz;
        float filt_D_hz;
        float srmax;
        float srtau;
        float _kdff;
};

#endif // PID_H